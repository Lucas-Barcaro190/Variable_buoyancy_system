# simulation/emulator/virtual_mcu.py
"""
100% Pure Python RP2040 Virtual MCU Emulator for the Variable Buoyancy System.
Re-creates the 20Hz PID Control Loop, Velocity Profile Generator, and System Failsafe logic.
"""

import time
import math
import threading
from simulation.config import sim_config

class PIDController:
    """Discrete PID Controller matching src/motor/motor_control.cpp."""
    
    def __init__(self, Kp=1.5, Ki=0.05, Kd=0.01, out_min=-sim_config.DEFAULT_VMAX_MM_S, out_max=sim_config.DEFAULT_VMAX_MM_S):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement, dt):
        if dt <= 0.0:
            return 0.0
            
        error = setpoint - measurement
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup clamping
        self.integral += error * dt
        I = self.Ki * self.integral
        if I > self.out_max:
            I = self.out_max
            if self.Ki != 0.0:
                self.integral = I / self.Ki
        elif I < self.out_min:
            I = self.out_min
            if self.Ki != 0.0:
                self.integral = I / self.Ki
                
        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        self.prev_error = error
        
        # Saturated total output
        output = P + I + D
        output = max(self.out_min, min(self.out_max, output))
        return output

class VelocityGenerator:
    """Trapezoidal / Triangular S-curve motion profile generator matching velocity_generator.cpp."""
    
    def __init__(self):
        self.reset()

    def reset(self):
        self.h_start = 0.0
        self.h_target = 0.0
        self.t_start_sec = 0.0
        self.vmax = sim_config.DEFAULT_VMAX_MM_S
        self.accel_time = sim_config.DEFAULT_ACCEL_TIME_S
        self.a_max = self.vmax / self.accel_time
        self.total_distance = 0.0
        self.dir = 1.0
        self.is_triangular = False
        self.t1 = 0.0
        self.t2 = 0.0
        self.t3 = 0.0
        self.t_total = 0.0
        self.v_peak = 0.0
        self.active = False
        self.is_deadband = False

    def start(self, h_start, h_target, t_now_sec, vmax=sim_config.DEFAULT_VMAX_MM_S, accel_time=3.0):
        self.h_start = h_start
        self.h_target = h_target
        self.t_start_sec = t_now_sec
        
        delta_h = abs(h_target - h_start)
        self.total_distance = delta_h
        
        # Deadband check (< 0.15 mm)
        if delta_h < sim_config.VELOCITY_DEADBAND_MM:
            self.active = False
            self.is_deadband = True
            self.t_total = 0.0
            self.v_peak = 0.0
            return
            
        self.is_deadband = False
        self.dir = 1.0 if h_target >= h_start else -1.0
        
        accel_time = max(2.0, min(5.0, accel_time))
        self.accel_time = accel_time
        self.vmax = vmax if vmax > 0.0 else sim_config.DEFAULT_VMAX_MM_S
        self.a_max = self.vmax / self.accel_time
        
        d_accel_decel = (self.vmax * self.vmax) / self.a_max
        
        if delta_h < d_accel_decel:
            # Triangular profile for short displacements
            self.is_triangular = True
            self.v_peak = math.sqrt(self.a_max * delta_h)
            self.t1 = self.v_peak / self.a_max
            self.t2 = 0.0
            self.t3 = self.t1
            self.t_total = self.t1 + self.t3
        else:
            # Trapezoidal profile for long displacements
            self.is_triangular = False
            self.v_peak = self.vmax
            self.t1 = self.accel_time
            d_cruise = delta_h - d_accel_decel
            self.t2 = d_cruise / self.vmax
            self.t3 = self.accel_time
            self.t_total = self.t1 + self.t2 + self.t3
            
        self.active = True

    def update(self, t_now_sec):
        if self.is_deadband or not self.active:
            return self.h_target, 0.0, True
            
        t = t_now_sec - self.t_start_sec
        
        if t <= 0.0:
            return self.h_start, 0.0, False
            
        if t >= self.t_total:
            self.active = False
            return self.h_target, 0.0, True
            
        if self.is_triangular:
            if t <= self.t1:
                vref = self.dir * (self.a_max * t)
                href = self.h_start + self.dir * (0.5 * self.a_max * t * t)
            else:
                t_prime = t - self.t1
                vref = self.dir * (self.v_peak - self.a_max * t_prime)
                d_accel = 0.5 * self.a_max * self.t1 * self.t1
                href = self.h_start + self.dir * (d_accel + self.v_peak * t_prime - 0.5 * self.a_max * t_prime * t_prime)
        else:
            if t <= self.t1:
                vref = self.dir * (self.a_max * t)
                href = self.h_start + self.dir * (0.5 * self.a_max * t * t)
            elif t <= (self.t1 + self.t2):
                t_prime = t - self.t1
                vref = self.dir * self.vmax
                d_accel = 0.5 * self.a_max * self.t1 * self.t1
                href = self.h_start + self.dir * (d_accel + self.vmax * t_prime)
            else:
                t_double_prime = t - (self.t1 + self.t2)
                vref = self.dir * (self.vmax - self.a_max * t_double_prime)
                d_accel = 0.5 * self.a_max * self.t1 * self.t1
                d_cruise = self.vmax * self.t2
                href = self.h_start + self.dir * (d_accel + d_cruise + self.vmax * t_double_prime - 0.5 * self.a_max * t_double_prime * t_double_prime)
                
        return href, vref, False

class VirtualMCUEmulator:
    """Virtual RP2040 Microcontroller executing FreeRTOS motor task loop."""
    
    def __init__(self, shared_state, plant_model):
        self.state = shared_state
        self.plant = plant_model
        
        self.pid = PIDController()
        self.vel_gen = VelocityGenerator()
        
        self.mctl_state = sim_config.MCTL_IDLE
        self.target_pot = sim_config.MINIMAL_THRESHOLD
        self.pulses_remaining = 0
        self.pulse_direction = 0
        
        self.running = False
        self.mcu_thread = None
        self.start_time = time.time()
        self.state.sys_state = sim_config.SYS_OPERATIONAL

    def execute_command(self, cmd_type, target_pot=None, pulses=0, direction=0):
        """Enqueues a new motor control command."""
        self.state.commands_queued += 1
        self.mctl_state = cmd_type
        
        h_medido = self.plant.pot_to_piston_pos(self.state.potentiometer_raw)
        now_sec = time.time() - self.start_time
        
        if cmd_type in (sim_config.MCTL_MOVING_UNTIL_POT, sim_config.MCTL_MOVING_ABSOLUTE):
            if target_pot is not None:
                self.target_pot = target_pot
                self.state.target_pot_value = target_pot
            h_target = self.plant.pot_to_piston_pos(self.target_pot)
            self.vel_gen.start(h_medido, h_target, now_sec)
            self.pid.reset()
            if self.state.verbose_level >= 1:
                print(f"[VirtualMCU] New command: type={cmd_type}, target_pot={self.target_pot} ({h_target:.2f} mm)")
                
        elif cmd_type == sim_config.MCTL_MOVING_PULSES:
            self.pulses_remaining = pulses
            self.pulse_direction = direction
            if self.state.verbose_level >= 1:
                print(f"[VirtualMCU] Move pulses: {pulses}, dir={direction}")
                
        elif cmd_type == sim_config.MCTL_IDLE:
            self.plant.set_motor_velocity(0.0)
            if self.state.verbose_level >= 1:
                print("[VirtualMCU] Motor stopped.")

    def start(self):
        """Starts 20Hz Motor Control Task Loop."""
        self.running = True
        self.start_time = time.time()
        self.mcu_thread = threading.Thread(target=self._mcu_loop, daemon=True)
        self.mcu_thread.start()

    def stop(self):
        self.running = False

    def _mcu_loop(self):
        dt = 1.0 / sim_config.MCU_LOOP_HZ  # 50 ms loop
        
        while self.running:
            loop_start = time.time()
            
            # 1. Evaluate Limit Switch Interrupts (Core 1 Failsafe)
            if self.state.flag_min_limit_hit:
                print("[VirtualMCU **CRITICAL**] Min limit switch hit! Recovering by moving 8189 steps (+1 direction)...")
                self.state.sys_state = sim_config.SYS_CRITICAL_ERROR
                self.state.sys_fault_code = sim_config.FAULT_MIN_LIMIT_HIT
                self.state.fault_counts[sim_config.FAULT_MIN_LIMIT_HIT] += 1
                
                # Move 8189 steps in +1 direction (direction 0 = positive displacement)
                self.execute_command(sim_config.MCTL_MOVING_PULSES, pulses=8189, direction=0)
                while self.pulses_remaining > 0 and self.running:
                    speed = sim_config.DEFAULT_VMAX_MM_S
                    self.plant.set_motor_velocity(speed)
                    steps_moved = int(abs(speed) * sim_config.STEPS_PER_MM * dt)
                    self.pulses_remaining = max(0, self.pulses_remaining - steps_moved)
                    time.sleep(dt)
                    
                self.plant.set_motor_velocity(0.0)
                self.mctl_state = sim_config.MCTL_IDLE
                self.state.flag_min_limit_hit = False
                self.state.flag_max_limit_hit = False
                self.state.sys_fault_code = sim_config.FAULT_NONE
                self.state.sys_state = sim_config.SYS_OPERATIONAL
                print("[VirtualMCU] Min limit recovery complete (moved 8189 steps).")
                continue

            if self.state.flag_max_limit_hit:
                print("[VirtualMCU **CRITICAL**] Max limit switch hit! Recovering by moving 8189 steps (-1 direction)...")
                self.state.sys_state = sim_config.SYS_CRITICAL_ERROR
                self.state.sys_fault_code = sim_config.FAULT_MAX_LIMIT_HIT
                self.state.fault_counts[sim_config.FAULT_MAX_LIMIT_HIT] += 1
                
                # Move 8189 steps in -1 direction (direction 1 = negative displacement)
                self.execute_command(sim_config.MCTL_MOVING_PULSES, pulses=8189, direction=1)
                while self.pulses_remaining > 0 and self.running:
                    speed = -sim_config.DEFAULT_VMAX_MM_S
                    self.plant.set_motor_velocity(speed)
                    steps_moved = int(abs(speed) * sim_config.STEPS_PER_MM * dt)
                    self.pulses_remaining = max(0, self.pulses_remaining - steps_moved)
                    time.sleep(dt)
                    
                self.plant.set_motor_velocity(0.0)
                self.mctl_state = sim_config.MCTL_IDLE
                self.state.flag_min_limit_hit = False
                self.state.flag_max_limit_hit = False
                self.state.sys_fault_code = sim_config.FAULT_NONE
                self.state.sys_state = sim_config.SYS_OPERATIONAL
                print("[VirtualMCU] Max limit recovery complete (moved 8189 steps).")
                continue

            # 2. Read Potentiometer & Update State
            current_raw_pot = self.state.potentiometer_raw
            h_medido = self.plant.pot_to_piston_pos(current_raw_pot)
            
            # 3. Process Control State Loop
            if self.mctl_state in (sim_config.MCTL_MOVING_UNTIL_POT, sim_config.MCTL_MOVING_ABSOLUTE):
                now_sec = time.time() - self.start_time
                href, vref, is_completed = self.vel_gen.update(now_sec)
                pos_error = href - h_medido
                
                if is_completed and abs(pos_error) < 0.2:
                    self.plant.set_motor_velocity(0.0)
                    self.mctl_state = sim_config.MCTL_IDLE
                    self.state.moves_completed += 1
                    if self.state.verbose_level >= 1:
                        print(f"[VirtualMCU] Target reached! h_medido={h_medido:.2f} mm, error={pos_error:.2f} mm")
                else:
                    v_control = self.pid.compute(href, h_medido, dt)
                    self.plant.set_motor_velocity(v_control)
                    
            elif self.mctl_state == sim_config.MCTL_MOVING_PULSES:
                if self.pulses_remaining <= 0:
                    self.plant.set_motor_velocity(0.0)
                    self.mctl_state = sim_config.MCTL_IDLE
                    self.state.moves_completed += 1
                else:
                    speed = sim_config.DEFAULT_VMAX_MM_S if self.pulse_direction == 0 else -sim_config.DEFAULT_VMAX_MM_S
                    self.plant.set_motor_velocity(speed)
                    # Decrement pulse count based on step frequency
                    steps_moved = int(abs(speed) * sim_config.STEPS_PER_MM * dt)
                    self.pulses_remaining = max(0, self.pulses_remaining - steps_moved)
            else:
                self.plant.set_motor_velocity(0.0)
                
            # Sleep remainder of 50ms (20Hz) period
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
