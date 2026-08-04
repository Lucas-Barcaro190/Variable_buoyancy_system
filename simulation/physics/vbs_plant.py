# simulation/physics/vbs_plant.py
"""
Real-time plant dynamics simulation for the Variable Buoyancy System.
Calculates piston motion, volume displacement, potentiometer output, and limit switch states.
"""

import time
import math
from simulation.config import sim_config

class VBSPlantModel:
    """Real-time physical model of the VBS actuator and cylinder."""
    
    def __init__(self, shared_state):
        self.state = shared_state
        self.last_update_time = time.time()
        self.applied_velocity_mm_s = 0.0
        self.is_running = False

    def steps_to_mm(self, steps):
        """
        Converts motor step pulse count to linear piston displacement in mm.
        400 steps/rev * 61.417 reduction / 3.0 mm spindle pitch = 8188.9333 steps/mm.
        """
        return float(steps) / sim_config.STEPS_PER_MM

    def mm_to_steps(self, pos_mm):
        """Converts linear piston displacement in mm to motor step count."""
        return float(pos_mm) * sim_config.STEPS_PER_MM

    def pot_to_piston_pos(self, pot_value):
        """Converts raw 9-bit ADC potentiometer value to position in mm."""
        pot_clamped = max(sim_config.MINIMAL_THRESHOLD, min(sim_config.MAXIMUM_THRESHOLD, pot_value))
        fraction = (pot_clamped - sim_config.MINIMAL_THRESHOLD) / sim_config.POT_RANGE
        return (fraction * sim_config.PISTON_RANGE) - sim_config.MAX_PISTON_POSITION

    def piston_pos_to_pot(self, pos_mm):
        """Converts position in mm to raw 9-bit ADC potentiometer value."""
        pos_clamped = max(-sim_config.MAX_PISTON_POSITION, min(sim_config.MAX_PISTON_POSITION, pos_mm))
        fraction = (pos_clamped + sim_config.MAX_PISTON_POSITION) / sim_config.PISTON_RANGE
        return int(round(sim_config.MINIMAL_THRESHOLD + (fraction * sim_config.POT_RANGE)))

    def piston_pos_to_volume(self, pos_mm):
        """Calculates displaced fluid volume in cm^3."""
        vol = pos_mm * sim_config.VOL_MULTIPLIER_CM3_PER_MM
        return max(-sim_config.MAX_VOLUME, min(sim_config.MAX_VOLUME, vol))

    def volume_to_piston_pos(self, volume_cm3):
        """Calculates piston position in mm from volume in cm^3."""
        pos = volume_cm3 / sim_config.VOL_MULTIPLIER_CM3_PER_MM
        return max(-sim_config.MAX_PISTON_POSITION, min(sim_config.MAX_PISTON_POSITION, pos))

    def piston_pos_to_adc_voltage(self, pos_mm):
        """Converts position in mm to ADC channel voltage (0.0V - 3.3V)."""
        pot_9bit = self.piston_pos_to_pot(pos_mm)
        raw_12bit = pot_9bit * 8
        voltage = (raw_12bit / 4095.0) * 3.3
        return max(0.0, min(3.3, voltage))

    def get_limit_switches(self, pos_mm):
        """
        Returns limit switch trigger states (sw_min_hit, sw_max_hit).
        Limit switches hit when piston passes -23.5mm or +23.5mm.
        """
        sw_min = (pos_mm <= -23.5)
        sw_max = (pos_mm >= 23.5)
        return sw_min, sw_max

    def set_motor_velocity(self, speed_mm_s):
        """Applies motor drive velocity in mm/s."""
        self.applied_velocity_mm_s = speed_mm_s

    def update_physics(self):
        """Performs one integration step of the physical plant (100 Hz loop)."""
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        if dt <= 0.0 or dt > 0.5:
            return  # Skip invalid or paused integration step
            
        current_pos = self.state.get_position()
        
        # Integrate position change: h(t) = h(t-1) + v * dt
        new_pos = current_pos + (self.applied_velocity_mm_s * dt)
        
        # Physical Travel Limits (-24.0mm to +24.0mm max)
        max_travel = sim_config.MAX_PISTON_POSITION + 1.0  # Allow reaching 23.5mm for limit switch
        if new_pos <= -max_travel:
            new_pos = -max_travel
            if self.applied_velocity_mm_s < 0:
                self.applied_velocity_mm_s = 0.0
                
        if new_pos >= max_travel:
            new_pos = max_travel
            if self.applied_velocity_mm_s > 0:
                self.applied_velocity_mm_s = 0.0

        # Push updated state
        self.state.set_position(new_pos)
        self.state.potentiometer_value = self.piston_pos_to_pot(new_pos)
        self.state.volume_cm3 = self.piston_pos_to_volume(new_pos)
