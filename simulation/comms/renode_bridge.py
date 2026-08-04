# simulation/comms/renode_bridge.py
"""
Renode Bridge Module — Connects the Python Digital Twin physical simulation loop
and 3D visualizer directly to a running Renode RP2040 emulator instance.

Features:
- Port 4321: UART0 serial CLI & telemetry stream
- Port 1234: Renode Monitor socket for hardware signal injection (ADC voltage & Limit Switches)
- Real-time 3D Piston rendering synchronized with physics engine
"""

import socket
import time
import threading
from simulation.comms.serial_bridge import SerialBridge
from simulation.config import sim_config


class RenodeBridge(SerialBridge):
    """
    Bridge connecting Python Digital Twin to Renode RP2040 simulation instance.
    """

    def __init__(self, protocol_parser, plant_model, host: str = "127.0.0.1", uart_port: int = 4321, monitor_port: int = 1234):
        super().__init__(protocol_parser, port=f"tcp://{host}:{uart_port}")
        self.renode_host = host
        self.renode_uart_port = uart_port
        self.renode_monitor_port = monitor_port
        self.plant = plant_model

        self._monitor_sock = None
        self._monitor_connected = False
        self._last_sent_voltage = None
        self._last_sw_min = None
        self._last_sw_max = None
        self._target_pot = None
        self._target_pos = None

    def start(self):
        print(f"[RenodeBridge] Connecting to Renode UART0 at {self.renode_host}:{self.renode_uart_port}...")
        super().start()
        self._connect_monitor()

    def send_input_line(self, line: str):
        """Dispatches command to Renode UART and updates plant physics model velocity."""
        cmd_str = line.strip()
        if not cmd_str:
            return

        parts = cmd_str.split()
        cmd = parts[0].lower()

        curr_pos = self.plant.state.get_position()

        # Update plant model velocity based on command
        if cmd == "move" and len(parts) >= 2:
            try:
                pulses = float(parts[1])
                speed_param = float(parts[2]) if len(parts) >= 3 else 30.0
                speed_mm_s = max(0.05, min(0.5, speed_param * 0.00733))
                velocity = speed_mm_s if pulses > 0 else -speed_mm_s
                
                # Convert pulse count to exact millimeter travel
                delta_mm = self.plant.steps_to_mm(pulses)
                self._target_pos = curr_pos + delta_mm
                self.plant.set_motor_velocity(velocity)
                self._target_pot = None
            except ValueError:
                pass
        elif cmd == "move_until_pot" and len(parts) >= 2:
            try:
                self._target_pot = int(parts[1])
                self._target_pos = None
                curr_pot = self.plant.piston_pos_to_pot(curr_pos)
                if self._target_pot > curr_pot:
                    self.plant.set_motor_velocity(sim_config.DEFAULT_VMAX_MM_S)
                elif self._target_pot < curr_pot:
                    self.plant.set_motor_velocity(-sim_config.DEFAULT_VMAX_MM_S)
            except ValueError:
                pass
        elif cmd == "full_contract":
            self._target_pot = 43
            self._target_pos = None
            self.plant.set_motor_velocity(-sim_config.DEFAULT_VMAX_MM_S)
        elif cmd == "full_expand":
            self._target_pot = 435
            self._target_pos = None
            self.plant.set_motor_velocity(sim_config.DEFAULT_VMAX_MM_S)
        elif cmd in ("stop", "disable"):
            self.plant.set_motor_velocity(0.0)
            self._target_pot = None
            self._target_pos = None

        super().send_input_line(line)

    def _connect_monitor(self):
        """Connects background socket to Renode Monitor for live signal injection."""
        def monitor_worker():
            for attempt in range(60):
                if self._stop.is_set():
                    return
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.connect((self.renode_host, self.renode_monitor_port))
                    sock.settimeout(0.5)
                    self._monitor_sock = sock
                    self._monitor_connected = True
                    print(f"[RenodeBridge] Hardware signal injection bridge active (Port {self.renode_monitor_port})")
                    self._hardware_injection_loop()
                    return
                except (ConnectionRefusedError, OSError):
                    time.sleep(0.5)

            print(f"[RenodeBridge Warning] Could not connect to Renode Monitor on port {self.renode_monitor_port}. Hardware signal injection paused.")

        t = threading.Thread(target=monitor_worker, daemon=True)
        t.start()

    def _send_monitor_cmd(self, cmd: str):
        if self._monitor_connected and self._monitor_sock:
            try:
                self._monitor_sock.sendall((cmd + "\r\n").encode("utf-8"))
            except Exception:
                self._monitor_connected = False

    def _hardware_injection_loop(self):
        """Continuously computes and injects Potentiometer ADC Voltage and Limit Switches into Renode."""
        while not self._stop.is_set() and self._monitor_connected:
            pos_mm = self.plant.state.get_position()
            curr_pot = self.plant.piston_pos_to_pot(pos_mm)

            # Target potentiometer check for move_until_pot
            if self._target_pot is not None:
                if abs(curr_pot - self._target_pot) <= 2:
                    self.plant.set_motor_velocity(0.0)
                    self._target_pot = None

            # Target position check for move <pulses>
            if self._target_pos is not None:
                if (self.plant.applied_velocity_mm_s > 0 and pos_mm >= self._target_pos) or \
                   (self.plant.applied_velocity_mm_s < 0 and pos_mm <= self._target_pos):
                    self.plant.set_motor_velocity(0.0)
                    self._target_pos = None

            # 1. Update Potentiometer ADC Voltage
            voltage = self.plant.piston_pos_to_adc_voltage(pos_mm)
            if self._last_sent_voltage is None or abs(voltage - self._last_sent_voltage) >= 0.005:
                self._send_monitor_cmd(f"sysbus.adc SetDefaultVoltageOnChannel 0 {voltage:.4f}")
                self._last_sent_voltage = voltage

            # 2. Update Limit Switches (Active Low: false = pressed/hit, true = released/normal)
            sw_min, sw_max = self.plant.get_limit_switches(pos_mm)
            if sw_min != self._last_sw_min:
                state_str = "false" if sw_min else "true"
                self._send_monitor_cmd(f"sysbus.gpioIn.SW_MIN_LIMIT State {state_str}")
                self._last_sw_min = sw_min

            if sw_max != self._last_sw_max:
                state_str = "false" if sw_max else "true"
                self._send_monitor_cmd(f"sysbus.gpioIn.SW_MAX_LIMIT State {state_str}")
                self._last_sw_max = sw_max

            time.sleep(0.02)  # 50 Hz signal injection loop
