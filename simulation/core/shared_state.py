# simulation/core/shared_state.py
"""
Thread-safe shared state management for the VBS simulation framework.
"""

import threading
import time
from simulation.config import sim_config

class SharedSimulationState:
    """Thread-safe global state for plant physics, MCU emulator, and 3D visualizer."""
    
    def __init__(self):
        self._lock = threading.Lock()
        
        # Physical plant variables
        self.piston_pos_mm = 0.0          # Current physical piston position (-23.0 to +23.0 mm)
        self.target_piston_pos_mm = 0.0   # Setpoint piston position
        self.potentiometer_raw = 239      # Raw 9-bit ADC reading (0 to 511, neutral center ~239)
        self.target_pot_value = 239       # Target raw 9-bit ADC value
        self.current_volume_cm3 = 0.0     # Displaced volume in cm^3
        
        # Limit switch states (True when triggered/active)
        self.flag_min_limit_hit = False
        self.flag_max_limit_hit = False
        
        # System state and diagnostics
        self.sys_state = sim_config.SYS_INIT
        self.sys_fault_code = sim_config.FAULT_NONE
        self.verbose_level = 1
        
        # Statistics
        self.commands_queued = 0
        self.moves_completed = 0
        self.move_timeouts = 0
        self.fault_counts = {
            sim_config.FAULT_MIN_LIMIT_HIT: 0,
            sim_config.FAULT_MAX_LIMIT_HIT: 0,
            sim_config.FAULT_PC_TIMEOUT: 0,
            sim_config.FAULT_MOTOR_STALL: 0,
            sim_config.FAULT_QUEUE_OVERFLOW: 0,
        }
        self.last_heartbeat_time = time.time()
        self.address = 0x01
        
    def get_position(self):
        with self._lock:
            return self.piston_pos_mm

    def set_position(self, pos_mm):
        with self._lock:
            # Clamp to physical piston range [-23.0, +23.0]
            pos_mm = max(-sim_config.MAX_PISTON_POSITION, min(sim_config.MAX_PISTON_POSITION, pos_mm))
            self.piston_pos_mm = pos_mm
            
            # Update potentiometer raw reading (43 to 435)
            fraction = (pos_mm + sim_config.MAX_PISTON_POSITION) / sim_config.PISTON_RANGE
            pot = sim_config.MINIMAL_THRESHOLD + (fraction * sim_config.POT_RANGE)
            self.potentiometer_raw = int(round(pot))
            
            # Update volume in cm^3
            self.current_volume_cm3 = pos_mm * sim_config.VOL_MULTIPLIER_CM3_PER_MM
            
            # Evaluate end stops
            self.flag_min_limit_hit = (pos_mm <= -sim_config.MAX_PISTON_POSITION)
            self.flag_max_limit_hit = (pos_mm >= sim_config.MAX_PISTON_POSITION)

    def update_heartbeat(self):
        with self._lock:
            self.last_heartbeat_time = time.time()

    def state_to_string(self, state_val=None):
        if state_val is None:
            state_val = self.sys_state
        mapping = {
            sim_config.SYS_INIT: "SYS_INIT",
            sim_config.SYS_OPERATIONAL: "SYS_OPERATIONAL",
            sim_config.SYS_FAILSAFE_ASCENT: "SYS_FAILSAFE_ASCENT",
            sim_config.SYS_CRITICAL_ERROR: "SYS_CRITICAL_ERROR",
            sim_config.SYS_CALIBRATION_MIN: "SYS_CALIBRATION_MIN",
            sim_config.SYS_CALIBRATION_MAX: "SYS_CALIBRATION_MAX",
            sim_config.SYS_MANUAL_CONTROL: "SYS_MANUAL_CONTROL"
        }
        return mapping.get(state_val, f"UNKNOWN({state_val})")

    def fault_to_string(self, fault_val=None):
        if fault_val is None:
            fault_val = self.sys_fault_code
        mapping = {
            sim_config.FAULT_NONE: "FAULT_NONE",
            sim_config.FAULT_MIN_LIMIT_HIT: "FAULT_MIN_LIMIT_HIT",
            sim_config.FAULT_MAX_LIMIT_HIT: "FAULT_MAX_LIMIT_HIT",
            sim_config.FAULT_PC_TIMEOUT: "FAULT_PC_TIMEOUT",
            sim_config.FAULT_MOTOR_STALL: "FAULT_MOTOR_STALL",
            sim_config.FAULT_QUEUE_OVERFLOW: "FAULT_QUEUE_OVERFLOW"
        }
        return mapping.get(fault_val, f"UNKNOWN(0x{fault_val:02X})")
