# simulation/comms/protocol_parser.py
"""
Protocol parser and CRC8 checksum calculator for ASCII CLI and binary packet protocol.
Matches src/comms/protocol.cpp and src/tasks/task_handlers.cpp.
"""

import struct
from simulation.config import sim_config

# CRC8 Table matching protocol.cpp (Bluetooth CRC8 generator polynomial x^8 + x^7 + x^4 + x^3 + x + 1)
CRC8_TABLE = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xC2, 0xC5, 0xCC, 0xCB, 0xFE, 0xF9, 0xF0, 0xF7
]

def calculate_crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc

class ProtocolParser:
    """Parses incoming serial ASCII text commands or binary packets."""
    
    def __init__(self, virtual_mcu):
        self.mcu = virtual_mcu

    def process_line(self, line: str) -> str:
        """Processes an ASCII command line, prints any response, and returns it."""
        response = self.parse_line(line)
        if response:
            print(response)
        return response

    def parse_line(self, line: str) -> str:
        """Processes an ASCII CLI command line and returns text response."""
        line = line.strip()
        if not line:
            return ""
            
        self.mcu.state.update_heartbeat()
        parts = line.split()
        cmd = parts[0].lower()
        args = parts[1:]
        
        if cmd == "help":
            return (
                "Available commands:\n"
                "  help         - Show this help\n"
                "  move N       - Move N steps (signed integer, e.g. move 4094 or move -4094)\n"
                "  move_pot P   - Move to potentiometer value P (0-511, e.g. move_pot 250)\n"
                "  diag         - Print diagnostics now\n"
                "  pot          - Print current potentiometer value\n"
                "  stop         - Send driver stop command\n"
                "  verbose V    - Set verbose level (0-6)"
            )
            
        elif cmd == "pot":
            return f"Potentiometer (raw ADC avg): {self.mcu.state.potentiometer_raw}"
            
        elif cmd == "diag":
            s = self.mcu.state
            return (
                "\n--- DIAGNOSTICS (on-demand) ---\n"
                f"State: {s.state_to_string()}\n"
                f"Fault: {s.fault_to_string()}\n"
                f"Potentiometer: {s.potentiometer_raw} [{sim_config.MINIMAL_THRESHOLD} - {sim_config.MAXIMUM_THRESHOLD}]\n"
                f"Position: {s.get_position():.2f} mm | Volume: {s.current_volume_cm3:.2f} cm^3\n"
                f"Commands queued: {s.commands_queued}\n"
                f"Moves completed: {s.moves_completed}\n"
                f"Move timeouts: {s.move_timeouts}\n"
                "--- end diagnostics ---\n"
            )
            
        elif cmd == "stop":
            self.mcu.execute_command(sim_config.MCTL_IDLE)
            return "Queued stop command"
            
        elif cmd == "move_pot":
            if not args:
                return "Usage: move_pot <pot_value>"
            try:
                target_pot = int(args[0])
                target_pot = max(0, min(511, target_pot))
                self.mcu.execute_command(sim_config.MCTL_MOVING_UNTIL_POT, target_pot=target_pot)
                return f"Queued move_pot target={target_pot}"
            except ValueError:
                return "Error: Invalid potentiometer integer value"
                
        elif cmd == "move":
            if not args:
                return "Usage: move <steps>"
            try:
                steps = int(args[0])
                direction = 1 if steps < 0 else 0
                pulses = abs(steps)
                self.mcu.execute_command(sim_config.MCTL_MOVING_PULSES, pulses=pulses, direction=direction)
                return f"Queued move command: pulses={pulses}, direction={direction}"
            except ValueError:
                return "Error: Invalid steps integer value"
                
        elif cmd == "verbose":
            if args:
                try:
                    v = int(args[0])
                    self.mcu.state.verbose_level = max(0, min(6, v))
                    return f"Verbose level set to {self.mcu.state.verbose_level}"
                except ValueError:
                    pass
            return f"Current verbose level: {self.mcu.state.verbose_level}"
            
        else:
            return f"Unknown command: '{cmd}'"
