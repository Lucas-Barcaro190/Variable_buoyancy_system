"""
Variable Buoyancy System Sensor Characterization Script
Python version - communicates with Raspberry Pi Pico over serial
"""

import serial
import time
import datetime
import os
from typing import List, Tuple

class VBSSensorCharacterization:
    def __init__(self, port: str = "COM5", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.log_file = None

    def connect(self) -> bool:
        """Connect to the serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            #print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            #print(f"Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from the serial port"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            #print("Disconnected from serial port")

    def send_command(self, command: str) -> bool:
        """Send a command to the Pico"""
        if not self.serial or not self.serial.is_open:
            #print("Serial port not connected")
            return False

        try:
            self.serial.write(f"{command}\r\n".encode())
            self.serial.flush()
            return True
        except serial.SerialTimeoutException:
            print(f"Timeout sending command: {command}")
            return False
        except Exception as e:
            #print(f"Error sending command '{command}': {e}")
            return False

    def read_response(self) -> str:
        """Read a line from the serial port"""
        if not self.serial or not self.serial.is_open:
            return ""

        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                return line
        except Exception as e:
            return
            #print(f"Error reading response: {e}")
        return ""

    def create_log_file(self) -> str:
        """Create a timestamped log file"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        filename = f"pressure_log_{timestamp}.txt"
        log_path = os.path.join("logs", filename)

        # Ensure logs directory exists
        os.makedirs("logs", exist_ok=True)

        self.log_file = open(log_path, 'w', encoding='utf-8')
        self.log_file.write("PC_Timestamp,Pico_Timestamp,Pressure,Temperature,Encoder_Degrees,steps\n")
        self.log_file.flush()

        #print(f"Logging to: {log_path}")
        return log_path

    def log_data(self, pico_data: str):
        """Log data to file with PC timestamp - only log CSV data lines"""
        if self.log_file and pico_data:
            # Only log lines that look like CSV data (start with numbers)
            # CSV format: timestamp,pressure,temperature,encoder (all numeric)
            try:
                parts = pico_data.split(',')
                if len(parts) == 5:
                    # Check if all parts are numeric
                    float(parts[0])  # timestamp
                    float(parts[1])  # pressure
                    float(parts[2])  # temperature
                    float(parts[3])  # encoder
                    int(parts[4])    # steps
                    
                    pc_timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]  # milliseconds
                    log_line = f"{pc_timestamp},{pico_data}"
                    self.log_file.write(log_line + "\n")
                    self.log_file.flush()
                    #print(log_line)
            except (ValueError, IndexError):
                # Not a valid CSV data line, skip logging
                pass

    def vbs_command(self, repeat: int, command: str, speed: int = 0, value: int = 0, wait_time: int = 100) -> List[Tuple[str, int]]:
        """Create a list of commands to send"""
        commands = []
        #print(f"Creating {repeat} commands: {command} with value={value}, speed={speed}, wait_time={wait_time}ms")
        for _ in range(repeat):
            if command == "move":
                commands.append((f"{command} {value} {speed}", wait_time))
            elif command == "depth_sensor":
                commands.append((command, wait_time))
            elif command == " ":
                commands.append((command, wait_time))
            else:
                raise ValueError(f"Unknown command: {command}")
        return commands

    def create_step_commands(self, repeat: int, value: int) -> List[Tuple[str, int]]:
        """Create step commands for movement phases
            - Move at speed 2 for 86 pulses (1 step) then read depth sensor 4 times
            - Repeat this pattern for 2 times to create the 1 second step pattern.
        """
        commands = []
        for _ in range(repeat):
            commands.extend(self.vbs_command(1, "move", speed=2, value=value, wait_time=100))
            commands.extend(self.vbs_command(4, "depth_sensor", wait_time=100))
        return commands

    def run_sequence(self):
        """Run the main characterization sequence"""
        # Define the sequence phases
        sequence_phases = [
            {
                "message": "Starting reading period of 3 minutes in air...\n=======================================",
                "commands": self.vbs_command(1800, "depth_sensor", wait_time=100),
                "user_input": None
            },
            {
                "message": "Waiting for the user to put the VBS in water...",
                "commands": [],
                "user_input": "Type 'start' to begin the sequence: "
            },
            {
                "message": "30 seconds to put the VBS in water and stabilize...",
                "commands": self.vbs_command(1, " ", wait_time=30000),
                "user_input": None
            },
            {
                "message": "Starting reading period of 3 minutes in water surface...",
                "commands": self.vbs_command(1800, "depth_sensor", wait_time=100),
                "user_input": None
            },
            {
                "message": "Starting descend to full depth phase...",
                "commands": sum([self.create_step_commands(2 * 10, value=-86) for _ in range(40)], []),
                "user_input": None
            },
            {
                "message": "Starting wait period of 3 minutes at full depth...",
                "commands": self.vbs_command(1800, "depth_sensor", wait_time=100),
                "user_input": None
            },
            {
                "message": "Starting ascend to surface phase...",
                "commands": sum([self.create_step_commands(2 * 10, value=86) for _ in range(40)], []),
                "user_input": None
            }
        ]

        print("Starting sequence...")

        # Build the complete sequence
        sequence = []
        for phase in sequence_phases:
            if phase["user_input"]:
                user_input = input(phase["user_input"])
                while user_input.lower() != "start":
                    #print("Invalid input. Please type 'start' to proceed.")
                    user_input = input(phase["user_input"])
            sequence.extend(phase["commands"])

        # Main execution loop
        step = 0
        start_time = time.time()
        next_action_time = 0

        try:
            while True:
                current_time = time.time() - start_time

                # Read any available data from Pico
                response = self.read_response()
                if response:
                    self.log_data(response)

                # Send commands at appropriate times
                if step < len(sequence):
                    if current_time >= next_action_time:
                        cmd, delay = sequence[step]
                        if self.send_command(cmd):
                            print(f">>> SENT [{step}]: {cmd} (Next in: {delay/1000:.1f}s)")
                            next_action_time = current_time + (delay / 1000.0)
                            step += 1
                        else:
                            print("Failed to send command. Aborting sequence.")
                            break

                # Small delay to prevent CPU hogging
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nSequence interrupted by user")
        finally:
            if self.log_file:
                self.log_file.close()

def main():
    # Create characterization instance
    vbs = VBSSensorCharacterization()

    try:
        # Connect to serial port
        if not vbs.connect():
            return

        # Create log file
        log_path = vbs.create_log_file()

        # Run the characterization sequence
        vbs.run_sequence()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        vbs.disconnect()
        print(f"Connection closed. Output saved to: {log_path if 'log_path' in locals() else 'N/A'}")

if __name__ == "__main__":
    main()