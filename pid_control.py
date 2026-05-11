"""PID control for the Variable Buoyancy System.

Uses Pico serial commands:
- "depth_sensor" to request a sensor reading
- "expand" to move the actuator out by 0.05 mm
- "contract" to move the actuator in by 0.05 mm

Reads Pico CSV output:
    timestamp,pressure,temperature,encoder
"""

import argparse
import datetime
import os
import sys
import threading
import time

import serial
import numpy as np


class VBSController:
    def __init__(self, port: str, baudrate: int, kp: float, ki: float, kd: float, setpoint: float, log_path=None):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error_prev = 0.0
        self.integral = 0.0
        self.max_integral = 0.5
        self.pressure_offset = None
        self.depth = 0.0
        self.pressure = 0.0
        self.temperature = 0.0
        self.encoder_degrees = 0.0
        self.running = False
        self.log_file = None

        if log_path:
            os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
            self.log_file = open(log_path, "w", encoding="utf-8")
            self.log_file.write("PC_Timestamp,Pico_Timestamp,Pressure,Temperature,Encoder_Degrees,Depth_m\n")
            self.log_file.flush()

    def connect(self):
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1, write_timeout=1)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        if self.log_file:
            self.log_file.close()

    def send_command(self, command: str):
        if not self.serial or not self.serial.is_open:
            return False
        try:
            self.serial.write(f"{command}\r\n".encode())
            self.serial.flush()
            return True
        
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def read_line(self):
        if not self.serial or not self.serial.is_open:
            return ""
        try:
            return self.serial.readline().decode().strip()
        except:
            return ""

    def request_depth_sensor(self):
        if not self.send_command("depth_sensor"):
            return False
        line = self.read_line()
        if not line:
            return False
        parts = line.split(",")
        if len(parts) < 4:
            return False
        try:
            pico_timestamp = int(parts[0])
            self.pressure = float(parts[1])
            self.temperature = float(parts[2])
            self.encoder_degrees = float(parts[3])
            if self.pressure_offset is None:
                self.pressure_offset = self.pressure
                self.depth = 0.0
            else:
                self.depth = (self.pressure - self.pressure_offset) / 9806.0  # Simplified freshwater conversion

            if self.log_file:
                pc_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                self.log_file.write(f"{pc_ts},{pico_timestamp},{self.pressure:.3f},{self.temperature:.3f},{self.encoder_degrees:.3f},{self.depth:.6f}\n")
                self.log_file.flush()
            
            return True
        
        except ValueError:
            return False

    def pid_update(self, measurement: float, dt: float):
        error = self.setpoint - measurement
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        derivative = ((error - self.error_prev) / dt) if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.error_prev = error
        
        return output

    def control_loop(self, interval: float, deadband: float):
        self.running = True
        self.error_prev = 0.0
        self.integral = 0.0
        
        # Take initial sensor reading to set depth zero
        print("Taking initial sensor reading to set depth zero...")
        if not self.request_depth_sensor():
            print("Failed to get initial sensor reading")
            return
        
        print(f"Depth zero set at pressure {self.pressure:.3f} Pa")
        last_time = time.time()
        while self.running:
            loop_start = time.time()
            dt = loop_start - last_time
            last_time = loop_start
            if self.request_depth_sensor():
                pid_out = self.pid_update(self.depth, dt)
                cmd = None
                if pid_out < deadband:
                    cmd = "expand"
                elif pid_out > -deadband:
                    cmd = "contract"
                if cmd:
                    self.send_command(cmd)
                print(f"Depth={self.depth:.6f} m | Setpoint={self.setpoint:.6f} m | PID={pid_out:.6f} | Cmd={cmd or 'none'}")
            else:
                print("Warning: no sensor response")
            time.sleep(max(0, interval - (time.time() - loop_start)))

    def stop(self):
        self.running = False


def setpoint_input_loop(controller):
    while controller.running:
        try:
            inp = input("New setpoint (m) or 'q': ").strip()
            if inp.lower() == 'q':
                controller.stop()
                break
            controller.setpoint = float(inp)
            print(f"Setpoint: {controller.setpoint:.6f} m")
        except ValueError:
            print("Invalid input")
        except EOFError:
            break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--kp", type=float, default=0.001)
    parser.add_argument("--ki", type=float, default=0.0001)
    parser.add_argument("--kd", type=float, default=0.005)
    parser.add_argument("--setpoint", type=float, default=0.0)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--deadband", type=float, default=0.00005)
    parser.add_argument("--log", action="store_true")
    args = parser.parse_args()

    log_path = None
    if args.log:
        ts = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        log_path = os.path.join("logs", f"pid_control_{ts}.csv")

    controller = VBSController(args.port, args.baudrate, args.kp, args.ki, args.kd, args.setpoint, log_path)
    if not controller.connect():
        return 1

    try:
        threading.Thread(target=setpoint_input_loop, args=(controller,), daemon=True).start()
        controller.control_loop(args.interval, args.deadband)
    
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        controller.stop()
        controller.disconnect()
        if log_path:
            print(f"Log: {log_path}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())