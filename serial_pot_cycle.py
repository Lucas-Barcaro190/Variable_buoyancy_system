import argparse
import signal
import sys
import time
import serial


def main():
    parser = argparse.ArgumentParser(description="Cycle full_expand/full_contract with fixed 100s delays")
    parser.add_argument("--port", default="COM12", help="Serial port (default: COM12)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--cycles", type=int, default=100, help="Number of cycles to run (default: 100)")
    parser.add_argument("--kp", type=int, default=1616, help="Changing the Kp parameter")
    parser.add_argument("--kd", type=int, default=1616, help="Changing the Kd parameter")
    parser.add_argument("--changeKp", type=bool, default=False, help="Changes the Kp parameter at every cycle")
    parser.add_argument("--changeKd", type=bool, default=False, help="Changes the Kd parameter at every cycle")
    
    args = parser.parse_args()

    ser = None
    def handleResponse():
        start_time = time.time()
        while True:
                try:
                    line = ser.readline().decode().strip()
                    #if line:
                        #print(f"Received: {line}")

                    if "[Driver -> Pico]: 0xE0 0x02 0xE2 0x00" in line:
                        elapsed_time = time.time() - start_time
                        print(f"Got expected response, continuing... (took {elapsed_time:.2f} seconds)")
                        break
                    
                except Exception as e:
                    print(f"Read failed: {e}")
        
    def handle_sigint(signum, frame):
        print("\nInterrupted, closing serial and exiting...")
        try:
            if ser and ser.is_open:
                ser.close()

        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)

    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}")
        sys.exit(2)

    print(f"Opened {args.port} @ {args.baud}, cycling {args.cycles} times")
    
    Kd = 1616
    Kp = 1616
    
    try:
        for cycle in range (args.cycles):
            
            # change Kd block
            if(args.changeKd):
                message = f"..."
                if Kd > 400:
                    message = f"change_PID Kd {Kd - 100}"
                    Kd -= 100
                elif Kd > 70:
                    message = f"change_PID Kd {Kd - 10}"
                    Kd -= 10
                else:
                    message = f"change_PID Kd {Kd}"

                print(f"New Kd: {Kd}")
                ser.write((message + "\n").encode())
            
            # change Kp block
            if(args.changeKp and Kp <= 2000):
                message = f"change_PID Kp {Kp + 10}"
                ser.write((message + "\n").encode())

            # cycle per se
            print(f"Cycle {cycle}/{args.cycles}: sending move 1506572 32")

            try:
                ser.write(b"move 1506572 32\n")
            except Exception as e:
                print(f"Write failed: {e}")

            print("Waiting for response...")
            
            handleResponse()

            print(f"Cycle {cycle}/{args.cycles}: sending move -1506572 32")

            try:
                ser.write(b"move -1506572 32\n")
            except Exception as e:
                print(f"Write failed: {e}")

            handleResponse()

        print("Completed all cycles")

    finally:
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
