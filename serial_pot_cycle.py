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
    args = parser.parse_args()

    ser = None

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

    cycle = 0
    try:
        while cycle < args.cycles:
            cycle += 1
            print(f"Cycle {cycle}/{args.cycles}: sending full_expand")
            try:
                ser.write(b"full_expand\n")
            except Exception as e:
                print(f"Write failed: {e}")
            print("Waiting 200 seconds...")
            time.sleep(200)

            print(f"Cycle {cycle}/{args.cycles}: sending full_contract")
            try:
                ser.write(b"full_contract\n")
            except Exception as e:
                print(f"Write failed: {e}")
            print("Waiting 100 seconds...")
            time.sleep(100)

        print("Completed all cycles")

    finally:
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
