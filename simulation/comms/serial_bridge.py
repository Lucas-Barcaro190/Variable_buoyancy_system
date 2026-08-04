# simulation/comms/serial_bridge.py
"""
Serial/TCP Bridge — connects the Python simulation console to an external
MCU emulator (rp2040js) or physical hardware listening on a serial or TCP port.

Supported port formats:
  "tcp://127.0.0.1:8888"  — connect to rp2040js Node.js emulator
  "COM3" / "/dev/ttyACM0" — connect to physical RP2040 over USB-serial
"""

import threading
import time

try:
    import serial
    _PYSERIAL_OK = True
except ImportError:
    _PYSERIAL_OK = False

try:
    import socket as _socket
    _SOCKET_OK = True
except ImportError:
    _SOCKET_OK = False


class SerialBridge:
    """
    Reads lines from a serial/TCP source, passes them to `protocol_parser`,
    and exposes `send_input_line` to write commands back to the MCU.
    """

    def __init__(self, protocol_parser, port: str):
        self.protocol_parser = protocol_parser
        self.port = port
        self._stop = threading.Event()
        self._conn = None          # serial.Serial or socket
        self._is_tcp = port.startswith("tcp://")
        self._thread = None

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self):
        if self._is_tcp:
            self._connect_tcp()
        else:
            self._connect_serial()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        try:
            if self._conn:
                self._conn.close()
        except Exception:
            pass

    def send_input_line(self, line: str):
        """Send a command string (with CRLF newline) to the MCU."""
        cmd_str = line.strip()
        if not cmd_str:
            return
        data = (cmd_str + "\r\n").encode("utf-8")
        try:
            if self._is_tcp:
                self._conn.sendall(data)
            else:
                self._conn.write(data)
        except Exception as e:
            print(f"[SerialBridge] Send error: {e}")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _connect_tcp(self):
        """Connect to tcp://host:port (rp2040js / Renode emulator)."""
        url = self.port[len("tcp://"):]
        host, port_str = url.rsplit(":", 1)
        port = int(port_str)

        # Wait for the emulator to start
        for attempt in range(120):
            try:
                sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
                sock.connect((host, port))
                sock.settimeout(0.1)
                self._conn = sock
                print(f"[SerialBridge] Connected to {self.port}")
                return
            except (ConnectionRefusedError, OSError):
                if attempt == 0:
                    print(f"[SerialBridge] Waiting for emulator on {self.port} ...")
                time.sleep(0.5)

        raise RuntimeError(f"[SerialBridge] Could not connect to {self.port} after 60 s. Ensure Renode is running.")

    def _connect_serial(self):
        if not _PYSERIAL_OK:
            raise ImportError("pyserial is not installed. Run: pip install pyserial")
        self._conn = serial.Serial(self.port, baudrate=115200, timeout=0.1)
        print(f"[SerialBridge] Opened serial port {self.port}")

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                if self._is_tcp:
                    chunk = self._conn.recv(256)
                    if not chunk:
                        break
                else:
                    chunk = self._conn.read(256)
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        print(f"[MCU] {text}")
            except (_socket.timeout, Exception):
                pass
