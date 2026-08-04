# simulation/main.py
"""
Main Entry Point for the Variable Buoyancy System (VBS) Digital Twin & HIL Simulator.
Orchestrates Plant Physics (100Hz), Virtual RP2040 MCU (20Hz), and PyVista 3D Visualizer (60Hz).

Modes:
  virtual   — 100% pure Python MCU emulator (default, no extra tools needed)
  rp2040js  — Node.js rp2040js ARM emulator running the compiled .elf firmware
  hil       — Physical RP2040 connected via USB-serial (COM port)
"""

import os
import sys
import time
import argparse
import threading

# Add workspace parent directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.init.sim_init import initialize_simulation
from simulation.core.shared_state import SharedSimulationState
from simulation.physics.vbs_plant import VBSPlantModel
from simulation.emulator.virtual_mcu import VirtualMCUEmulator
from simulation.comms.protocol_parser import ProtocolParser
from simulation.comms.serial_bridge import SerialBridge
from simulation.comms.renode_bridge import RenodeBridge
from simulation.viz.render_3d import VBS3DVisualizer


def physics_thread_loop(plant_model, shared_state, visualizer, stop_event):
    """Background worker loop executing plant kinematics integration at 100 Hz."""
    dt = 0.01  # 100 Hz
    while not stop_event.is_set():
        loop_start = time.time()
        plant_model.update_physics()
        if visualizer:
            visualizer.update_position(shared_state.get_position())
        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


import queue


def interactive_cli_loop(send_fn, stop_event):
    """
    Interactive command-line shell.
    send_fn(cmd) dispatches a command string to whichever MCU backend is active.
    """
    time.sleep(0.5)
    print("\n--- VBS Interactive Console Ready ---")
    print("Type 'help' for commands (e.g. 'move_pot 300', 'move 4000', 'diag', 'pot', 'stop', 'exit').")

    while not stop_event.is_set():
        try:
            line = input("> ")
            cmd_str = line.strip()
            if not cmd_str:
                continue

            if cmd_str.lower() in ("exit", "quit"):
                print("[Console] Exiting simulation...")
                stop_event.set()
                break

            send_fn(cmd_str)
            time.sleep(0.05)
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break
        except Exception as e:
            print(f"[Console Error] {e}")
            break


def main():
    parser = argparse.ArgumentParser(
        description="VBS Digital Twin & Hardware-in-the-Loop (HIL) Simulator Host"
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="virtual",
        choices=["virtual", "renode", "rp2040js", "hil"],
        help=(
            "Simulation mode:\n"
            "  virtual   — pure Python MCU emulator (default)\n"
            "  renode    — Renode RP2040 ARM instruction emulator (start with renode/vbs_simulation.resc)\n"
            "  rp2040js  — Node.js rp2040js ARM emulator (start separately with 'node simulation/emulator/rp2040_runner.js')\n"
            "  hil       — physical RP2040 via serial port (requires --port)"
        ),
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial COM port for HIL mode (e.g. COM3 or /dev/ttyACM0)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable PyVista 3D visualizer window",
    )
    args = parser.parse_args()

    initialize_simulation()

    # ── 1. Shared State & Physics Engine ──────────────────────────────────────
    shared_state = SharedSimulationState()
    plant_model = VBSPlantModel(shared_state)

    # ── 2. Virtual MCU & Protocol Parser ──────────────────────────────────────
    virtual_mcu = VirtualMCUEmulator(shared_state, plant_model)
    protocol_parser = ProtocolParser(virtual_mcu)

    # ── 3. Choose MCU backend ─────────────────────────────────────────────────
    serial_bridge = None

    if args.mode == "renode":
        # Renode emulator: connect to TCP server port 4321 (UART) and 1234 (Monitor)
        serial_bridge = RenodeBridge(protocol_parser, plant_model, uart_port=4321, monitor_port=1234)
        print("[System] Running in Renode RP2040 Instruction Emulator Mode.")
        print("[System] Real-time physical plant simulation & hardware signal injection active.")

    elif args.mode == "rp2040js":
        # rp2040js emulator: connect to Node.js server on TCP 8888
        serial_bridge = SerialBridge(protocol_parser, port="tcp://127.0.0.1:8888")
        print("[System] Running in rp2040js ARM Emulator Mode.")
        print("[System] Make sure the emulator is running:")
        print("         node simulation/emulator/rp2040_runner.js")

    elif args.mode == "hil":
        if not args.port:
            parser.error("--port is required for HIL mode (e.g. --port COM3)")
        serial_bridge = SerialBridge(protocol_parser, port=args.port)
        print(f"[System] Running in Physical HIL Mode on port {args.port}.")

    else:  # virtual
        virtual_mcu.start()
        print("[System] Running in 100% Pure Python Virtual MCU Mode.")

    # ── 4. Start 3D Visualizer Process ─────────────────────────────────────────
    visualizer = None
    if not args.no_viz:
        visualizer = VBS3DVisualizer(shared_state)
        visualizer.start_window()

    # ── 5. Start Background Threads ────────────────────────────────────────────
    stop_event = threading.Event()

    phys_thread = threading.Thread(
        target=physics_thread_loop,
        args=(plant_model, shared_state, visualizer, stop_event),
        daemon=True,
    )
    phys_thread.start()

    if serial_bridge:
        serial_bridge.start()

    # CLI dispatch function depends on mode
    if serial_bridge:
        send_cmd = serial_bridge.send_input_line
    else:
        send_cmd = protocol_parser.process_line

    cli_thread = threading.Thread(
        target=interactive_cli_loop,
        args=(send_cmd, stop_event),
        daemon=True,
    )
    cli_thread.start()

    # Main thread wait loop
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[System] Shutdown requested.")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    stop_event.set()
    virtual_mcu.stop()
    if serial_bridge:
        serial_bridge.stop()
    print("[System] Simulation terminated cleanly.")


if __name__ == "__main__":
    main()
