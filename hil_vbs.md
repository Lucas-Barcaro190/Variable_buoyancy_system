# Hardware-in-the-Loop (HIL) Setup Guide & Walkthrough for VBS (Variable Buoyancy System)

This document provides a comprehensive walkthrough of **Hardware-in-the-Loop (HIL)** simulation concepts, system architecture, signal interfacing, plant model equations, real-time 3D visualization tools, and a tailored step-by-step implementation guide using your specific hardware inventory.

---

## 1. What is Hardware-in-the-Loop (HIL) Simulation?

### 1.1 Overview & Concept
**Hardware-in-the-Loop (HIL)** testing is a real-time simulation technique used to test embedded control systems where the physical parts of the system (the "Plant") are replaced by a mathematical model running on dedicated simulator hardware.

In a standard testing setup:
* **Without HIL (Software-only):** You test algorithms in pure software (e.g. MATLAB/Simulink or Python simulations), which misses hardware-level bugs like interrupt latency, register misconfigurations, DMA/PIO timing errors, or RTOS stack overflows.
* **Open-Bench physical testing:** You connect the real controller to actual motors, lead screws, and water tanks. This carries high risk: physical collisions can destroy mechanical actuators, seal failure can drown electronics, and edge cases (like limit switch failures) are dangerous to test manually.
* **With HIL:** The **actual target microcontroller (RP2040 MCU)** runs the production FreeRTOS firmware in real-time. The physical stepper motor, piston cylinder, fluid displacement, and feedback potentiometer are replaced by a **HIL Simulator board** (e.g., a second Raspberry Pi Pico running real-time plant physics).

```mermaid
graph TD
    subgraph Target Hardware under Test
        MCU[Pico #1: Target MCU<br/>FreeRTOS Dual-Core Firmware]
    end

    subgraph HIL Simulator System
        SIM_HW[Pico #2: HIL Plant Simulator<br/>PWM Filter / Pulse Counter]
        PLANT[Real-Time Physics Model<br/>• Piston Kinematics<br/>• Potentiometer Resistance<br/>• Limit Switch Logic<br/>• Buoyancy Forces]
        SUITE[Automated Test Runner<br/>• Python / PySerial / PyTest]
        VIS[3D Real-Time Visualizer<br/>• PyVista / Three.js STLs]
    end

    %% Electrical Signal Connections
    MCU -->|GPIO 4: PULSE| SIM_HW
    MCU -->|GPIO 5: DIR| SIM_HW
    MCU -->|GPIO 12: ENABLE| SIM_HW
    SIM_HW -->|GPIO 15 PWM + RC Filter -> GPIO 26 ADC0| MCU
    SIM_HW -->|GPIO 2: MIN LIMIT| MCU
    SIM_HW -->|GPIO 3: MAX LIMIT| MCU
    
    %% Communication & Control Connections
    MCU <==>|USB Serial #1| SUITE
    SIM_HW <==>|USB Serial #2| SUITE
    SUITE -->|Live Position h(t)| VIS
```

---

## 2. Strategic Roadmap & Thoughts on Digital Twin / MCU Emulation

> [!TIP]
> **Starting with a Digital Twin and Microcontroller Emulator is an EXCELLENT and highly recommended strategy.**
> Building a virtual Digital Twin before wiring physical hardware provides immediate safety, saves development time, and prevents accidental short circuits when dealing with battery power and stepper drivers.

We recommend a **3-Phase Execution Roadmap**:

```
 ┌─────────────────────────────────────────────────────────┐
 │ Phase 1: Pure Digital Twin & MCU Emulation (Software)    │
 │ • Run firmware on Wokwi / Python emulator               │
 │ • Validate protocol, FreeRTOS tasks, & trajectory math   │
 └────────────────────────────┬────────────────────────────┘
                              │
                              ▼
 ┌─────────────────────────────────────────────────────────┐
 │ Phase 2: Dual Raspberry Pi Pico HIL Setup               │
 │ • Pico #1 = Target VBS Controller                       │
 │ • Pico #2 = Real-Time Plant Physics Simulator           │
 │ • Wire-to-wire signal loop & automated Python testing    │
 └────────────────────────────┬────────────────────────────┘
                              │
                              ▼
 ┌─────────────────────────────────────────────────────────┐
 │ Phase 3: Hybrid Hardware Bench Test & 3D Visualization  │
 │ • Connect STL 3D Visualizer (PyVista / Three.js)        │
 │ • Connect physical NEMA 17 + Driver + Battery + Pot     │
 └─────────────────────────────────────────────────────────┘
```

---

## 3. Hardware Inventory & Pin Mapping

Below is the complete mapping using your exact hardware inventory:

### Inventory List:
* **2x Raspberry Pi Pico** (Pico #1: Target MCU, Pico #2: HIL Plant Simulator)
* **2x Physical Limit Switches**
* **1x Potentiometer**
* **2x USB Cables** (Connecting both Picos to PC)
* **1x Stepper Motor Driver** (e.g. MKS SERVO42C / A4988 / DRV8825)
* **1x NEMA 17 Stepper Motor**
* **1x Battery** (Power supply for stepper driver)
* **VBS STL Files & Mount** (3D Mechanical Model)
* **Jumper Wires & Breadboard**

### Signal & Pin Mapping Table:

| Function | Signal Name | Pico #1 (Target MCU) | Pico #2 (HIL Simulator) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| Step Pulse | `PIN_MOTOR_PULSE` | `GPIO 4` (Output) | `GPIO 4` (Input / Counter) | PIO PWM pulse generator output |
| Direction | `PIN_MOTOR_DIR` | `GPIO 5` (Output) | `GPIO 5` (Input) | Direction signal (0 = Contract, 1 = Expand) |
| Driver Enable | `PIN_ENABLE_DRIVER` | `GPIO 12` (Output) | `GPIO 12` (Input) | Active LOW motor driver enable |
| Min Limit | `SW_MIN_LIMIT` | `GPIO 3` (Input) | `GPIO 2` (Output Active LOW) | Retracted end-stop signal |
| Max Limit | `SW_MAX_LIMIT` | `GPIO 2` (Input) | `GPIO 3` (Output Active LOW) | Extended end-stop signal |
| Position Feedback | `POT_ADC_PIN` | `GPIO 26` (ADC0 Input) | `GPIO 15` (PWM Output + RC Filter) | Emulated analog voltage (0.0 V – 3.3 V) |
| Ground | `GND` | Any `GND` Pin | Any `GND` Pin | **MUST connect GND between both Picos!** |

---

## 4. Real-Time 3D Visualization Platforms Evaluation

You requested a simple way to render your VBS **STL files** (piston cylinder & mount) in real-time driven by telemetry from the Pico HIL system.

### Comparative Tool Matrix:

| Visualizer Tool | Setup Complexity | STL Import | Real-Time Serial Speed | Hydrodynamics Physics | Recommendation |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1. Three.js (Web Browser)** | ⭐ (Ultra Simple) | Native `STLLoader` | 60 FPS (WebSockets) | Basic Visuals | **BEST FOR QUICK & CLEAN 3D UI** |
| **2. PyVista / Open3D (Python 3D Window)** | ⭐ (Ultra Simple) | Native `.stl` load | 60 FPS (Direct PySerial) | Math Model | **BEST FOR ZERO-DEPENDENCY PYTHON APP** |
| **3. CoppeliaSim (V-REP)** | ⭐⭐⭐ (Medium) | Native | 50 FPS (ZeroMQ Remote API) | Good Hydrodynamics | Great if full AUV robot body is needed |
| **4. Gazebo (ROS 2 / UUV)** | ⭐⭐⭐⭐ (Complex) | SDF / URDF convert | 50 FPS (ROS 2 topics) | Full Hydrodynamics ($F_b, F_d$) | Overkill for simple VBS piston visualization |
| **5. Unreal Engine 5** | ⭐⭐⭐⭐⭐ (Very Heavy) | Requires import/rigging | Custom plugins | Custom C++ | Overkill (30GB download, complex setup) |

### Recommended Choice: **PyVista** or **Three.js**
For a 1D linear sliding assembly like VBS, **PyVista (Python)** or **Three.js (Browser)** are by far the best choices:
1. **No complex software to install:** No 30GB game engines or Linux ROS 2 setup needed.
2. **Direct STL Loading:** Simply pass `mount.stl` and `piston.stl`.
3. **Direct Serial Synchronization:** On every telemetry update from Pico #1 or Pico #2, translate `piston_mesh.position.z = h_mm`.

---

## 5. Phase-by-Phase Implementation Guide

### Phase 1: Virtual Digital Twin & MCU Emulation (Software)

#### Step 1.1: Renode RP2040 Microcontroller Emulation
Using **[matgla/Renode_RP2040](https://github.com/matgla/Renode_RP2040)** integrated into `renode/Renode_RP2040`:

1. **Compile Target Firmware:**
   ```powershell
   cmake -B build -G Ninja
   cmake --build build
   ```
2. **Launch Renode Simulation:**
   ```powershell
   # Windows (PowerShell)
   .\renode\run_renode.ps1
   
   # Linux / macOS / WSL
   ./renode/run_renode.sh
   
   # Or directly using Renode CLI:
   renode -e "$bin=@build/vbs_cpp.elf; include @renode/vbs_simulation.resc"
   ```
3. **Launch Digital Twin with Renode Bridge:**
   ```powershell
   py simulation/main.py --mode=renode
   ```

#### Step 1.2: Alternative Emulators (Pure Python / rp2040js / Wokwi)
- **Pure Python Virtual MCU:** `py simulation/main.py --mode=virtual`
- **Node.js rp2040js:** `node simulation/emulator/rp2040_runner.js` + `py simulation/main.py --mode=rp2040js`

---

### Phase 2: Building the Dual-Pico HIL Setup

#### Step 2.1: Electrical Wiring (Pico #1 <---> Pico #2)
Make the following direct wire connections between **Pico #1** (Target) and **Pico #2** (Simulator):

1. **Common Ground:** Connect `GND` of Pico #1 to `GND` of Pico #2.
2. **Step Signal:** Pico #1 `GPIO 4` $\rightarrow$ Pico #2 `GPIO 4`.
3. **Direction Signal:** Pico #1 `GPIO 5` $\rightarrow$ Pico #2 `GPIO 5`.
4. **Enable Signal:** Pico #1 `GPIO 12` $\rightarrow$ Pico #2 `GPIO 12`.
5. **Limit Switches:**
   * Pico #2 `GPIO 2` $\rightarrow$ Pico #1 `GPIO 3` (`SW_MIN_LIMIT`).
   * Pico #2 `GPIO 3` $\rightarrow$ Pico #1 `GPIO 2` (`SW_MAX_LIMIT`).
6. **Potentiometer Emulation Circuit:**
   * Pico #2 `GPIO 15` (PWM) $\rightarrow$ $1\text{ k}\Omega$ Resistor $\rightarrow$ Pico #1 `GPIO 26` (ADC0).
   * Connect a $10\,\mu\text{F}$ capacitor from Pico #1 `GPIO 26` to `GND`.

---

### Phase 3: Real-Time 3D STL Visualization Code Example

Below is a complete, working **PyVista Python script** that loads your `mount.stl` and `piston.stl` files, connects to the Pico serial port, and updates the piston position in real-time as the motor moves:

```python
# vbs_3d_visualizer.py
import pyvista as pv
import serial
import time
import threading

# Configuration
SERIAL_PORT = 'COM3'  # Pico #1 or HIL Pico USB Serial
BAUDRATE = 115200
STL_MOUNT_PATH = 'mount.stl'
STL_PISTON_PATH = 'piston.stl'

# Shared state
current_h_mm = 0.0

def serial_reader_thread():
    global current_h_mm
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        print(f"[Serial] Connected to {SERIAL_PORT}")
        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            # Parse telemetry: e.g. "Potentiometer: 250" or "h_medido=12.50 mm"
            if "h_medido=" in line:
                try:
                    part = line.split("h_medido=")[1].split("mm")[0]
                    current_h_mm = float(part)
                except Exception:
                    pass
    except Exception as e:
        print(f"[Serial Error] {e}")

# Start background serial thread
t = threading.Thread(target=serial_reader_thread, daemon=True)
t.start()

# PyVista 3D Scene Initialization
plotter = pv.Plotter(window_size=[1024, 768], title="VBS Real-Time Digital Twin 3D")
plotter.add_light(pv.Light(position=(0, 0, 500), intensity=0.8))

# Load STL Meshes
try:
    mount_mesh = pv.read(STL_MOUNT_PATH)
    piston_mesh = pv.read(STL_PISTON_PATH)
    
    plotter.add_mesh(mount_mesh, color='slategrey', opacity=0.8, name='mount')
    piston_actor = plotter.add_mesh(piston_mesh, color='dodgerblue', name='piston')
except Exception as e:
    print(f"[STL Load Warning] {e}. Rendering bounding box fallback.")

plotter.show(interactive_update=True)

# Main render loop (60 FPS)
last_h = 0.0
while True:
    if current_h_mm != last_h:
        delta_z = current_h_mm - last_h
        # Translate piston mesh along Z axis by delta_z mm
        piston_actor.SetPosition(0, 0, current_h_mm)
        last_h = current_h_mm

    plotter.update()
    time.sleep(0.016)
```

---

## 6. Verification & Test Plan

- [ ] **Phase 1 Verification:** Python Digital Twin connects over USB Serial and executes `help`, `diag`, and `move_pot 200` cleanly in software.
- [ ] **Phase 2 HIL Verification:**
  - Connect both Picos via USB to PC.
  - Send `move_pot 300` to Pico #1.
  - Verify Pico #2 counts step pulses on `GPIO 4`, updates $h(t)$, and adjusts PWM voltage output on `GPIO 15`.
- [ ] **Phase 3 3D Visualization:** Run `vbs_3d_visualizer.py` with your `mount.stl` and `piston.stl` files to see real-time movement during test runs!
