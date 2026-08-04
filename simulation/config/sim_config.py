# simulation/config/sim_config.py
"""
Configuration constants for the VBS Digital Twin & HIL Simulator.
Mirrors hardware_config.h and source parameters.
"""

import math

# Mechanical and Movement Parameters
MINIMAL_THRESHOLD = 43       # Raw potentiometer 9-bit ADC min threshold
MAXIMUM_THRESHOLD = 435      # Raw potentiometer 9-bit ADC max threshold
POT_RANGE = float(MAXIMUM_THRESHOLD - MINIMAL_THRESHOLD)

MAX_PISTON_POSITION = 23.0   # Maximum physical extension/retraction in mm [-23.0, +23.0]
PISTON_RANGE = 2.0 * MAX_PISTON_POSITION  # 46.0 mm total stroke
MAX_VOLUME = 350.0           # Maximum cylinder displacement volume in cm^3

# Volumetric conversion: Area = pi * (D/2)^2 for D = 14.0 mm
# Area in mm^2 = pi * 7.0^2 = 153.93804 mm^2 = 0.15393804 cm^2 per mm of travel
CYLINDER_DIAMETER_MM = 14.0
VOL_MULTIPLIER_CM3_PER_MM = (math.pi * (CYLINDER_DIAMETER_MM / 2.0)**2) / 100.0

# Stepper Motor Parameters
MOTOR_STEPS_PER_REV = 400.0   # 400 steps per revolution (200 steps * 2 microsteps)
GEAR_REDUCTION = 61.417       # 61.417:1 gearbox reduction
SPINDLE_PITCH_MM = 3.0        # 3.0 mm lead screw / spindle pitch
STEPS_PER_MM = (MOTOR_STEPS_PER_REV * GEAR_REDUCTION) / SPINDLE_PITCH_MM  # ~8188.9333 steps/mm

FULL_STROKE_TIME_S = 196.0    # Full travel (-23mm to +23mm = 46mm stroke) takes 196 seconds
DEFAULT_VMAX_MM_S = PISTON_RANGE / FULL_STROKE_TIME_S  # ~0.23469387755 mm/s (0.2347 mm/s)
DEFAULT_ACCEL_TIME_S = 3.0
VELOCITY_DEADBAND_MM = 0.15

# System States
SYS_INIT = 0
SYS_OPERATIONAL = 1
SYS_FAILSAFE_ASCENT = 2
SYS_CRITICAL_ERROR = 3
SYS_CALIBRATION_MIN = 4
SYS_CALIBRATION_MAX = 5
SYS_MANUAL_CONTROL = 6

# Fault Codes
FAULT_NONE = 0x00
FAULT_MIN_LIMIT_HIT = 0xE1
FAULT_MAX_LIMIT_HIT = 0xE2
FAULT_PC_TIMEOUT = 0xE3
FAULT_MOTOR_STALL = 0xE6
FAULT_QUEUE_OVERFLOW = 0xE7

# Motor Control States
MCTL_IDLE = 0
MCTL_MOVING_ABSOLUTE = 1
MCTL_MOVING_UNTIL_POT = 2
MCTL_MOVING_PULSES = 3

# 3D Visualizer Settings
STL_MOUNT_FILENAME = "mount.stl"
STL_PISTON_FILENAME = "piston.stl"
WINDOW_SIZE = (1024, 768)
RENDER_FPS = 60
PHYSICS_HZ = 100
MCU_LOOP_HZ = 20
