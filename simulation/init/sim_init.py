# simulation/init/sim_init.py
"""
Initialization routines for the VBS simulation environment.
"""

import sys
import importlib

def check_dependencies():
    """Verifies that required Python packages are installed."""
    required = ["serial", "numpy"]
    missing = []
    
    for pkg in required:
        try:
            importlib.import_module(pkg)
        except ImportError:
            missing.append(pkg)
            
    try:
        importlib.import_module("pyvista")
    except ImportError:
        print("[Init Notice] PyVista is not installed. 3D window visualization will be disabled unless installed ('pip install pyvista').")

    if missing:
        print(f"[Init Error] Missing required packages: {missing}")
        print(f"Please install them using: pip install -r simulation/requirements.txt")
        return False
        
    print("[Init] Dependency check complete. All core packages available.")
    return True

def initialize_simulation():
    """Initializes the simulation environment."""
    print("\n=== VBS Digital Twin & HIL Simulator Initialization ===")
    if not check_dependencies():
        sys.exit(1)
    print("=== Environment Initialization Complete ===\n")
