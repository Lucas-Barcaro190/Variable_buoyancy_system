# simulation/viz/render_3d.py
"""
PyVista 3D Real-Time Visualizer for the Variable Buoyancy System.
Renders mount.stl and piston.stl meshes with fallback geometry if STL files are not present.

Uses multiprocessing process isolation to keep PyVista VTK GUI render window
completely decoupled from the main process console stdin and socket bridges.
"""

import os
import time
import multiprocessing
from simulation.config import sim_config

try:
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False


def _create_fallback_geometry():
    """Creates parametric 3D fallback CAD geometry if STLs are missing."""
    mount_mesh = pv.Cylinder(
        center=(0, 0, 0),
        direction=(0, 0, 1),
        radius=140.0 / 2.0,
        height=300.0,
        capping=False
    )
    piston_mesh = pv.Cylinder(
        center=(0, 0, 112),
        direction=(0, 0, 1),
        radius=140.0 / 2.0,
        height=30.0,
        capping=True
    )
    return mount_mesh, piston_mesh


def _load_stl_meshes():
    """Attempts to load STL files or uses fallback geometry."""
    mount_path = sim_config.STL_MOUNT_FILENAME
    piston_path = sim_config.STL_PISTON_FILENAME

    if not os.path.exists(mount_path):
        mount_path = os.path.join(os.path.dirname(__file__), "..", "..", sim_config.STL_MOUNT_FILENAME)
    if not os.path.exists(piston_path):
        piston_path = os.path.join(os.path.dirname(__file__), "..", "..", sim_config.STL_PISTON_FILENAME)

    if os.path.exists(mount_path) and os.path.exists(piston_path):
        try:
            print(f"[Viz] Loading STL files: '{mount_path}' and '{piston_path}'")
            mount_mesh = pv.read(mount_path)
            piston_mesh = pv.read(piston_path)
            return mount_mesh, piston_mesh
        except Exception as e:
            print(f"[Viz Warning] Error reading STL files: {e}")

    print("[Viz] Creating parametric 3D geometry for VBS Mount & Piston...")
    return _create_fallback_geometry()


def _viz_process_worker(pos_val, stop_flag):
    """Child process worker function for PyVista rendering."""
    if not PYVISTA_AVAILABLE:
        print("[Viz Error] PyVista library is not installed.")
        return

    plotter = pv.Plotter(window_size=list(sim_config.WINDOW_SIZE), title="VBS Real-Time Digital Twin 3D")
    plotter.add_light(pv.Light(position=(0, 100, 200), intensity=0.8))
    plotter.add_axes()
    plotter.set_background("#1E1E2E")

    mount_mesh, piston_mesh = _load_stl_meshes()

    plotter.add_mesh(mount_mesh, color="#A6ADC8", opacity=0.6, name="mount_mesh", show_edges=True)
    piston_actor = plotter.add_mesh(piston_mesh, color="#89B4FA", name="piston_mesh", show_edges=True)

    print("[Viz] 3D Visualizer active. Rendering at 60 FPS.")
    plotter.show(interactive_update=True)

    last_pos = None
    dt = 1.0 / sim_config.RENDER_FPS

    while not stop_flag.value:
        loop_start = time.time()
        current_pos = pos_val.value

        if current_pos != last_pos:
            piston_actor.SetPosition(0, 0, current_pos)
            last_pos = current_pos

        plotter.update()

        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    try:
        plotter.close()
    except Exception:
        pass


class VBS3DVisualizer:
    """Real-time PyVista 3D visualizer process manager."""

    def __init__(self, shared_state):
        self.state = shared_state
        self.pos_val = multiprocessing.Value('d', 0.0)
        self.stop_flag = multiprocessing.Value('b', False)
        self.process = None

    def start_window(self):
        """Initializes and runs the PyVista 3D render loop in a dedicated process."""
        if not PYVISTA_AVAILABLE:
            print("[Viz Error] PyVista library is not installed. Install with 'pip install pyvista pyvistaqt'.")
            return

        self.process = multiprocessing.Process(
            target=_viz_process_worker,
            args=(self.pos_val, self.stop_flag),
            daemon=True
        )
        self.process.start()

    def update_position(self, pos_mm):
        """Pushes new position value to child visualizer process."""
        self.pos_val.value = pos_mm

    def stop(self):
        """Stops child visualizer process."""
        self.stop_flag.value = True
        if self.process and self.process.is_alive():
            self.process.join(timeout=1.0)
            if self.process.is_alive():
                self.process.terminate()
