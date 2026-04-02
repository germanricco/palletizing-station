import sys
import os
import time
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from control.robot_EEZYbotMK2 import RoboticArmMK2
from control.workspace_generator import generate_lateral_boundary_path
from visualization.robot_simulator import LateralWireframeVisualizer

if __name__ == "__main__":
    twin = RoboticArmMK2()
    q_traj, tcp_bound = generate_lateral_boundary_path(twin, resolution=50)
    vis = LateralWireframeVisualizer(tcp_bound)
    
    try:
        while plt.fignum_exists(vis.fig.number):
            for q_cmd in q_traj:
                if not plt.fignum_exists(vis.fig.number):
                    break
                twin.update_state(*q_cmd)
                vis.draw_robot_lateral(twin.get_cartesian_state())
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.close('all')