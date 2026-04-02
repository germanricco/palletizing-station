import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from control.robot_EEZYbotMK2 import RoboticArmMK2
from control.workspace_generator import generate_workspace_boundary
from visualization.robot_simulator import WorkspaceVisualizer

if __name__ == "__main__":
    twin = RoboticArmMK2()
    boundary = generate_workspace_boundary(twin, resolution=60)
    vis = WorkspaceVisualizer()
    vis.plot_static_workspace(boundary)
    vis.show()