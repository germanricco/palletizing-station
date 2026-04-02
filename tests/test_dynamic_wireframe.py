import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from control.robot_EEZYbotMK2 import RoboticArmMK2
from visualization.robot_simulator import WireframeVisualizer3D

if __name__ == "__main__":
    twin = RoboticArmMK2()
    vis = WireframeVisualizer3D()
    t = 0.0
    try:
        while plt.fignum_exists(vis.fig.number):
            q1 = np.sin(t) * 0.5
            q2 = np.deg2rad(80) + np.sin(t * 1.5) * 0.3
            q3_min, q3_max = twin._calc_q3_limits(q2)
            q3 = ((q3_max + q3_min) / 2.0) + np.cos(t * 2) * 0.2
            
            twin.update_state(q1, q2, q3)
            vis.draw_robot(twin.get_cartesian_state())
            t += 0.05
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.close('all')