# ==============================================================================
# Archivo: tests/test_trajectory_generator.py
# ==============================================================================
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Resolución de rutas para importar los módulos del proyecto
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from control.robot_EEZYbotMK2 import RoboticArmMK2
from control.trajectory_generator import TrajectoryGenerator

def plot_joint_space(time_array: np.ndarray, q_matrix: np.ndarray, title: str = "Espacio Articular (Cinemático)"):
    """a
    Genera 3 subplots apilados para visualizar la evolución temporal 
    de cada articulación en grados.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title("Análisis de Espacio Articular")
    
    # Conversión a grados para facilitar la lectura humana
    q_deg = np.rad2deg(q_matrix)
    labels = ['q1 (Base)', 'q2 (Hombro)', 'q3 (Codo)']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

    for i in range(3):
        axs[i].plot(time_array, q_deg[:, i], color=colors[i], linewidth=2.5)
        axs[i].set_ylabel(f"{labels[i]} [°]", fontsize=10, fontweight='bold')
        axs[i].grid(True, linestyle='--', alpha=0.7)

    axs[-1].set_xlabel("Tiempo [s]", fontsize=12)
    plt.suptitle(title, fontsize=14, fontweight='bold')
    fig.tight_layout()

def plot_cartesian_space(xyz_matrix: np.ndarray):
    """
    Renderiza la trayectoria en un espacio 3D para verificar 
    la linealidad del movimiento del TCP.
    """
    fig = plt.figure(figsize=(8, 8))
    fig.canvas.manager.set_window_title("Análisis de Espacio Cartesiano")
    ax = fig.add_subplot(111, projection='3d')

    X = xyz_matrix[:, 0]
    Y = xyz_matrix[:, 1]
    Z = xyz_matrix[:, 2]

    # Plot de la trayectoria principal
    ax.plot(X, Y, Z, color='purple', linewidth=3, label='Trayectoria TCP')
    
    # Marcadores de inicio y fin
    ax.scatter(X[0], Y[0], Z[0], color='green', s=100, marker='o', label='Inicio (P0)')
    ax.scatter(X[-1], Y[-1], Z[-1], color='red', s=100, marker='X', label='Fin (P1)')

    ax.set_xlabel('Eje X [mm]')
    ax.set_ylabel('Eje Y [mm]')
    ax.set_zlabel('Eje Z [mm]')
    ax.set_title('Trayectoria Cartesiana 3D del Tool Center Point', fontsize=12, fontweight='bold')
    
    # Ajuste de proporciones para evitar distorsión visual
    ax.set_box_aspect([1, 1, 1]) 
    ax.legend()


def plot_comparative_kinematics(dt, s_trap, s_quintic, s_septic, title):
    """
    Calcula y grafica Posición, Velocidad y Aceleración para comparar 
    los TRES perfiles de movimiento.
    """
    # Vectores de tiempo
    t_trap = np.arange(len(s_trap)) * dt
    t_quintic = np.arange(len(s_quintic)) * dt
    t_septic = np.arange(len(s_septic)) * dt

    # Derivadas Numéricas (edge_order=2 mejora la precisión en los extremos t=0 y t=T)
    v_trap = np.gradient(s_trap, dt, edge_order=2)
    a_trap = np.gradient(v_trap, dt, edge_order=2)

    v_quintic = np.gradient(s_quintic, dt, edge_order=2)
    a_quintic = np.gradient(v_quintic, dt, edge_order=2)

    v_septic = np.gradient(s_septic, dt, edge_order=2)
    a_septic = np.gradient(v_septic, dt, edge_order=2)

    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=False)
    fig.canvas.manager.set_window_title(title)
    
    # --- Plot Posición ---
    axs[0].plot(t_trap, s_trap, label='Trapezoidal (1er Orden)', color='#1f77b4', linewidth=2)
    axs[0].plot(t_quintic, s_quintic, label='Quintic (5to Orden)', color='#ff7f0e', linestyle='--', linewidth=2.5)
    axs[0].plot(t_septic, s_septic, label='Septic (7mo Orden)', color='#2ca02c', linestyle='-.', linewidth=2.5)
    axs[0].set_ylabel("Distancia [mm]", fontweight='bold')
    axs[0].set_title(title, fontweight='bold', fontsize=13)
    axs[0].legend(loc='lower right')
    axs[0].grid(True, linestyle='--', alpha=0.6)

    # --- Plot Velocidad ---
    axs[1].plot(t_trap, v_trap, color='#1f77b4', linewidth=2)
    axs[1].plot(t_quintic, v_quintic, color='#ff7f0e', linestyle='--', linewidth=2.5)
    axs[1].plot(t_septic, v_septic, color='#2ca02c', linestyle='-.', linewidth=2.5)
    axs[1].set_ylabel("Velocidad [mm/s]", fontweight='bold')
    axs[1].grid(True, linestyle='--', alpha=0.6)

    # --- Plot Aceleración ---
    axs[2].plot(t_trap, a_trap, color='#1f77b4', linewidth=2)
    axs[2].plot(t_quintic, a_quintic, color='#ff7f0e', linestyle='--', linewidth=2.5)
    axs[2].plot(t_septic, a_septic, color='#2ca02c', linestyle='-.', linewidth=2.5)
    axs[2].set_ylabel("Aceleración [mm/s²]", fontweight='bold')
    axs[2].set_xlabel("Tiempo [s]", fontweight='bold', fontsize=11)
    axs[2].grid(True, linestyle='--', alpha=0.6)

    fig.tight_layout()

if __name__ == "__main__":
    dt = 0.02
    robot = RoboticArmMK2()
    planner = TrajectoryGenerator(robot, dt_seconds=dt)

    P0 = np.array([0.0, 122.0, 230.0])
    P1 = np.array([0.0, 122.0, 150.0])
    
    # Distancia a recorrer = 80mm
    dist = np.linalg.norm(P1 - P0)

    print(f"Distancia cartesiana a recorrer: {dist:.2f} mm\n")

    # ==========================================================
    # EXPERIMENTO 1: MODO ASÍNCRONO (Límites Fijos)
    # ==========================================================
    V_MAX = 50.0  # mm/s
    A_MAX = 150.0 # mm/s^2
    print(f"--- MODO ASÍNCRONO (Vmax={V_MAX}, Amax={A_MAX}) ---")

    # 1A. Trapezoidal Asíncrono
    xyz_trap_async, _, _ = planner.generate_linear_trajectory(
        P0, P1, profile='trapezoidal', v_max=V_MAX, a_max=A_MAX)
    s_trap_async = np.linalg.norm(xyz_trap_async - P0, axis=1)

    # 1B. Quintic Asíncrono
    xyz_quintic_async, _, _ = planner.generate_linear_trajectory(
        P0, P1, profile='quintic', v_max=V_MAX, a_max=A_MAX)
    s_quintic_async = np.linalg.norm(xyz_quintic_async - P0, axis=1)

    # 1C. Septic Asíncrono
    xyz_septic_async, _, _ = planner.generate_linear_trajectory(
        P0, P1, profile='septic', v_max=V_MAX, a_max=A_MAX)
    s_septic_async = np.linalg.norm(xyz_septic_async - P0, axis=1)

    print(f"Tiempo total Trapezoidal: {(len(s_trap_async)-1)*dt:.2f}s")
    print(f"Tiempo total Quintic:     {(len(s_quintic_async)-1)*dt:.2f}s")
    print(f"Tiempo total Septic:      {(len(s_septic_async)-1)*dt:.2f}s (El más lento y seguro)")
    
    plot_comparative_kinematics(dt, s_trap_async, s_quintic_async, s_septic_async, 
                                "EXPERIMENTO 1: MODO ASÍNCRONO (Límites Fijos)")

    # ==========================================================
    # EXPERIMENTO 2: MODO SÍNCRONO (Tiempo Fijo)
    # ==========================================================
    T_TOTAL = 3.0 # Segundos exactos
    print(f"\n--- MODO SÍNCRONO (Tiempo Exacto = {T_TOTAL}s) ---")

    # 2A. Trapezoidal Síncrono
    xyz_trap_sync, _, _ = planner.generate_linear_trajectory(
        P0, P1, profile='trapezoidal', total_time=T_TOTAL)
    s_trap_sync = np.linalg.norm(xyz_trap_sync - P0, axis=1)

    # 2B. Quintic Síncrono
    xyz_quintic_sync, _, _ = planner.generate_linear_trajectory(
        P0, P1, profile='quintic', total_time=T_TOTAL)
    s_quintic_sync = np.linalg.norm(xyz_quintic_sync - P0, axis=1)

    # 2C. Septic Síncrono
    xyz_septic_sync, _, _ = planner.generate_linear_trajectory(
        P0, P1, profile='septic', total_time=T_TOTAL)
    s_septic_sync = np.linalg.norm(xyz_septic_sync - P0, axis=1)

    print(f"Tiempo total Trapezoidal: {(len(s_trap_sync)-1)*dt:.2f}s")
    print(f"Tiempo total Quintic:     {(len(s_quintic_sync)-1)*dt:.2f}s")
    print(f"Tiempo total Septic:      {(len(s_septic_sync)-1)*dt:.2f}s")

    plot_comparative_kinematics(dt, s_trap_sync, s_quintic_sync, s_septic_sync, 
                                f"EXPERIMENTO 2: MODO SÍNCRONO (T={T_TOTAL}s)")

    plt.show()