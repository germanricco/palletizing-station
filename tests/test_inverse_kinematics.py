import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# Resolución de rutas para importar módulos del proyecto
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from control.robot_EEZYbotMK2 import RoboticArmMK2
from visualization.robot_simulator import WireframeVisualizer3D

def generate_smooth_joint_trajectory(arm: RoboticArmMK2, num_points: int = 200) -> np.ndarray:
    """
    Genera una trayectoria armónica suave y mecánicamente válida en el espacio articular.
    """
    t = np.linspace(0, 2 * np.pi, num_points)
    
    # Oscilación de la base (q1) dentro de [-30, 30] grados
    q1 = np.sin(t) * np.deg2rad(25.0)
    
    # Oscilación del hombro (q2) dentro de un rango seguro
    q2_mean = np.deg2rad(80.0)
    q2_amp = np.deg2rad(15.0)
    q2 = q2_mean + np.cos(t) * q2_amp
    
    # Oscilación del codo (q3) respetando los límites dinámicos
    q3_min, q3_max = arm._calc_q3_limits(q2)
    q3_mean = (q3_max + q3_min) / 2.0
    q3_amp = (q3_max - q3_min) * 0.3 # Utilizar el 30% del margen disponible
    q3 = q3_mean + np.sin(t * 2) * q3_amp
    
    return np.column_stack((q1, q2, q3))

def plot_kinematic_validation(q_orig: np.ndarray, q_calc: np.ndarray, valid_mask: np.ndarray) -> None:
    """
    Genera un reporte gráfico contrastando las trayectorias articulares originales y reconstruidas.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title("Reporte de Validación: Cinemática Inversa")
    
    t_steps = np.arange(q_orig.shape[0])
    labels = ['Articulación 1 (Base)', 'Articulación 2 (Hombro)', 'Articulación 3 (Codo)']
    
    # Convertir a grados para legibilidad en el análisis
    q_orig_deg = np.rad2deg(q_orig)
    q_calc_deg = np.rad2deg(q_calc)
    
    for i in range(3):
        axs[i].plot(t_steps, q_orig_deg[:, i], 'b-', linewidth=2, label='Q Original (Input)')
        
        # Plotear solo los puntos válidos calculados por la IK
        axs[i].plot(t_steps[valid_mask], q_calc_deg[valid_mask, i], 'r--', linewidth=2, label='Q Calculado (IK)')
        
        axs[i].set_ylabel(f"{labels[i]}\n[Grados]")
        axs[i].grid(True, linestyle='--', alpha=0.7)
        axs[i].legend(loc='upper right')

    axs[-1].set_xlabel("Puntos de Trayectoria Discretos (N)")
    plt.suptitle("Validación de Ciclo Cerrado: $Q_{orig} \\rightarrow FK \\rightarrow XYZ \\rightarrow IK \\rightarrow Q_{calc}$", fontsize=14)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("Iniciando auditoría de cinemática bidireccional...")
    
    digital_twin = RoboticArmMK2()
    
    # 1. Generación de Trayectoria Original en Espacio Articular (Q)
    q_original = generate_smooth_joint_trajectory(digital_twin, num_points=250)
    
    # 2. Cinemática Directa (FK): Mapeo a Espacio Cartesiano (X)
    print("Ejecutando Cinemática Directa Vectorizada...")
    cartesian_tensor = digital_twin.forward_kinematics(q_original)
    tcp_trajectory = cartesian_tensor[:, 5, :] # Extracción exclusiva del TCP
    
    # 3. Cinemática Inversa (IK): Reconstrucción del Espacio Articular (Q_hat)
    print("Ejecutando Cinemática Inversa Vectorizada sobre trayectoria del TCP...")
    q_calculated, valid_mask = digital_twin.inverse_kinematics(tcp_trajectory)
    
    # 4. Análisis de Error Residual
    valid_points = np.sum(valid_mask)
    print(f"Puntos validados matemáticamente por IK: {valid_points}/{q_original.shape[0]}")
    
    if valid_points > 0:
        error_residual = np.abs(q_original[valid_mask] - q_calculated[valid_mask])
        mae_grados = np.rad2deg(np.mean(error_residual))
        max_error_grados = np.rad2deg(np.max(error_residual))
        
        print("-" * 50)
        print("MÉTRICAS DE ERROR GEOMÉTRICO:")
        print(f"Error Absoluto Medio (MAE): {mae_grados:.8e} grados")
        print(f"Error Máximo Absoluto:    {max_error_grados:.8e} grados")
        print("-" * 50)
        
        if mae_grados > 1e-4:
            print("[ADVERTENCIA] Desviación geométrica superior al límite de tolerancia de punto flotante.")
    else:
        print("[ERROR CRÍTICO] La cinemática inversa rechazó la totalidad de la trayectoria.")
        sys.exit(1)

    # 5. Reporte Gráfico Estático
    print("Generando gráficos de análisis (Cierre la ventana para iniciar simulación 3D)...")
    plot_kinematic_validation(q_original, q_calculated, valid_mask)
    
    # 6. Simulación Dinámica del Seguimiento
    print("Iniciando renderizado 3D de la trayectoria calculada...")
    visualizer = WireframeVisualizer3D()
    
    try:
        while plt.fignum_exists(visualizer.fig.number):
            for i in range(q_calculated.shape[0]):
                if not plt.fignum_exists(visualizer.fig.number):
                    break
                
                # Ignorar puntos que la IK marcó como inválidos o en singularidad
                if not valid_mask[i]:
                    continue
                
                # Inyectar el estado reconstruido en el Gemelo Digital
                digital_twin.update_state(q_calculated[i, 0], q_calculated[i, 1], q_calculated[i, 2])
                
                # Renderizar estado
                visualizer.draw_robot(digital_twin.get_cartesian_state())
                
                time.sleep(0.01) # Simular procesamiento a 100Hz para visualización fluida
                
    except KeyboardInterrupt:
        print("Simulación interrumpida.")
    finally:
        plt.ioff()
        plt.close('all')