import numpy as np
from typing import Tuple

from control.robot_EEZYbotMK2 import RoboticArmMK2

def generate_workspace_boundary(arm: 'RoboticArmMK2', resolution: int = 50) -> np.ndarray:
    """
    Genera la nube de puntos cartesiana (Nx3) correspondiente a la frontera del 
    espacio de trabajo del manipulador. Utiliza muestreo vectorizado sobre 
    las 6 superficies del espacio articular acotado.
    
    Parámetros:
        arm: Instancia inicializada del gemelo digital RoboticArmMK2.
        resolution: Cantidad de subdivisiones por eje para el muestreo espacial.
        
    Retorno:
        Matriz de dimensiones (N, 3) con las coordenadas (X, Y, Z) del TCP.
    """
    
    # 1. Vectores base de barrido lineal
    q1_sweep = np.linspace(arm.q1_min_rad, arm.q1_max_rad, resolution)
    q2_sweep = np.linspace(arm.q2_min_rad, arm.q2_max_rad, resolution)
    t_sweep = np.linspace(0.0, 1.0, resolution)
    
    boundary_points = []

    # =========================================================
    # SUPERFICIES 1 & 2: q1 estático en mínimos y máximos
    # =========================================================
    Q2_mesh, T_mesh = np.meshgrid(q2_sweep, t_sweep)
    Q2_flat = Q2_mesh.flatten()
    T_flat = T_mesh.flatten()
    
    # Evaluación de límites dinámicos de q3 sobre la malla de q2
    q3_min_arr, q3_max_arr = arm._calc_q3_limits(Q2_flat)
    Q3_flat = q3_min_arr + T_flat * (q3_max_arr - q3_min_arr)
    
    Q1_min_flat = np.full_like(Q2_flat, arm.q1_min_rad)
    Q1_max_flat = np.full_like(Q2_flat, arm.q1_max_rad)
    
    boundary_points.append(np.column_stack((Q1_min_flat, Q2_flat, Q3_flat)))
    boundary_points.append(np.column_stack((Q1_max_flat, Q2_flat, Q3_flat)))

    # =========================================================
    # SUPERFICIES 3 & 4: q2 estático en mínimos y máximos
    # =========================================================
    Q1_mesh, T_mesh = np.meshgrid(q1_sweep, t_sweep)
    Q1_flat = Q1_mesh.flatten()
    T_flat = T_mesh.flatten()
    
    # Evaluaciones de q3 en los extremos estáticos de q2
    q3_min_at_q2min, q3_max_at_q2min = arm._calc_q3_limits(arm.q2_min_rad)
    Q3_q2min_flat = q3_min_at_q2min + T_flat * (q3_max_at_q2min - q3_min_at_q2min)
    
    q3_min_at_q2max, q3_max_at_q2max = arm._calc_q3_limits(arm.q2_max_rad)
    Q3_q2max_flat = q3_min_at_q2max + T_flat * (q3_max_at_q2max - q3_min_at_q2max)
    
    Q2_min_flat = np.full_like(Q1_flat, arm.q2_min_rad)
    Q2_max_flat = np.full_like(Q1_flat, arm.q2_max_rad)
    
    boundary_points.append(np.column_stack((Q1_flat, Q2_min_flat, Q3_q2min_flat)))
    boundary_points.append(np.column_stack((Q1_flat, Q2_max_flat, Q3_q2max_flat)))

    # =========================================================
    # SUPERFICIES 5 & 6: q3 estático en mínimos y máximos dinámicos
    # =========================================================
    Q1_mesh, Q2_mesh = np.meshgrid(q1_sweep, q2_sweep)
    Q1_flat = Q1_mesh.flatten()
    Q2_flat = Q2_mesh.flatten()
    
    Q3_min_flat, Q3_max_flat = arm._calc_q3_limits(Q2_flat)
    
    boundary_points.append(np.column_stack((Q1_flat, Q2_flat, Q3_min_flat)))
    boundary_points.append(np.column_stack((Q1_flat, Q2_flat, Q3_max_flat)))

    # =========================================================
    # PROCESAMIENTO CINEMÁTICO VECTORIZADO
    # =========================================================
    # Concatenación de todas las matrices de superficies (N_total x 3)
    q_boundary_matrix = np.vstack(boundary_points)
    
    # Invocación de la cinemática directa. Retorna tensor (N_total, 6, 3)
    cartesian_tensor = arm.forward_kinematics(q_boundary_matrix)
    
    # Extracción de la capa correspondiente al TCP (índice 5)
    tcp_boundary_points = cartesian_tensor[:, 5, :]
    
    return tcp_boundary_points




def generate_lateral_boundary_path(arm: 'RoboticArmMK2', resolution: int = 100) -> Tuple[np.ndarray, np.ndarray]:
    """
    Genera un ciclo continuo (C0) en el espacio articular que recorre el perímetro
    exacto de la envolvente de alcance en el plano lateral (q1 = 0), resolviendo
    las transiciones en los límites de saturación de q2.
    
    Retorno:
        q_path_matrix: Tensor (N, 3) con la trayectoria articular continua.
        tcp_boundary_points: Tensor (N, 3) con las coordenadas cartesianas del TCP.
    """
    # Resolución reducida para las transiciones puramente verticales (q2 estático)
    trans_resolution = resolution // 2

    # =========================================================================
    # 1. Arista Inferior (Avance): q2 [min -> max] con q3 = q3_min
    # =========================================================================
    q2_e1 = np.linspace(arm.q2_min_rad, arm.q2_max_rad, resolution)
    q3_min_e1, _ = arm._calc_q3_limits(q2_e1)
    q3_e1 = q3_min_e1

    # =========================================================================
    # 2. Arista Derecha (Transición): q2 estático en max, q3 [min -> max]
    # =========================================================================
    q3_min_at_q2max, q3_max_at_q2max = arm._calc_q3_limits(arm.q2_max_rad)
    q2_e2 = np.full(trans_resolution, arm.q2_max_rad)
    q3_e2 = np.linspace(q3_min_at_q2max, q3_max_at_q2max, trans_resolution)

    # =========================================================================
    # 3. Arista Superior (Retroceso): q2 [max -> min] con q3 = q3_max
    # =========================================================================
    q2_e3 = np.linspace(arm.q2_max_rad, arm.q2_min_rad, resolution)
    _, q3_max_e3 = arm._calc_q3_limits(q2_e3)
    q3_e3 = q3_max_e3

    # =========================================================================
    # 4. Arista Izquierda (Transición): q2 estático en min, q3 [max -> min]
    # =========================================================================
    q3_min_at_q2min, q3_max_at_q2min = arm._calc_q3_limits(arm.q2_min_rad)
    q2_e4 = np.full(trans_resolution, arm.q2_min_rad)
    q3_e4 = np.linspace(q3_max_at_q2min, q3_min_at_q2min, trans_resolution)

    # =========================================================================
    # Ensamblaje Vectorizado del Ciclo Cerrado Continuo
    # =========================================================================
    q2_path = np.concatenate((q2_e1, q2_e2, q2_e3, q2_e4))
    q3_path = np.concatenate((q3_e1, q3_e2, q3_e3, q3_e4))
    q1_path = np.zeros_like(q2_path)  # Restricción planar lateral

    q_path_matrix = np.column_stack((q1_path, q2_path, q3_path))

    # Invocación de la cinemática directa tensorizada para extraer el TCP O(1)
    cartesian_tensor = arm.forward_kinematics(q_path_matrix)
    tcp_boundary_points = cartesian_tensor[:, 5, :]

    return q_path_matrix, tcp_boundary_points