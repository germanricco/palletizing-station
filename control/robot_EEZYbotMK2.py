import numpy as np
from typing import Tuple, Optional
import logging

class RoboticArmMK2:
    """
    Gemelo Dgital para el robot EEZYbotARM MK2.
    Implementa cinematica para generacion de trayectorias (tensorizada).
    """
    # ==== INICIALIZACION Y PARAMETROS FISICOS ====
    def __init__(self, initial_q_rad: Optional[np.ndarray] = None):
        """
        Inicializa la topologia del manipulador y las estructuras de estado.
        Todas las dimensiones expresadas en mm.
        """
        self.logger = logging.getLogger("STM32_Controller.RoboticArmMK2")
        self.logger.info("Inicializando Gemelo Digital EEZTbotARM MK2.")

        # Parametros Geometricos Fisicos
        #! Obtenidos de Repositorio de Referencia.
        self.L1: float = 92.0       # MCS -> J1
        self.L2: float = 135.0      # J1 -> J2
        self.L3: float = 147.0      # J2 -> WP
        self.L4: float = 25.0       # WP -> TCP (reestriccion de paralelogramo)

        # Restricciones
        self.singularity_margin_mm = 3.0   # Margen de seguridad para evitar singularidad en frontera
        
        # Limites Fisicos Estaticos (radianes)
        self.q1_min_rad = np.deg2rad(0.0)
        self.q1_max_rad = np.deg2rad(180.0)
        self.q2_min_rad = np.deg2rad(39.0)
        self.q2_max_rad = np.deg2rad(120.0)

        # Transformacion de Celda de Trabajo (GCS a MCS)
        self.T_GCS_MCS = np.eye(4, dtype=np.float32)

        #! Vectores de calibracion de hardware
        # q1: directo, q2: directo, q3:invertido
        self.hw_dir = np.array([1.0, 1.0, -1.0], dtype=np.float32)
        # offsets en radiantes
        self.hw_offset = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Estado Interno
        if initial_q_rad is None:
            self.q = np.zeros(3, dtype=np.float32)
        else:
            self.q = np.array(initial_q_rad, dtype=np.float32)
        self.logger.debug(f"Estado inicial de articulaciones (rad): {self.q}")

        # Cache de Coordenadas cartesianas absolutas. (1, 6, 3) [GCS, MCS, J1, J2, WP, TCP]
        self.cartesian_state = np.zeros((1, 6, 3), dtype=np.float32)
        self._update_internal_state(self.q)

    
    # ==== GETTERS (INTERFAZ DE LECTURA DE ESTADO) ====
    def get_angular_state(self) -> np.ndarray:
        """Retorna copia del vector de estado articular actual [q1, q2, q3]"""
        return self.q.copy()
    
    def get_cartesian_state(self) -> np.ndarray:
        """Retorna la matriz (6, 3) del estado actual absoluto (GCS)."""
        return self.cartesian_state[0].copy()


    # ==== CAPA DE ADAPTACION HARDWARE <-> MATEMATICA
    def servo_to_kinematic(self, q_servo: np.ndarray) -> np.ndarray:
        """
        Traduce valores crudos de los servomotores al espacio matemático.
        q_servo en radianes
        """
        return (q_servo * self.hw_dir) + self.hw_offset

    def kinematic_to_servo(self, q_kin: np.ndarray) -> np.ndarray:
        """Traduce ángulos matemáticos a valores físicos de los servomotores."""
        return (q_kin - self.hw_offset) / self.hw_dir
    
    def map_kinematic_trajectory_to_servos(self, q_kin_matrix: np.ndarray) -> np.ndarray:
        """
        Recibe una matriz de trayectoria calculada matemáticamente (N, 3)
        y la traduce de vuelta a ángulos que el microcontrolador entienda (0 a 180°).
        """
        # Operación vectorizada: Q_servo = (Q_kin - offset) / dir
        q_servo_matrix = (q_kin_matrix - self.hw_offset) / self.hw_dir
        return q_servo_matrix
    
    
    # ==== GESTION DE ESTADO INTERNO Y RESTRICCIONES ====
    def update_state(self, q1_servo_rad: float, q2_servo_rad: float, q3_servo_rad: float) -> None:
        """
        Actualiza el estado interno.
        Realiza saturacion de valores anomalos a los limites fisicos
        """
        q_servo = np.array([q1_servo_rad, q2_servo_rad, q3_servo_rad], dtype=np.float32)
        q_kin = (q_servo * self.hw_dir) + self.hw_offset

        q1_safe = np.clip(q_kin[0], self.q1_min_rad, self.q1_max_rad)
        q2_safe = np.clip(q_kin[1], self.q2_min_rad, self.q2_max_rad)

        # Calcular limites q3 dinamicamente
        q3_min, q3_max = self._calc_q3_limits(q2_safe)
        q3_safe = np.clip(q_kin[2], q3_min, q3_max)

        q_safe = np.array([q1_safe, q2_safe, q3_safe], dtype=np.float32)

        self._update_internal_state(q_safe)
    
    def _update_internal_state(self, q_vector: np.ndarray) -> None:
        """
        Actualiza la variable de estado y recalcula la cache de coordenadas cartesianas.
        """
        self.q[:] = q_vector
        # Se invoca la cinematica directa expandiendo las dimensiones a (1, 3)
        self.cartesian_state = self.forward_kinematics(self.q[np.newaxis, :])
        
    def _calc_q3_limits(self, q2_rad: float) -> Tuple[float, float]:
        """Calcula los límites dinámicos mecánicos para la articulación 3."""
        q2_deg = np.rad2deg(q2_rad)
        q3_min_deg = (-0.6755 * q2_deg) - 70.768
        q3_max_deg = (-0.7165 * q2_deg) - 13.144
        return np.deg2rad(q3_min_deg), np.deg2rad(q3_max_deg)   

    
    # ==== MOTOR DE CINEMATICA ====
    @staticmethod
    def _dh_tensor(thetas : np.ndarray, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Construye un tensor de matrices de Transformación Homogénea de DH (N, 4, 4).
        """
        # Cantidad de puntos en la trayectoria (dimensión temporal)
        N = thetas.shape[0]
        # Inicializacion de tensor de transformaciones (N, 4, 4)
        T = np.zeros((N, 4, 4), dtype=np.float32)
        
        ct = np.cos(thetas)
        st = np.sin(thetas)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T[:, 0, 0] = ct
        T[:, 0, 1] = -st * ca
        T[:, 0, 2] = st * sa
        T[:, 0, 3] = a * ct
        
        T[:, 1, 0] = st
        T[:, 1, 1] = ct * ca
        T[:, 1, 2] = -ct * sa
        T[:, 1, 3] = a * st
        
        T[:, 2, 1] = sa
        T[:, 2, 2] = ca
        T[:, 2, 3] = d
        
        T[:, 3, 3] = 1.0
        return T

    def forward_kinematics(self, q_matrix: np.ndarray) -> np.ndarray:
        """
        Cinemática Directa Vectorizada.

        Parametros:
            * q_matrix: np.ndarray de forma (N, 3) con N posturas articulares
        
        Retorno:
            * np.ndarray de forma (N, 6, 3) que representa el esqueleto completo para cada uno de los N puntos de la trayectoria
        """
        if q_matrix.ndim == 1:
            q_matrix = q_matrix[np.newaxis, :]

        N = q_matrix.shape[0]

        # Tensores relativos de Denavit-Hartenberg (N, 4, 4)
        # 0: MCS
        # 1: J1
        # 2: J2
        # 3: WP
        T_01 = self._dh_tensor(q_matrix[:, 0], self.L1, 0.0, np.pi/2)
        T_12 = self._dh_tensor(q_matrix[:, 1], 0.0, self.L2, 0.0)
        T_23 = self._dh_tensor(q_matrix[:, 2], 0.0, self.L3, 0.0)

        # Multiplicaciones matricial rensorizada
        T_02 = T_01 @ T_12
        T_03 = T_02 @ T_23

        # Premultiplicacion por matriz global 
        T_GCS_1 = self.T_GCS_MCS @ T_01
        T_GCS_2 = self.T_GCS_MCS @ T_02
        T_GCS_3 = self.T_GCS_MCS @ T_03

        # Ensamblaje del tensor resultante
        cartesian_tensor = np.zeros((N, 6, 3), dtype=np.float32)

        # Origenes estaticos
        cartesian_tensor[:, 0, :] = 0.0
        cartesian_tensor[:, 1, :] = self.T_GCS_MCS[:3, 3]

        # Extracción de traslaciones dinámicas (Slicing)
        cartesian_tensor[:, 2, :] = T_GCS_1[:, :3, 3]               # J1
        cartesian_tensor[:, 3, :] = T_GCS_2[:, :3, 3]               # J2
        cartesian_tensor[:, 4, :] = T_GCS_3[:, :3, 3]               # WP

        # Restricción del Paralelogramo (TCP vectorizado)
        R_GCS_MCS = self.T_GCS_MCS[:3, :3]
        
        # delta_tcp_local tiene dimensión (3, N)
        delta_tcp_local = np.array([
            self.L4 * np.cos(q_matrix[:, 0]),
            self.L4 * np.sin(q_matrix[:, 0]),
            np.zeros(N)
        ], dtype=np.float32)
        
        # Trasposición necesaria para compatibilidad de dimensiones (N, 3)
        delta_tcp_global = (R_GCS_MCS @ delta_tcp_local).T
        cartesian_tensor[:, 5, :] = cartesian_tensor[:, 4, :] + delta_tcp_global

        return cartesian_tensor
    
    def inverse_kinematics(self, xyz_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Cinemática inversa geométrica vectorizada.
        
        Parámetros:
            * xyz_matrix: Tensor de dimensiones (N, 3) con posiciones deseadas del TCP.
            
        Retorno:
            * (q_matrix, valid_mask): 
                - q_matrix: Tensor (N, 3) con los ángulos calculados.
                - valid_mask: Arreglo booleano (N,) indicando qué puntos son físicamente alcanzables.
        """
        if xyz_matrix.ndim == 1:
            xyz_matrix = xyz_matrix[np.newaxis, :]
            
        N = xyz_matrix.shape[0]
        x = xyz_matrix[:, 0]
        y = xyz_matrix[:, 1]
        z = xyz_matrix[:, 2]

        # Resolución de la base
        q1 = np.arctan2(y, x)

        # Desplazamiento del efector hacia la muñeca
        xw = x - self.L4 * np.cos(q1)
        yw = y - self.L4 * np.sin(q1)
        zw = z

        r_planar = np.hypot(xw, yw)
        z_diff = zw - self.L1
        d_squared = r_planar**2 + z_diff**2

        # Zona Muerta
        # Definicion de las fronteras teoricas del espacio de trabajo
        r_max_geometric = self.L2 + self.L3
        r_min_geometric = abs(self.L2 - self.L3)
        
        # Reduccion del espacio de trabajo para prevenir singularidad
        r_max_safe = r_max_geometric - self.singularity_margin_mm
        r_min_safe = r_min_geometric + self.singularity_margin_mm

        # Validación de Espacio de Trabajo Físico (Vectorizada)
        max_reach_safe_sq = r_max_safe**2
        min_reach_safe_sq = r_min_safe**2
        
        # Validación vectorizada de frontera
        valid_mask = (d_squared <= max_reach_safe_sq) & (d_squared >= min_reach_safe_sq)

        # Resolución Trigonométrica
        cos_q3 = (d_squared - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        
        q3 = -np.arccos(cos_q3)
        
        q2_alpha = np.arctan2(z_diff, r_planar)
        q2_beta = np.arctan2(self.L3 * np.sin(q3), self.L2 + self.L3 * np.cos(q3))
        q2 = q2_alpha - q2_beta

        # Validación de Límites Angulares (Joint Limits)
        valid_mask &= (q1 >= self.q1_min_rad) & (q1 <= self.q1_max_rad)
        valid_mask &= (q2 >= self.q2_min_rad) & (q2 <= self.q2_max_rad)
        
        q3_min, q3_max = self._calc_q3_limits(q2)
        valid_mask &= (q3 >= q3_min) & (q3 <= q3_max)

        # Logging de seguridad
        invalid_count = np.sum(~valid_mask)
        if invalid_count > 0:
            self.logger.warning(
                f"Prevención de Singularidad: {invalid_count}/{N} puntos bloqueados. "
                f"Zonas muertas: R_max < {r_max_safe}mm, R_min > {r_min_safe}mm."
            )


        q_matrix = np.column_stack((q1, q2, q3))
        # Asignar NaN a configuraciones inalcanzables o en colisión
        q_matrix[~valid_mask] = np.nan

        return q_matrix, valid_mask
        
