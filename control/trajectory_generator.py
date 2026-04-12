import numpy as np
import logging
from typing import Tuple, Optional, Protocol

# =========================================================================
# 1. DEFINICIÓN DE INTERFACES (Principio de Inversión de Dependencias)
# =========================================================================

class IKinematicModel(Protocol):
    """
    Contrato que cualquier robot debe cumplir para ser compatible con el 
    generador de trayectorias. Aisla la matemática de la cinemática.
    """
    def inverse_kinematics(self, xyz_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ...
        
    def forward_kinematics(self, q_matrix: np.ndarray) -> np.ndarray:
        ...


# =========================================================================
# 2. GENERADOR DE TRAYECTORIAS
# =========================================================================

class TrajectoryGenerator:
    """
    Motor de planificación de trayectorias.
    Responsabilidad Única (SRP): Calcular puntos en el espacio y en el tiempo, 
    y mapearlos al espacio articular utilizando un modelo cinemático inyectado.
    """

    def __init__(self, robot_model: IKinematicModel, dt_seconds: float = 0.02):
        """
        Inicializa el generador inyectando sus dependencias.
        
        Parámetros:
            robot_model: Instancia que cumple con IKinematicModel (Ej: RoboticArmMK2).
            dt_seconds: Período de actualización por defecto (Ej: 0.02s = 50Hz).
        """
        self.logger = logging.getLogger("STM32_Controller.TrajectoryGenerator")
        self.robot = robot_model
        self.dt = dt_seconds
        
        # Estado Interno: Caché de las últimas trayectorias generadas
        self._last_cartesian_traj: Optional[np.ndarray] = None
        self._last_joint_traj: Optional[np.ndarray] = None
        self._last_valid_mask: Optional[np.ndarray] = None

    # =========================================================================
    # GETTERS: ACCESO A RESULTADOS
    # =========================================================================
    
    def get_cartesian_trajectory(self) -> Optional[np.ndarray]:
        """Retorna la última trayectoria calculada en el espacio cartesiano (N, 3)."""
        return self._last_cartesian_traj

    def get_joint_trajectory(self) -> Optional[np.ndarray]:
        """Retorna la última trayectoria calculada en el espacio articular (N, DOF)."""
        return self._last_joint_traj
        
    def get_validity_mask(self) -> Optional[np.ndarray]:
        """Retorna la máscara booleana (N,) de puntos alcanzables físicamente."""
        return self._last_valid_mask

    # =========================================================================
    # MÉTODOS PRIVADOS: NÚCLEO MATEMÁTICO TEMPORAL
    # =========================================================================

    def _generate_trapezoidal_profile(self, distance: float, v_max: float, a_max: float, dt: float) -> np.ndarray:
        """
        Genera el vector de distancia paramétrica s(t) normalizado [0, 1].
        """
        if distance <= 1e-6:
            return np.array([0.0])

        t_a = v_max / a_max
        d_a = 0.5 * a_max * t_a**2
        
        # Verificación de perfil triangular (La distancia es muy corta para alcanzar v_max)
        if 2 * d_a > distance:
            v_max = np.sqrt(a_max * distance)
            t_a = v_max / a_max
            d_a = distance / 2.0
            t_v = 0.0
        else:
            t_v = (distance - 2 * d_a) / v_max
            
        t_f = 2 * t_a + t_v
        
        # Discretización del tiempo
        t = np.arange(0, t_f, dt)
        
        # Asegurar que el último punto coincida exactamente con t_f
        if len(t) == 0 or not np.isclose(t[-1], t_f):
            t = np.append(t, t_f)
            
        s = np.zeros_like(t)
        
        # Vectorización condicional para mayor rendimiento (Reemplaza el for-loop)
        # Fase 1: Aceleración
        mask_accel = t <= t_a
        s[mask_accel] = 0.5 * a_max * t[mask_accel]**2
        
        # Fase 2: Velocidad Constante
        mask_const = (t > t_a) & (t <= (t_a + t_v))
        s[mask_const] = d_a + v_max * (t[mask_const] - t_a)
        
        # Fase 3: Desaceleración
        mask_decel = t > (t_a + t_v)
        s[mask_decel] = distance - 0.5 * a_max * (t_f - t[mask_decel])**2
            
        # Normalización del parámetro s en el rango [0, 1]
        return s / distance

    def _generate_quintic_profile(self, total_time: float, dt: float) -> np.ndarray:
        """
        Genera un perfil paramétrico s(t) polinómico de 5to orden (Síncrono).
        Asegura que v(0)=v(T)=0 y a(0)=a(T)=0.
        """
        if total_time <= dt:
            return np.array([0.0, 1.0])

        t = np.arange(0, total_time, dt)
        if len(t) == 0 or not np.isclose(t[-1], total_time):
            t = np.append(t, total_time)

        # Normalización del tiempo tau = [0 -> 1]
        tau = t / total_time 

        # Polinomio de 5to orden
        s_norm = 10 * (tau**3) - 15 * (tau**4) + 6 * (tau**5)
        
        return s_norm
    
    def _generate_septic_profile(self, total_time: float, dt: float) -> np.ndarray:
        """
        Genera un perfil paramétrico s(t) polinómico de 7mo orden.
        Asegura que v(0)=v(T)=0, a(0)=a(T)=0 Y jerk(0)=jerk(T)=0.
        """
        if total_time <= dt:
            return np.array([0.0, 1.0])

        t = np.arange(0, total_time, dt)
        if len(t) == 0 or not np.isclose(t[-1], total_time):
            t = np.append(t, total_time)

        tau = t / total_time 

        # Polinomio de 7mo orden
        s_norm = -20 * (tau**7) + 70 * (tau**6) - 84 * (tau**5) + 35 * (tau**4)
        
        return s_norm

    # =========================================================================
    # MÉTODOS PÚBLICOS: GENERADORES GEOMÉTRICOS
    # =========================================================================

    def generate_linear_trajectory(
        self, 
        p_start: np.ndarray, 
        p_end: np.ndarray, 
        profile: str = 'quintic',  # 'trapezoidal', 'quintic' o 'septic'
        v_max: Optional[float] = None, 
        a_max: Optional[float] = None,
        total_time: Optional[float] = None,
        override_dt: Optional[float] = None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Genera una trayectoria rectilínea cruzando Modos (Síncrono/Asíncrono) y Perfiles."""
        dt_to_use = override_dt if override_dt is not None else self.dt
        use_time_mode = total_time is not None
        use_limit_mode = (v_max is not None) and (a_max is not None)

        if not (use_time_mode ^ use_limit_mode):
            raise ValueError("Debe proveer [total_time] O [v_max y a_max], pero no ambos simultáneamente.")

        p_start = np.asarray(p_start, dtype=np.float32)
        p_end = np.asarray(p_end, dtype=np.float32)
        delta_p = p_end - p_start
        distance = np.linalg.norm(delta_p)

        if distance < 1e-3:
            return np.array([p_start]), np.array([[0,0,0]]), np.array([True])

        # --- PUENTES MATEMÁTICOS DE CONVERSIÓN ---
        if use_time_mode:
            T = total_time
            if profile == 'trapezoidal':
                v_req = (1.5 * distance) / T
                a_req = (4.5 * distance) / (T**2)
                s_norm = self._generate_trapezoidal_profile(distance, v_req, a_req, dt_to_use)
            elif profile == 'quintic':
                s_norm = self._generate_quintic_profile(T, dt_to_use)
            elif profile == 'septic':
                s_norm = self._generate_septic_profile(T, dt_to_use)
            else:
                raise ValueError("Perfil no soportado. Use 'trapezoidal', 'quintic' o 'septic'.")

        elif use_limit_mode:
            if profile == 'quintic' or profile == 'septic':
                # Nota: Las ecuaciones exactas de T mínimo para el Septic son más complejas,
                # Para simplificar en modo asíncrono, aproximamos usando los coeficientes máximos.
                # Max V del Septic = 2.1875 * (D/T), Max A = 7.51 * (D/T^2)
                coef_v = 1.875 if profile == 'quintic' else 2.1875
                coef_a = 5.7735 if profile == 'quintic' else 7.51
                
                t_v = (coef_v * distance) / v_max
                t_a = np.sqrt((coef_a * distance) / a_max)
                T = max(t_v, t_a)
                
                if profile == 'quintic':
                    s_norm = self._generate_quintic_profile(T, dt_to_use)
                else:
                    s_norm = self._generate_septic_profile(T, dt_to_use)
            else:
                s_norm = self._generate_trapezoidal_profile(distance, v_max, a_max, dt_to_use)

        xyz_matrix = p_start + s_norm[:, np.newaxis] * delta_p
        q_matrix, valid_mask = self.robot.inverse_kinematics(xyz_matrix)
        
        self._last_cartesian_traj, self._last_joint_traj, self._last_valid_mask = xyz_matrix, q_matrix, valid_mask
        return xyz_matrix, q_matrix, valid_mask