import numpy as np
from enum import Enum

class SystemState(Enum):
    """Estados finitos del sistema robótico."""
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    AUTOMATIC = "AUTOMATIC"
    RECORDING = "RECORDING"

class JointAngles:
    """
    DTO para un único punto articular. Útil para telemetría individual 
    o cálculos aislados, pero no para trayectorias completas.
    """
    def __init__(self, q1: float, q2: float, q3: float):
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def to_numpy(self) -> np.ndarray:
        """Convierte el DTO a un vector de NumPy."""
        return np.array([self.q1, self.q2, self.q3], dtype=np.float32)

    @classmethod
    def from_numpy(cls, arr: np.ndarray) -> 'JointAngles':
        """Instancia un DTO desde un vector de NumPy."""
        return cls(arr[0], arr[1], arr[2])

class CartesianPose:
    """Posición en el espacio cartesiano."""
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z