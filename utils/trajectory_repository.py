import os
import numpy as np
import logging

class TrajectoryRepository:
    """
    Se encarga de la persistencia de datos matriciales (física).
    """
    def __init__(self, storage_dir: str = "trajectories"):
        self.storage_dir = storage_dir
        os.makedirs(self.storage_dir, exist_ok=True)

        self.logger = logging.getLogger("STM32_Controller.TrajectoryRepo")

    def save_trajectory(self, name: str, trajectory_matrix: np.ndarray) -> bool:
        """
        Guarda una matriz NumPy en el disco.
        Retorna True si la operación fue exitosa, False en caso contrario.
        """
        # Validación de seguridad: Asegurarse de que realmente sea un objeto NumPy
        if not isinstance(trajectory_matrix, np.ndarray):
            self.logger.error("Fallo al guardar '{name}': El objeto proporcionado no es una matriz NumPy.")
            return False

        # Si el usuario no incluyó la extensión, se la agregamos
        if not name.endswith('.npy'):
            name += '.npy'
            
        filepath = os.path.join(self.storage_dir, name)
        
        try:
            np.save(filepath, trajectory_matrix)
            self.logger.debug(f"Matriz '{name}' (Shape: {trajectory_matrix.shape}) guardada exitosamente en '{self.storage_dir}'.")
            return True
        except IOError as e:
            # Si el disco está lleno o hay un problema de permisos, fallará aquí.
            self.logger.error(f"Error I/O escribiendo '{name}: {e}")
            return False

    def load_trajectory(self, name: str) -> np.ndarray:
        """
        Carga una matriz binaria desde el disco hacia la memoria RAM.
        Retorna la matriz, o una matriz vacía si ocurre un error.
        """
        if not name.endswith('.npy'):
            name += '.npy'
            
        filepath = os.path.join(self.storage_dir, name)
        
        if not os.path.exists(filepath):
            # Retorna un array vacío de 0 filas y 4 columnas (q1, q2, q3, actuador)
            self.logger.warning(f"Intento de carga fallido. Archivo {filepath} no encontrado.")
            return np.empty((0, 4), dtype=np.float32)
            
        try:
            # np.load lee el binario y reconstruye la matriz en RAM instantáneamente.
            matrix = np.load(filepath)
            self.logger.debug(f"Matriz '{name}' cargada exitosamente. Shape: {matrix.shape}. desde '{self.storage_dir}'.")
            return matrix
        except Exception as e:
            self.logger.error(f"Corrupcion de datos al cargar '{name}': {e}")
            return np.empty((0, 4), dtype=np.float32)