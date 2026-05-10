import os
import numpy as np
import logging
from typing import List

class TrajectoryRepository:
    """
    Se encarga de la persistencia de datos matriciales (física).
    """
    # ==== INICIALIZACION
    def __init__(self, storage_dir: str = "trajectories", expected_columns: int = 6):
        """
        Inicializa el repositorio.

        Parametros:
            storage_dir: Directorio donde se guardarán los archivos binarios.
            expected_columns: Dimension D de la matriz (N, D)
        """
        self.storage_dir = storage_dir
        self.expected_columns = expected_columns
        os.makedirs(self.storage_dir, exist_ok=True)
        self.logger = logging.getLogger("STM32_Controller.TrajectoryRepo")

    def _sanitize_filename(self, name: str) -> str:
        """
        Extrae solo el nombre base del archivo sin extensión.
        """
        base_name = os.path.basename(name)
        if not base_name.endswith('.npy'):
            base_name += '.npy'
        return base_name

    
    # ==== OPERACIONES DE PERSISTENCIA ====
    def save_trajectory(self, name: str, trajectory_matrix: np.ndarray) -> bool:
        """
        Valida y guarda una matriz NumPy en el disco.
        """
        # Validacion de Tipo
        if not isinstance(trajectory_matrix, np.ndarray):
            self.logger.error(f"Fallo al guardar '{name}': El objeto proporcionado no es un np.ndarray.")
            return False
        
        # Validación Dimensional
        if trajectory_matrix.ndim != 2:
            self.logger.error(f"Fallo al guardar '{name}': La matriz debe ser bidimensional. Shape actual: {trajectory_matrix.shape}.")
            return False
        
        # Validacion Estructural
        if trajectory_matrix.shape[1] != self.expected_columns:
            self.logger.error(f"Fallo al guardar '{name}': Incompatibilidad de columnas. Esperadas: {self.expected_columns}, Recibidas: {trajectory_matrix.shape[1]}.")
            return False

        # Prevencion de guardado de matrices vacias
        if trajectory_matrix.size == 0:
            self.logger.warning(f"La trayectoria '{name}' está vacía (0 puntos). Guardado abortado.")
            return False
            
        safe_name = self._sanitize_filename(name)
        filepath = os.path.join(self.storage_dir, safe_name)
        
        try:
            np.save(filepath, trajectory_matrix)
            self.logger.debug(f"Matriz '{name}' (Shape: {trajectory_matrix.shape}) guardada exitosamente en '{self.storage_dir}'.")
            return True
        except IOError as e:
            self.logger.error(f"Error I/O escribiendo '{name}: {e}")
            return False

    def load_trajectory(self, name: str) -> np.ndarray:
        """
        Carga una matriz binaria desde el disco hacia la memoria RAM.
        Retorna la matriz, o una matriz vacía si ocurre un error.
        """
        safe_name = self._sanitize_filename(name)

        filepath = os.path.join(self.storage_dir, safe_name)

        # Matriz de retorno por defecto
        empty_fallback = np.empty((0, self.expected_columns), dtype=np.float32)

        if not os.path.exists(filepath):
            self.logger.warning(f"Intento de carga fallido. Archivo '{filepath}' no encontrado en el repositorio.")
            return empty_fallback
            
        try:
            # [Seguridad] allow_pickle=False evita la ejecución de código Python inyectado
            matrix = np.load(filepath, allow_pickle=False)

            # Validación Post-Carga
            if matrix.ndim != 2 or matrix.shape[1] != self.expected_columns:
                self.logger.error(f"Incompatibilidad detectada en '{safe_name}'. Shape leído: {matrix.shape}. Esperado: (N, {self.expected_columns}).")
                return empty_fallback

            self.logger.debug(f"Matriz '{safe_name}' cargada exitosamente. Shape: {matrix.shape}.")
            return matrix
        
        except Exception as e:
            self.logger.error(f"Corrupcion de datos al intentar cargar '{safe_name}': {e}")
            return empty_fallback
        
    
    # ==== METODOS DE USABILIDAD ====
    def list_trajectories(self) -> List[str]:
        """
        Escanea el directorio de almacenamiento y retorna los nombres de 
        las trayectorias disponibles (sin extensión).
        Útil para poblar ComboBoxes o ListBox en la UI.
        """
        try:
            files = os.listdir(self.storage_dir)
            return [f.replace('.npy', '') for f in files if f.endswith('.npy')]
        except Exception as e:
            self.logger.error(f"No se pudo listar el contenido del directorio '{self.storage_dir}': {e}")
            return []

    def delete_trajectory(self, name: str) -> bool:
        """
        Elimina un archivo de trayectoria del disco de forma permanente.
        """
        safe_name = self._sanitize_filename(name)
        filepath = os.path.join(self.storage_dir, safe_name)
        
        if os.path.exists(filepath):
            try:
                os.remove(filepath)
                self.logger.info(f"Trayectoria '{safe_name}' eliminada del repositorio.")
                return True
            except OSError as e:
                self.logger.error(f"No se pudo eliminar '{safe_name}': {e}")
                return False
        else:
            self.logger.warning(f"Intento de borrado fallido. '{safe_name}' no existe.")
            return False