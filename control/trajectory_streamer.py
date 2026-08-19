import logging
import numpy as np
from typing import Optional, Callable, Any
from enum import Enum


class StreamState(Enum):
    IDLE = 0
    WAITING_ACK = 1
    STREAMING = 2

class TrajectoryStreamer:
    """
    Orquesta el envío segmentado de una matriz de trayectoria usando NumPy.
    Implementa el protocolo de Handshake estricto con el microcontrolador.
    """
    # ==== INICIALIZACION ====
    def __init__(self, comm, chunk_size: int = 32, precision: int = 4, num_dofs: int = 3):
        """
        Inicializa el Streamer.

        Argumentos:
            comm: Instancia del comunicador UART (AsyncUARTCommunicator).
            chunk_size (int): Puntos a enviar por cada confirmacion del STM32.
            precision (int): Decimales para el formateo de radianes.
            num_dofs (int): Numero de grados de libertad
        """
        self.comm = comm
        self.chunk_size = chunk_size
        self.precision = precision
        self.num_dofs = num_dofs

        self.state = StreamState.IDLE
        self.trajectory: Optional[np.ndarray] = None
        self.current_index = 0
        self.total_points = 0

        # Memoria de estado para inyeccion diferencial de actuadores
        self._last_actuator_state: bool = False

        # Callback opcional para reportar avances
        self.progress_callback: Optional[Callable[[int], None]] = None

        # Logger
        self.logger = logging.getLogger("STM32_Controller.TrajectoryStreamer")

    def start_streaming(self, trajectory_matrix: np.ndarray) -> bool:
        """
        Valida la matriz y solicita el inicio del streaming al hardware.

        Retorna:
            bool: True si la matriz es valida y se inicio el handshake, False en caso contrario.
        """
        # Validacion de estado actual
        if self.state != StreamState.IDLE:
            self.logger.warning("Solicitud denegada: Streaming ya en progreso o esperando ACK.")
            return False
        
        # Validacion Estructural y de Integridad
        if trajectory_matrix is None or trajectory_matrix.size == 0:
            self.logger.error("Error crítico: Matriz de trayectoria vacía o nula.")
            return False
        
        if trajectory_matrix.shape[1] < self.num_dofs:
            self.logger.error(f"Error: La matriz debe tener al menos {self.num_dofs} columnas para la cinemática.")
            return False
            
        if np.isnan(trajectory_matrix).any() or np.isinf(trajectory_matrix).any():
            self.logger.error("Error crítico: La matriz contiene valores NaN o Infinitos. Abortando para proteger hardware.")
            return False

        # Inicializacion
        self.trajectory = np.asarray(trajectory_matrix, dtype=np.float32)
        self.total_points = self.trajectory.shape[0]
        self.current_index = 0

        self._last_actuator_state = False
        
        # Handshake
        self.state = StreamState.WAITING_ACK
        self.logger.info(f"Iniciando transmisión de {self.total_points} puntos. Solicitando handshake (M112).")
        return self.comm.send_message("M112")

    
    def process_message(self, msg: str) -> None:
        """
        Enrutador de confirmaciones de la capa de transporte.
        """
        clean_msg = msg.strip()

        if self.state == StreamState.WAITING_ACK and clean_msg == "ACK_M112":
            self.logger.debug("Handshake 'ACK_M112' aceptado. A la espera de buffer STM32...")
            self.state = StreamState.STREAMING
            
        elif self.state == StreamState.STREAMING and clean_msg == "REQ_CHUNK":
            self.logger.debug("Petición de bloque recibida. Transmitiendo chunk...")
            self._send_next_chunk()

    
    def reset(self) -> None:
        """
        Freno de emergencia local. Restablece el streamer y purga la memoria
        """

        if self.state != StreamState.IDLE:
            self.logger.warning(f"Abortando streaming asíncrono (Estado previo: {self.state.name}).")
            self.state = StreamState.IDLE
        
        # Limpieza exhaustiva de variables internas
        self.trajectory = None
        self.total_points = 0
        self.current_index = 0
        self._last_actuator_state = False

        # Reportar progreso
        self._report_progress(0)
        self.logger.debug("Caché y memoria del TrajectoryStreamer purgada.")

    def _send_next_chunk(self) -> None:
        """
        Motor de transmision principal.
        Maneja slicing eficiente, inyeccion de actuadores y formato G-Code
        """
        # Condicion de Fin de Trayectoria: M116 le indica al firmware que no llegaran mas
        # puntos, para que drene la FIFO restante y transicione solo a AutoReady (via
        # EVT:AUTO_COMPLETE) en vez de tratarlo como una pausa real (M113).
        if self.current_index >= self.total_points:
            self.logger.info("Trayectoria finalizada exitosamente. Transmitiendo 'M116' (EOF).")
            self.comm.send_message("M116")
            self.state = StreamState.IDLE
            self._report_progress(100)
            return
        
        # Slicing (Visa de memoria)
        end_idx = min(self.current_index + self.chunk_size, self.total_points)
        chunk_view = self.trajectory[self.current_index:end_idx, :]
        self.current_index = end_idx

        # Variables de formateo dinamico.
        has_actuators = chunk_view.shape[1] > self.num_dofs

        # Iteración sobre la vista matricial
        for row in chunk_view:
            # Construccion Unificada: Cinemática + Estado de Actuador
            s1, s2, s3 = row[0], row[1], row[2]
            
            # Si hay actuador, extraemos su estado (0 o 1). Si no, forzamos 0.
            a1 = int(row[self.num_dofs]) if has_actuators else 0
            
            cmd = f"G1:{s1:.{self.precision}f},{s2:.{self.precision}f},{s3:.{self.precision}f},{a1}"
            self.comm.send_message(cmd)

        self._report_progress()

    def _report_progress(self, force_value: Optional[int] = None) -> None:
        """Helper para calcular y despachar el avance porcentual."""
        if not self.progress_callback:
            return
            
        if force_value is not None:
            self.progress_callback(force_value)
        else:
            progress = int((self.current_index / self.total_points) * 100)
            self.progress_callback(progress)