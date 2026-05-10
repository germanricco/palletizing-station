import logging
import numpy as np
from typing import Optional, Callable
from enum import Enum


class StreamState(Enum):
    """
    Estados internos del streamer.

    Importante:
    - La PC NO manda datos apenas recibe ACK_M112.
    - La PC espera que el STM32 pida capacidad con REQ_CHUNK o REQ_CHUNK:n.
    """
    IDLE = 0
    WAITING_ACK_M112 = 1
    WAITING_REQ_CHUNK = 2
    WAITING_ACK_M113 = 3


class TrajectoryStreamer:
    """
    Orquesta el envío segmentado de una matriz de trayectoria usando NumPy.

    Protocolo esperado:
        PC  -> STM: M112
        STM -> PC : ACK_M112
        STM -> PC : REQ_CHUNK:32
        PC  -> STM: G1:q1,q2,q3,a1    (hasta n puntos)
        STM -> PC : REQ_CHUNK:32       (cuando vuelve a tener espacio)
        PC  -> STM: M113               (cuando ya no quedan puntos por enviar)
        STM -> PC : ACK_M113           (cuando terminó de consumir su FIFO)
    """

    def __init__(self, comm, chunk_size: int = 32, precision: int = 4, num_dofs: int = 3):
        self.comm = comm
        self.chunk_size = chunk_size
        self.precision = precision
        self.num_dofs = num_dofs

        self.state = StreamState.IDLE
        self.trajectory: Optional[np.ndarray] = None
        self.current_index = 0
        self.total_points = 0

        self.progress_callback: Optional[Callable[[int], None]] = None
        self.logger = logging.getLogger("STM32_Controller.TrajectoryStreamer")

    def start_streaming(self, trajectory_matrix: np.ndarray) -> bool:
        """
        Valida la matriz y solicita al STM32 entrar en modo AUTO_RUNNING.
        No envía puntos todavía: espera REQ_CHUNK del firmware.
        """
        if self.state != StreamState.IDLE:
            self.logger.warning("Solicitud denegada: streaming ya iniciado o pendiente de ACK.")
            return False

        if trajectory_matrix is None or trajectory_matrix.size == 0:
            self.logger.error("Error crítico: matriz de trayectoria vacía o nula.")
            return False

        if trajectory_matrix.ndim != 2:
            self.logger.error("Error: la trayectoria debe ser una matriz 2D Nx4 o Nx3.")
            return False

        if trajectory_matrix.shape[1] < self.num_dofs:
            self.logger.error(
                f"Error: la matriz debe tener al menos {self.num_dofs} columnas para Q1, Q2 y Q3."
            )
            return False

        if np.isnan(trajectory_matrix).any() or np.isinf(trajectory_matrix).any():
            self.logger.error("Error crítico: la matriz contiene NaN o Inf. Abortando para proteger hardware.")
            return False

        self.trajectory = np.asarray(trajectory_matrix, dtype=np.float32)
        self.total_points = int(self.trajectory.shape[0])
        self.current_index = 0

        self.state = StreamState.WAITING_ACK_M112
        self._report_progress(0)

        self.logger.info(
            f"Iniciando transmisión de {self.total_points} puntos. Enviando M112 y esperando ACK_M112."
        )
        return self.comm.send_message("M112")

    def process_message(self, msg: str) -> None:
        """
        Procesa respuestas del STM32 relacionadas con streaming.
        Puede ser llamado desde el hilo RX del comunicador UART.
        """
        if msg is None:
            return

        msg = msg.strip()
        if not msg:
            return

        if msg.startswith("ERR:"):
            self.logger.error(f"STM32 reportó error durante streaming: {msg}")
            self.reset()
            return

        # ACK_M113 significa que el STM32 ya salió de la ejecución automática
        # porque consumió toda la FIFO o porque se canceló desde pausa.
        if msg == "ACK_M113" and self.state != StreamState.IDLE:
            self.logger.info("ACK_M113 recibido. Streaming cerrado por hardware.")
            self.state = StreamState.IDLE
            self.trajectory = None
            self.current_index = 0
            self.total_points = 0
            self._report_progress(100)
            return

        if self.state == StreamState.WAITING_ACK_M112:
            if msg == "ACK_M112":
                self.logger.info("ACK_M112 recibido. Esperando REQ_CHUNK del STM32.")
                self.state = StreamState.WAITING_REQ_CHUNK
            return

        if self.state == StreamState.WAITING_REQ_CHUNK:
            if msg.startswith("REQ_CHUNK"):
                requested_points = self._parse_req_chunk(msg)
                self._send_next_chunk(requested_points)
            return

        if self.state == StreamState.WAITING_ACK_M113:
            if msg == "ACK_M113":
                self.logger.info("ACK_M113 recibido. Trayectoria finalizada por hardware.")
                self.state = StreamState.IDLE
                self.trajectory = None
                self.current_index = 0
                self.total_points = 0
                self._report_progress(100)
            return

    def reset(self) -> None:
        """
        Restablece el streamer local. No envía comandos al STM32.
        """
        if self.state != StreamState.IDLE:
            self.logger.warning(f"Abortando streaming local. Estado previo: {self.state.name}.")

        self.state = StreamState.IDLE
        self.trajectory = None
        self.total_points = 0
        self.current_index = 0
        self._report_progress(0)

    def _parse_req_chunk(self, msg: str) -> int:
        """
        Acepta:
            REQ_CHUNK
            REQ_CHUNK:32
        Limita el valor al chunk_size configurado en la PC.
        """
        requested = self.chunk_size

        if ":" in msg:
            try:
                raw_value = int(msg.split(":", 1)[1].strip())
                requested = raw_value
            except ValueError:
                self.logger.warning(f"REQ_CHUNK inválido recibido: {msg}. Usando chunk_size local.")

        requested = max(1, min(int(requested), int(self.chunk_size)))
        return requested

    def _send_next_chunk(self, requested_points: int) -> None:
        """
        Envía como máximo la cantidad de puntos solicitada por el STM32.
        """
        if self.trajectory is None:
            self.logger.error("No hay trayectoria cargada en el streamer.")
            self.reset()
            return

        if self.current_index >= self.total_points:
            self._send_end_of_stream()
            return

        amount = min(int(requested_points), self.total_points - self.current_index)

        self.logger.debug(
            f"Enviando chunk de {amount} puntos. Índice inicial: {self.current_index}/{self.total_points}."
        )

        for _ in range(amount):
            row = self.trajectory[self.current_index]

            q1 = float(row[0])
            q2 = float(row[1])
            q3 = float(row[2])
            a1 = int(row[self.num_dofs]) if row.shape[0] > self.num_dofs else 0
            a1 = 1 if a1 else 0

            cmd = (
                f"G1:{q1:.{self.precision}f},"
                f"{q2:.{self.precision}f},"
                f"{q3:.{self.precision}f},"
                f"{a1}"
            )

            if not self.comm.send_message(cmd):
                self.logger.error("No se pudo encolar un punto G1 en la cola TX de Python.")
                self.reset()
                return

            self.current_index += 1

        self._report_progress()

        if self.current_index >= self.total_points:
            self._send_end_of_stream()
        else:
            self.state = StreamState.WAITING_REQ_CHUNK

    def _send_end_of_stream(self) -> None:
        """
        Avisa que la PC ya no enviará más puntos.
        El STM32 debe responder ACK_M113 recién cuando haya consumido su FIFO.
        """
        if self.state == StreamState.WAITING_ACK_M113:
            return

        self.logger.info("Todos los puntos fueron enviados. Enviando M113 y esperando ACK_M113.")
        if self.comm.send_message("M113"):
            self.state = StreamState.WAITING_ACK_M113
        else:
            self.logger.error("No se pudo enviar M113 al STM32.")
            self.reset()

    def _report_progress(self, force_value: Optional[int] = None) -> None:
        if not self.progress_callback:
            return

        if force_value is not None:
            self.progress_callback(int(force_value))
            return

        if self.total_points <= 0:
            self.progress_callback(0)
            return

        progress = int((self.current_index / self.total_points) * 100)
        self.progress_callback(progress)
