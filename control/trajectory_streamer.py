import numpy as np
from typing import Optional
from interfaces.communication import ICommunication

class StreamState:
    IDLE = "IDLE"
    WAITING_ACK = "WAITING_ACK"
    STREAMING = "STREAMING"

class TrajectoryStreamer:
    """
    Orquesta el envío segmentado de una matriz de trayectoria usando NumPy.
    Implementa el protocolo de Handshake estricto con el microcontrolador.
    """
    def __init__(self, comm: ICommunication, chunk_size: int = 15):
        self.comm = comm
        self.chunk_size = chunk_size
        self.state = StreamState.IDLE
        
        self.trajectory: Optional[np.ndarray] = None
        self.current_index = 0
        self.total_points = 0
        
        # Suscribirse a los mensajes entrantes de la capa de comunicación
        self.comm.set_rx_callback(self._on_message_received)

    def start_streaming(self, trajectory_matrix: np.ndarray) -> None:
        """
        Punto de entrada. Prepara la matriz y solicita inicio al STM32.
        No bloquea el hilo principal.
        """
        if self.state != StreamState.IDLE:
            print("[Streamer] Advertencia: Ya hay un streaming en curso.")
            return

        # Asegurar tipo y vista NumPy
        self.trajectory = np.asarray(trajectory_matrix, dtype=np.float32)
        self.total_points = self.trajectory.shape[0]
        self.current_index = 0
        
        if self.total_points == 0:
            return

        self.state = StreamState.WAITING_ACK
        print("[Streamer] Enviando M112. Esperando ACK del STM32...")
        self.comm.send_message("M112")

    def _on_message_received(self, msg: str) -> None:
        """
        Callback invocado por el hilo de RX del UART.
        Actúa como la Máquina de Estados reactiva del streaming.
        """
        if self.state == StreamState.WAITING_ACK and msg == "ACK_M112":
            print("[Streamer] ACK recibido. Iniciando transmisión del primer chunk.")
            self.state = StreamState.STREAMING
            self._send_next_chunk()
            
        elif self.state == StreamState.STREAMING and msg == "REQ_CHUNK":
            self._send_next_chunk()

    def _send_next_chunk(self) -> None:
        """Extrae la siguiente vista de NumPy y la envía."""
        if self.current_index >= self.total_points:
            print("[Streamer] Trayectoria finalizada. Enviando M113.")
            self.comm.send_message("M113")
            self.state = StreamState.IDLE
            return

        # Slicing eficiente de NumPy (O(1))
        end_idx = min(self.current_index + self.chunk_size, self.total_points)
        chunk_view = self.trajectory[self.current_index:end_idx, :]
        self.current_index = end_idx

        # Iteración sobre la vista matricial
        for row in chunk_view:
            cmd = f"P:{row[0]:.2f},{row[1]:.2f},{row[2]:.2f}"
            self.comm.send_message(cmd)
        
        print(f"[Streamer] Chunk enviado ({chunk_view.shape[0]} puntos). Progreso: {self.current_index}/{self.total_points}")