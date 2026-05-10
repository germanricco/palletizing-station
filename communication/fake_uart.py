import threading
import queue
import time
from typing import Callable, Optional

class FakeUARTCommunicator:
    """
    Simulador mínimo del STM32 para probar el handshaking sin hardware.
    Implementa el protocolo:
        M112 -> ACK_M112 -> REQ_CHUNK:n
        G1... acumula puntos
        cuando consume n puntos -> REQ_CHUNK:n
        M113 -> consume lo pendiente -> ACK_M113
    """

    def __init__(self, consumption_rate_hz: int = 50, chunk_size: int = 32):
        self.rx_callback: Optional[Callable[[str], None]] = None
        self.tx_queue = queue.Queue()
        self._running = False
        self._stm_thread: Optional[threading.Thread] = None

        self.point_consumption_time = 1.0 / consumption_rate_hz
        self.expected_chunk_size = chunk_size

    def set_rx_callback(self, callback: Callable[[str], None]) -> None:
        self.rx_callback = callback

    def connect(self) -> bool:
        self._running = True
        self._stm_thread = threading.Thread(target=self._simulate_stm32_firmware, daemon=True)
        self._stm_thread.start()
        print("[FakeUART] Puerto virtual abierto.")
        return True

    def disconnect(self) -> None:
        self._running = False
        if self._stm_thread:
            self._stm_thread.join(timeout=1.0)
        print("[FakeUART] Puerto virtual cerrado.")

    def send_message(self, message: str) -> bool:
        self.tx_queue.put_nowait(message)
        return True

    def _emit(self, message: str) -> None:
        if self.rx_callback:
            self.rx_callback(message)

    def _simulate_stm32_firmware(self) -> None:
        points_in_buffer = 0
        eof_received = False

        while self._running:
            try:
                msg = self.tx_queue.get(timeout=0.01)

                if msg == "M112":
                    time.sleep(0.02)
                    self._emit("ACK_M112")
                    self._emit(f"REQ_CHUNK:{self.expected_chunk_size}")

                elif msg.startswith("G1:"):
                    points_in_buffer += 1

                    if points_in_buffer >= self.expected_chunk_size:
                        time.sleep(self.point_consumption_time * points_in_buffer)
                        points_in_buffer = 0
                        if not eof_received:
                            self._emit(f"REQ_CHUNK:{self.expected_chunk_size}")

                elif msg == "M113":
                    eof_received = True
                    if points_in_buffer > 0:
                        time.sleep(self.point_consumption_time * points_in_buffer)
                        points_in_buffer = 0
                    self._emit("ACK_M113")

            except queue.Empty:
                continue
