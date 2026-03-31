import threading
import queue
import time
from typing import Callable, Optional
from interfaces.communication import ICommunication

class FakeUARTCommunicator(ICommunication):
    """
    Simulador del STM32 para pruebas unitarias sin hardware.
    """
    def __init__(self, consumption_rate_hz: int = 50, chunk_size: int = 15):
        self.rx_callback: Optional[Callable[[str], None]] = None
        self.tx_queue = queue.Queue()
        self._running = False
        self._stm_thread: Optional[threading.Thread] = None
        
        # Parámetros físicos simulados
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
            self._stm_thread.join()
        print("[FakeUART] Puerto virtual cerrado.")

    def send_message(self, message: str) -> bool:
        """La PC envía un mensaje. Lo encolamos para que el firmware falso lo procese."""
        self.tx_queue.put_nowait(message)
        return True

    def _simulate_stm32_firmware(self) -> None:
        """Hilo en background que actúa como el microcontrolador real."""
        points_in_buffer = 0
        
        while self._running:
            try:
                # Leemos lo que llega de la PC
                msg = self.tx_queue.get(timeout=0.01)
                
                if msg == "M112":
                    # Simular latencia de inicialización del microcontrolador (ej. 50ms)
                    time.sleep(0.05) 
                    if self.rx_callback:
                        self.rx_callback("ACK_M112")
                        
                elif msg.startswith("P:"):
                    points_in_buffer += 1
                    
                    # Si el buffer simulado alcanza el tamaño del chunk, empezamos a "consumirlos"
                    if points_in_buffer == self.expected_chunk_size:
                        # Simular el tiempo que tarda el brazo en ejecutar físicamente el chunk
                        time_to_execute = self.point_consumption_time * self.expected_chunk_size
                        time.sleep(time_to_execute)
                        points_in_buffer = 0
                        
                        # Pedir el siguiente chunk a la PC
                        if self.rx_callback:
                            self.rx_callback("REQ_CHUNK")
                            
                elif msg == "M113":
                    print("[STM32 Firmware] Modo automático finalizado. Motores en HOLD.")
                    points_in_buffer = 0
                    
            except queue.Empty:
                continue