import serial
import logging
import threading
import queue
import time
from typing import Callable, Optional
from abc import ABC, abstractmethod

class ICommunication(ABC):
    """
    Contrato base para cualquier medio de comunicación bidireccional asíncrono.
    """
    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def send_message(self, message: str) -> bool:
        """Envía una cadena de texto sin bloquear el hilo principal."""
        pass

    @abstractmethod
    def set_rx_callback(self, callback: Callable[[str], None]) -> None:
        """Registra la función que será llamada cuando llegue un mensaje."""
        pass

"""
Notas:
1. Utiliza Threading (Hilos) uno exclusivo para RX y otro para TX.
2. La cola de transmisión (tx_queue) es un buffer FIFO thread-safe para mensajes salientes. (Actua como Pipeline o Tuberia)
3. El método send_message() es no bloqueante y encola mensajes para ser enviados por el hilo de TX.
4. El método set_rx_callback() permite registrar una función que será llamada cada vez que llegue un nuevo mensaje desde el microcontrolador.
5. El hilo de RX lee constantemente el puerto serie y dispara el callback con el mensaje decodificado.
6. El hilo de TX consume la cola de mensajes y los envía físicamente al puerto serie, añadiendo un terminador de línea esperado por el firmware del STM32.
7. La implementación maneja errores comunes de conexión y asegura que los hilos se cierren correctamente al desconectar.
"""

class AsyncUARTCommunicator(ICommunication):
    """
    Implementación concreta de ICommunication utilizando PySerial y concurrencia.
    Actúa como un conducto (pipe). solo junta y transfiere strings sin bloquear el hilo principal.
    """

    # Tope del buffer de linea en RX: si se acumulan mas bytes sin un '\n' es basura (ruido,
    # baudrate incorrecto) y se descarta para no crecer sin limite.
    _RX_MAX_LINE_BYTES = 4096

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        # La conexión serial real. Se inicializa en None hasta que se llame a connect()
        self.serial_conn: Optional[serial.Serial] = None
        
        # Cola de transmisión FIFO thread-safe
        self.tx_queue = queue.Queue(maxsize=500)
        
        self._running = False
        # Variables para los hilos de RX y TX
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        
        # Callback único requerido por ICommunication
        self._rx_callback: Optional[Callable[[str], None]] = None

        # Callback opcional para desconexiones NO deliberadas (excepcion en RX/TX), distinto
        # de disconnect() que es una desconexion pedida explicitamente por el usuario.
        self._disconnect_callback: Optional[Callable[[], None]] = None

        self.logger = logging.getLogger("STM32_Controller.UART")

    def connect(self) -> bool:
        """Abre el puerto serie y levanta los hilos daemon de trabajo."""
        try:
            # timeout=0.1: la lectura de _rx_loop no se bloquea indefinidamente sin datos
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            
            self._running = True
            # daemon=True asegura que los hilos se cierren si el programa principal (GUI) se cierra
            self._rx_thread = threading.Thread(target=self._rx_loop, name="UART_RX", daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, name="UART_TX", daemon=True)
            
            # Iniciar los hilos de trabajo en paralelo
            self._rx_thread.start()
            self._tx_thread.start()
            self.logger.info(f"Conexión establecida en {self.port} a {self.baudrate} bps.")
            return True
        
        except serial.SerialException as e:
            self.logger.error(f"Fallo al abrir el puerto {self.port}: {e}")
            return False

    def disconnect(self) -> None:
        """Detiene los hilos de forma segura y libera el puerto de hardware."""
        self._running = False
        
        # Esperar hasta 1 segundo a que los hilos terminen sus ciclos limpiamente
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._tx_thread and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=1.0)
            
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.logger.info(f"Puerto {self.port} cerrado correctamente.")

    def send_message(self, message: str) -> bool:
        """
        Implementación de la interfaz.
        Encola un string para ser enviado. Hilo-Seguro y no bloqueante.
        """
        try:
            self.tx_queue.put_nowait(message)
            return True
        except queue.Full:
            self.logger.error("Buffer TX de la PC lleno. Mensaje descartado.")
            return False

    def set_rx_callback(self, callback: Callable[[str], None]) -> None:
        """
        Registra la función de la capa superior (ej. TrajectoryStreamer)
        que será notificada cuando llegue un nuevo string.
        """
        self._rx_callback = callback

    def set_disconnect_callback(self, callback: Callable[[], None]) -> None:
        """
        Registra la función notificada cuando el enlace se cae de forma NO deliberada
        (excepcion en el hilo de RX o TX), a diferencia de disconnect() que es siempre
        una desconexion pedida explicitamente por el usuario.
        """
        self._disconnect_callback = callback

    def _notify_unexpected_disconnect(self) -> None:
        """
        Marca _running en False y dispara el callback de desconexion inesperada una única
        vez, aunque RX y TX detecten la caida casi al mismo tiempo.
        """
        was_running = self._running
        self._running = False
        if was_running and self._disconnect_callback:
            self._disconnect_callback()

    # ==========================================
    # Hilos de Trabajo Independientes (Workers)
    # ==========================================

    def _tx_loop(self) -> None:
        """Consume la cola tx_queue y escribe físicamente en el puerto."""
        while self._running:
            try:
                message = self.tx_queue.get(timeout=0.05)
                if self.serial_conn and self.serial_conn.is_open:
                    # Añadir terminador de línea esperado por el firmware C/C++ del STM32
                    encoded_msg = f"{message}\n".encode('ascii')
                    self.serial_conn.write(encoded_msg)
            except queue.Empty:
                pass
            except serial.SerialException as e:
                self.logger.error(f"Excepción en escritura TX: {e}")
                self._notify_unexpected_disconnect()

    def _rx_loop(self) -> None:
        """
        Escucha constantemente el puerto y dispara el callback por cada linea recibida.

        Lee por bloques (todo lo disponible) en vez de usar serial.readline(): en pyserial,
        readline() lee de a UN byte (un select + un read por caracter), y en cada uno suelta y
        vuelve a pedir el GIL. Con el hilo de la GUI ocupado, cada re-adquisicion puede esperar
        varios ms y la lectura queda por debajo de la tasa de telemetria (50 Hz): el atraso
        crece sin limite y los REQ_CHUNK se atienden tarde.
        """
        buffer = bytearray()

        while self._running:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    # Bloquea hasta timeout (0.1 s) si no hay datos; si los hay, trae todos.
                    data = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                    if not data:
                        continue
                    buffer.extend(data)

                    # Despachar cada linea completa (el firmware termina con \r\n)
                    while True:
                        idx = buffer.find(b"\n")
                        if idx < 0:
                            break
                        line_bytes = bytes(buffer[:idx])
                        del buffer[:idx + 1]

                        # 'ascii' con 'ignore' evita errores por caracteres corruptos.
                        line = line_bytes.decode('ascii', errors='ignore').strip()
                        if line and self._rx_callback:
                            self._rx_callback(line)

                    if len(buffer) > self._RX_MAX_LINE_BYTES:
                        self.logger.warning("Linea RX demasiado larga sin terminador. Descartada.")
                        buffer.clear()

                except (serial.SerialException, OSError) as e:
                    # in_waiting es un ioctl directo: ante un desenchufe del USB lanza OSError
                    # (EIO) sin envolver en SerialException. Sin capturarlo, el hilo RX moriria
                    # en silencio y no se dispararia el bloqueo por perdida de enlace.
                    self.logger.error(f"Excepción en lectura RX: {e}")
                    self._notify_unexpected_disconnect()
            else:
                # Prevenir saturación de CPU si el puerto se cierra inesperadamente
                time.sleep(0.01)
