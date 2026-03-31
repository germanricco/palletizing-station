from abc import ABC, abstractmethod
from typing import Callable

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