"""
Módulo de la Aplicación Principal.
Orquesta la inicialización de la interfaz gráfica, el controlador del robot
y establece los puentes de comunicación segura (Thread-Safe) entre ambos.
"""

import sys
import logging
from typing import Any, Dict, Callable
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QObject, Signal

# Importaciones de la vista y estilos
from views.main_window import MainWindow
from resources.styles.style_manager import StyleManager

# Importación hipotética del backend
# from app.core.robot_controller import RobotController


class SignalBridge(QObject):
    """
    Puente de señales para garantizar la seguridad entre hilos (Thread-Safety).
    Actúa como intermediario entre los callbacks del backend y los slots de la UI.
    """
    # Define la firma de los datos que enviará el controlador.
    # Ejemplo: dict con telemetría, estados, etc.
    telemetry_received = Signal(dict)
    log_received = Signal(str, str, str, str)  # timestamp, level, module, message


class RobotGUIApp:
    """
    Clase principal que orquesta el ciclo de vida de la aplicación.
    Instancia la UI, el backend y gestiona la inyección de dependencias.
    """

    def __init__(self) -> None:
        """Inicializa los componentes principales de la arquitectura."""
        # 1. Framework Qt
        self.app = QApplication(sys.argv)
        
        # 2. Componentes de la Arquitectura
        self.window = MainWindow()
        self.bridge = SignalBridge()
        
        # Mock del controlador (En producción: self.controller = RobotController())
        self.controller = self._init_mock_controller()

        # 3. Inicialización y Enlaces
        self._setup_logging()
        self._apply_styles()
        self._setup_connections()

    def _init_mock_controller(self) -> Any:
        """Crea un objeto mock o instancia el controlador real."""
        class MockController:
            ui_update_callback: Callable[[Dict[str, Any]], None] | None = None
            
            # NUEVO: Callback de logs del backend
            log_callback: Callable[[str, str, str, str], None] | None = None

            def start(self):
                logging.info("Controlador de Robot iniciado.")

            def send_manual_command(self, cmd: str) -> None:
                # Simulación de backend recibiendo un comando
                logging.info(f"STM32 Rx: {cmd}")
                if self.log_callback:
                    # Simulamos una respuesta del microcontrolador
                    self.log_callback("00:00:00.000", "INFO", "UART", f"Comando enviado: {cmd}")

        return MockController()

    def _setup_logging(self) -> None:
        """Configura el formato estándar para los logs de la aplicación."""
        logging.basicConfig(
            level=logging.INFO,
            format="[%(asctime)s] | [%(levelname)s] | [%(module)s] | %(message)s",
            datefmt="%H:%M:%S"
        )
        logging.info("Inicializando Robot Control GUI (Orientado a Objetos)...")

    def _apply_styles(self) -> None:
        """Aplica la paleta base y procesa el QSS global."""
        StyleManager.apply_base_palette(self.app)

    def _setup_connections(self) -> None:
        """
        Enlaza los callbacks del backend con las señales de Qt y, a su vez,
        conecta estas señales a los métodos de actualización visual de MainWindow.
        """
        # =====================================================================
        # 1. FLUJO: BACKEND -> FRONTEND (Actualizaciones de la UI)
        # =====================================================================

        # El backend ejecuta el callback (quizás en otro hilo). Esto simplemente
        # emite la señal thread-safe de Qt.
        #self.controller.log_callback = self._on_backend_log_emitted

        # La señal segura se conecta al método visual (corriendo en el hilo principal).
        #self.bridge.log_received.connect(self.window.append_system_log)

        # =====================================================================
        # 2. FLUJO: FRONTEND -> BACKEND (Acciones del usuario)
        # =====================================================================
        # Conectamos la señal que emite el widget de la terminal directamente 
        # al método del controlador que procesará el string.
        # Al ir desde la UI (hilo principal) al backend, es seguro hacer una llamada directa.
        #self.window.terminal_widget.command_submitted.connect(
        #    self.controller.send_manual_command
        #)

    def _on_backend_telemetry_update(self, data: Dict[str, Any]) -> None:
        """
        Callback que el RobotController invocará directamente.
        ATENCIÓN: Este método podría ejecutarse en un hilo secundario.
        NO interactuar con la UI aquí. Solo emitir la señal.
        
        Args:
            data (Dict[str, Any]): Datos empaquetados provenientes del hardware.
        """
        self.bridge.telemetry_received.emit(data)

    def run(self) -> None:
        """
        Despliega la interfaz, arranca el controlador e inicia el bucle de eventos.
        """
        try:
            self.window.show()
            self.controller.start()
        except Exception as e:
            logging.error(f"Fallo crítico al iniciar componentes: {e}")
            sys.exit(1)
        
        sys.exit(self.app.exec())


if __name__ == "__main__":
    # Punto de entrada estricto
    gui_application = RobotGUIApp()
    gui_application.run()