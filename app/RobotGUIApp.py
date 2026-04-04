"""
Módulo de la Aplicación Principal.
Orquesta la inicialización de la interfaz gráfica, el controlador del robot
y establece los puentes de comunicación segura (Thread-Safe) entre ambos.
"""

import sys
import logging
import os
import numpy as np
import time

from typing import Any
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QObject, Signal

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importaciones de la vista y estilos
from app.views.main_window import MainWindow
from app.resources.styles.style_manager import StyleManager
from control.robot_controller import RobotController, SystemState

class SignalBridge(QObject):
    """Puente de señales para garantizar la seguridad entre hilos."""
    # Define la firma de los datos que enviará el controlador.
    # * np.array: Matriz de telemetria con posiciones cartesiana
    # * list: Lista de telemetria con angulos articulares
    # * object: Objeto con el estado del sistema
    # * dict: Objeto con el estado de actuadores
    telemetry_received = Signal(np.ndarray, list, object, dict, int)



class RobotGUIApp:
    """
    Clase principal que orquesta el ciclo de vida de la aplicación.
    Instancia la UI, el backend y gestiona la inyección de dependencias.
    """

    def __init__(self, app: QApplication, controller) -> None:
        """
        Inicializa los componentes principales de la arquitectura.
        """
        self.app = app
        self.controller = controller
        
        # Componentes Visuales y Puente entre senales
        self.window = MainWindow()
        self.bridge = SignalBridge()

        # Variables para Throttling
        self._last_telemetry_time = 0.0
        self._ui_refresh_rate = 1.0 / 20.0  # 20FPS es ideal

        # Memoria de Estados Criticos
        self._last_state_name = ""
        self._last_progress = -1
        
        self._apply_styles()
        self._setup_connections()


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
        # 1. Telemetria.
        self.controller.ui_update_callback = self._on_backend_telemetry_update
        # Conexión directa hacia la actualización central de la MainWindow
        self.bridge.telemetry_received.connect(self.window.update_dashboard)

        # 2. Comandos Manuales.
        self.window.terminal_widget.command_submitted.connect(
            self.controller.send_manual_command
        )

        # Conexiones de cambio de Modo
        panel = self.window.control_panel_widget

        # Conexiones de Modos
        panel.request_mode_idle.connect(self.controller.request_idle_state)
        panel.request_mode_manual.connect(self.controller.request_manual_mode)
        panel.request_mode_auto.connect(self.controller.request_auto_ready)

        # Conexiones de Memoria
        panel.request_start_recording.connect(self.controller.start_recording)
        panel.request_stop_recording.connect(self.controller.stop_recording)
        panel.request_load_trajectory.connect(self.controller.load_trajectory)
        
        # Conexiones de ejecucion de trayectoria
        panel.request_execute_auto.connect(self.controller.execute_auto)
        panel.request_simulate_trajectory.connect(self.controller.simulate_trajectory)
        panel.request_stop_simulation.connect(self.controller.stop_simulation)
        panel.request_pause_trajectory.connect(self.controller.stop_trajectory)
        panel.request_resume_trajectory.connect(self.controller.resume_trajectory)
        panel.request_cancel_trajectory.connect(self.controller.cancel_trajectory)

    def _on_backend_telemetry_update(self, cartesian_state: np.ndarray, joints: list[float], current_state: Any, actuator_state: dict, progress=0) -> None:
        """
        ATENCIÓN: Este método es llamado por el hilo RX del AsyncUARTCommunicator.
        Su única responsabilidad es emitir la señal para cruzar al hilo principal.
        """
        now = time.time()
        
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)
        
        # 1. EVALUACIÓN DE BYPASS CRÍTICO
        # ¿El estado general cambió? ¿O el progreso saltó a 0 (abortado) o a 1 (iniciado)?
        is_critical_change = (
            (state_name != self._last_state_name) or 
            (progress == 0 and self._last_progress > 0) or 
            (progress == 1 and self._last_progress == 0)
        )
        
        # 2. PUERTA LÓGICA
        # Pasa si es urgente (bypass) O si ya pasó el tiempo de refresco (throttling normal)
        if is_critical_change or (now - self._last_telemetry_time) >= self._ui_refresh_rate:
            self.bridge.telemetry_received.emit(cartesian_state, joints, current_state, actuator_state, progress)
            
            # Actualizamos la memoria
            self._last_telemetry_time = now
            self._last_state_name = state_name
            self._last_progress = progress

    def run(self) -> None:
        """
        Despliega la interfaz, arranca el controlador e inicia el bucle de eventos.
        """
        try:
            self.window.show()
            #! Ver si agrego un inicio al controlador del Robot
            if hasattr(self.controller, 'start'):
                self.controller.start()

            elif hasattr(self.controller, 'uart') and not self.controller.uart.serial_conn:
                # Opcional: Podrías mandar llamar connect() aquí si no lo hiciste en main.py
                pass

        except Exception as e:
            logging.error(f"Fallo crítico al iniciar componentes: {e}")
            sys.exit(1)
        
        sys.exit(self.app.exec())

    


if __name__ == "__main__":
    # Punto de entrada estricto
    gui_application = RobotGUIApp()
    gui_application.run()