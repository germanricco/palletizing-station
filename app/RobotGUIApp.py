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
from PySide6.QtWidgets import QApplication, QDialog
from PySide6.QtCore import QObject, Signal

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importaciones de la vista y estilos
from app.views.main_window import MainWindow
from app.resources.styles.style_manager import StyleManager
from app.views.components.save_dialog import SaveTrajectoryDialog
from control.robot_controller import RobotController, SystemState

class SignalBridge(QObject):
    """
    Puente de señales para garantizar la seguridad entre hilos.
    """
    telemetry_received = Signal(object, list, object, bool, int)    # telemetria
    prompt_save_requested = Signal(str)  # solicitar nombre de guardado
    trajectory_loaded = Signal(dict)     # trayectoria/receta cargada (payload con tipo y progreso)
    conveyor_state_changed = Signal(str) # estado de la cinta transportadora (EVT:CINTA)


class RobotGUIApp:
    """
    Clase principal que orquesta el ciclo de vida de la aplicación.
    """

    # ==== INICIALIZACION Y CONFIGURACION ====
    def __init__(self, app: QApplication, controller) -> None:
        """
        Inicializa los componentes principales de la arquitectura.
        """
        # Aplicacion y Controlador
        self.app = app
        self.controller = controller
        
        # Componentes Visuales y Puente entre senales
        self.window = MainWindow()
        self.bridge = SignalBridge()

        # Variables para Throttling (bajar fps)
        self._last_telemetry_time = 0.0
        self._ui_refresh_rate = 1.0 / 20.0  # 20FPS

        # Contrapresion real: mientras haya una actualizacion ya encolada/en ejecucion en el
        # hilo de la GUI, no encolamos otra de telemetria rutinaria. Sin esto, un frame lento
        # ocasional (GC, hiccup del SO, etc.) hace crecer la cola de eventos Qt sin limite,
        # porque la telemetria de 50Hz nunca deja de llegar mientras haya conexion — el atraso
        # queda permanentemente pegado y se nota como lag general (hasta en el hover de botones).
        self._gui_update_pending = False

        # Memoria de Estados Criticos
        self._last_state_name = ""
        self._last_progress = 0
        self._last_actuator_state = False
        
        self._apply_styles()
        self._setup_connections()

    def _setup_logging(self) -> None:
        """
        Configura el formato estándar para los logs de la aplicación.
        """
        logging.basicConfig(
            level=logging.INFO,
            format="[%(asctime)s] | [%(levelname)s] | [%(module)s] | %(message)s",
            datefmt="%H:%M:%S"
        )
        logging.info("Inicializando Robot Control GUI (Orientado a Objetos)...")

    def _apply_styles(self) -> None:
        """
        Aplica la paleta base y procesa el QSS global.
        """
        StyleManager.apply_base_palette(self.app)

    
    # ==== ENRUTAMIENTO DE SENALES (WIRING) ====
    def _setup_connections(self) -> None:
        """
        Enlaza los callbacks del backend con las señales de Qt y, a su vez,
        conecta estas señales a los métodos de actualización visual de MainWindow.
        """
        # --- Flujos de Alta Frecuencia (Backend -> UI) ---
        # Telemetria y Cinematica
        self.controller.ui_update_callback = self._on_backend_telemetry_update
        self.bridge.telemetry_received.connect(self._on_dashboard_update)

        # Eventos Discretos de Memoria
        self.controller.trajectory_loaded_callback = self._on_trajectory_loaded
        self.bridge.trajectory_loaded.connect(self.window.control_panel_widget.update_loaded_file_name)

        self.controller.prompt_save_callback = self._on_prompt_save_requested
        self.bridge.prompt_save_requested.connect(self._show_save_dialog)

        # Estado de la Cinta Transportadora
        self.controller.conveyor_update_callback = self._on_conveyor_update
        self.bridge.conveyor_state_changed.connect(self.window.control_panel_widget.update_conveyor_state)

        # --- Eventos de Usuario (UI -> Backend)
        # A. Terminal UART
        self.window.terminal_widget.command_submitted.connect(self.controller.send_manual_command)

        # B. Panel de Control
        panel = self.window.control_panel_widget

        # Comunicaciones
        panel.request_connection.connect(self._handle_connection_request)
        
        # Modos Logicos
        panel.request_mode_idle.connect(self.controller.request_idle_state)
        panel.request_mode_manual.connect(self.controller.request_manual_mode)
        panel.request_mode_auto.connect(self.controller.request_auto_ready)
        
        # Jogging y Cinta Transportadora
        panel.request_jog.connect(self.controller.execute_jog)
        panel.request_toggle_conveyor.connect(self.controller.toggle_conveyor)
        
        # Memoria y Palletizado
        panel.request_start_recording.connect(self.controller.start_recording)
        panel.request_stop_recording.connect(self.controller.stop_recording)
        
        panel.request_load_trajectory.connect(self.controller.load_single_trajectory)
        panel.request_load_recipe.connect(self.controller.load_recipe)
        panel.request_start_palletizing.connect(self.controller.start_palletizing)
        panel.request_stop_palletizing.connect(self.controller.stop_palletizing)
        panel.request_unload_memory.connect(self.controller.unload_recipe)
        
        # Ejecucion de Trayectoria (la ejecucion real la dispara el sensor dentro del ciclo)
        panel.request_simulate_trajectory.connect(self.controller.simulate_trajectory)
        panel.request_stop_simulation.connect(self.controller.stop_simulation)
        panel.request_pause_trajectory.connect(self.controller.stop_trajectory)
        panel.request_resume_trajectory.connect(self.controller.resume_trajectory)
        panel.request_cancel_trajectory.connect(self.controller.cancel_trajectory)

        # C. Panel de Seguridad
        safety = self.window.safety_panel_widget
        safety.request_e_stop.connect(self.controller.trigger_emergency_stop)

        # Reinicio (salida controlada de FAULT/EMERGENCY, ver RobotController.reset_fault)
        safety.request_e_reset.connect(self.controller.reset_fault)


    # ==== METODOS PUENTE (THREAD-SAFE WRAPPERS) ====
    def _on_backend_telemetry_update(self, cartesian_state: np.ndarray, joints: list[float], current_state: Any, actuator_active: bool, progress=0) -> None:
        """
        Invocado por el hilo RX del AsyncUARTCommunicator.
        Cruza la barrera de hilos emitiendo la señal Qt.
        """
        now = time.time()
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)

        # Evaluacion de Bypass: los cambios criticos (transicion de estado, actuador, inicio/fin
        # de progreso) siempre se emiten de inmediato, nunca se descartan por contrapresion.
        is_critical_change = (
            (state_name != self._last_state_name) or
            (actuator_active != self._last_actuator_state) or
            (progress == 0 and self._last_progress > 0) or
            (progress > 0 and self._last_progress == 0)
        )

        if not is_critical_change:
            # Telemetria rutinaria. Contrapresion: si la GUI todavia no termino de procesar la
            # ultima actualizacion encolada, este frame se descarta (no se acumula) en vez de
            # apilarse en la cola de eventos de Qt.
            if self._gui_update_pending:
                return
            if (now - self._last_telemetry_time) < self._ui_refresh_rate:
                return

        self._gui_update_pending = True
        self.bridge.telemetry_received.emit(cartesian_state, joints, current_state, actuator_active, progress)

        # Actualizamos la memoria interna
        self._last_telemetry_time = now
        self._last_state_name = state_name
        self._last_progress = progress
        self._last_actuator_state = actuator_active

    def _on_dashboard_update(self, cartesian_state: np.ndarray, joints: list[float], current_state: Any, actuator_active: bool, progress: int) -> None:
        """
        Slot real (hilo de la GUI) conectado a la señal de telemetría. Ejecuta la actualización
        visual y recién ahí libera la contrapresión, para que el próximo frame pueda encolarse.
        """
        try:
            self.window.update_dashboard(cartesian_state, joints, current_state, actuator_active, progress)
        finally:
            self._gui_update_pending = False

    def _on_trajectory_loaded(self, payload: dict) -> None:
        """
        Puente seguro: Recibe del backend y emite la señal hacia el hilo principal de la GUI.
        """
        self.bridge.trajectory_loaded.emit(payload)

    def _on_conveyor_update(self, state: str) -> None:
        """Puente seguro: estado de la cinta desde el hilo RX hacia el hilo de la GUI."""
        self.bridge.conveyor_state_changed.emit(state)

    def _on_prompt_save_requested(self, default_name: str) -> None:
        """Puente para saltar al hilo principal de Qt."""
        self.bridge.prompt_save_requested.emit(default_name)


    # ==== GESTION DE EVENTOS COMPLEJOS Y CICLO ====
    def _handle_connection_request(self, request_connect: bool) -> None:
        """
        Actúa como intermediario entre la acción del botón en la UI 
        y el hardware serial en el backend.
        """
        if request_connect:
            # Pide conexión al backend
            self.controller.start_connection(max_retries=3, delay_seconds=1.0)
        else:
            # Si no conecta, finaliza la comunicación
            self.controller.finish_connection()
        self.window.control_panel_widget.set_connection_state(self.controller.is_connected)

    def _show_save_dialog(self, default_name: str) -> None:
        """Abre la ventana modal y procesa la respuesta del usuario."""
        # Se le pasa self.window como parent para que quede centrada
        dialog = SaveTrajectoryDialog(default_filename=default_name, parent=self.window)
        
        if dialog.exec() == QDialog.DialogCode.Accepted:
            # El usuario hizo clic en GUARDAR
            self.controller.confirm_save_trajectory(dialog.get_filename(), save=True)
        else:
            # El usuario hizo clic en CANCELAR o cerró la ventana
            self.controller.confirm_save_trajectory("", save=False)

    def run(self) -> None:
        """
        Despliega la interfaz, arranca el controlador e inicia el bucle de eventos.
        """
        try:
            self.window.show()
            if hasattr(self.controller, 'start'):
                self.controller.start()

        except Exception as e:
            logging.error(f"Fallo crítico al iniciar componentes: {e}")
            sys.exit(1)
        
        sys.exit(self.app.exec())

