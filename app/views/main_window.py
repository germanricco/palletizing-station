"""
Módulo de la Ventana Principal (MainWindow).
Estructura el esqueleto responsivo de la interfaz utilizando Layouts de Qt.
Actua como enrutador de datos entre el Orquestador y los componentes visuales.
"""

import numpy as np
from typing import Any

from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QFrame,
    QProgressBar
)
from PySide6.QtCore import Qt
from app.core.constants import AppConfig

# Componentes de la UI
from app.views.components.safety_panel import SafetyPanel
from app.views.components.control_panel import ControlPanel
from app.views.components.terminal_uart import TerminalUART
from app.views.components.log_viewer import LogViewer
from app.views.components.digital_twin import DigitalTwin


from control.robot_controller import SystemState


class MainWindow(QMainWindow):
    """
    Ventana principal de la aplicación.
    Implementa el Layout Maestro estructurando las divisiones proporcionales.
    """
    # ==== INICIALIZACION ====
    def __init__(self) -> None:
        """Inicializa la ventana principal y su estructura de UI."""
        super().__init__()
        self.setWindowTitle("Robot Control GUI")
        self.setMinimumSize(1024, 768)  # Resolución mínima de seguridad
        self.showMaximized()            # Comienza en pantalla completa
        
        self._setup_ui()


    # ==== ENSAMBLAJE DE LA INTERFAZ (LAYOUTS) ====
    def _setup_ui(self) -> None:
        """
        Configura los widgets y layouts principales de la ventana.
        No utiliza posiciones ni tamaños absolutos, delegando todo al Layout Manager.
        """
        # CONTENEDOR CENTRAL Y LAYOUT MAESTRO
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        central_widget.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")   # Color de fondo

        # Layout Maestro (Row Flex)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # CREACIÓN DE COLUMNAS PRINCIPALES
        # Columna Izquierda (Telemetría)
        left_col_widget = QWidget()
        left_col_layout = QVBoxLayout(left_col_widget)
        left_col_layout.setContentsMargins(0, 0, 0, 0)
        left_col_layout.setSpacing(0)

        # Columna Derecha (Operación)
        right_col_widget = QFrame()
        right_col_widget.setObjectName("RightColumn")
        
        right_col_layout = QVBoxLayout(right_col_widget)
        right_col_layout.setContentsMargins(1, 0, 0, 0)
        right_col_layout.setSpacing(0)
        
        # Borde divisor de la columna derecha
        right_col_widget.setStyleSheet(
            f"QWidget#RightColumn {{"
            f"  border-left: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        # Agregamos las columnas al layout maestro
        main_layout.addWidget(left_col_widget, stretch=75)
        main_layout.addWidget(right_col_widget, stretch=25)

        # ENSAMBLADO: COLUMNA IZQUIERDA (75vw)
        # Fila Superior: Gemelo Digital y Barra de Progreso
        self.digital_twin_container = QWidget()
        dt_layout = QVBoxLayout(self.digital_twin_container)
        dt_layout.setContentsMargins(0, 0, 0, 0)
        dt_layout.setSpacing(0)

        self.digital_twin_widget = DigitalTwin()

        # Barra de progreso estandarizada (oculta por defecto)
        self.progress_bar = QProgressBar()
        self.progress_bar.setFixedHeight(4)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setStyleSheet(f"""
            QProgressBar {{
                background-color: #1A1A1A;
                border: none;
            }}
            QProgressBar::chunk {{
                background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; 
            }}
        """)
        self.progress_bar.setValue(0)
        self.progress_bar.hide()

        dt_layout.addWidget(self.digital_twin_widget, stretch=1)
        dt_layout.addWidget(self.progress_bar)
        
        # Fila Inferior: Contenedor para Terminal y Logs (30vh)
        bottom_left_widget = QFrame()
        bottom_left_widget.setObjectName("BottomLeftContainer")
        bottom_left_widget.setStyleSheet(
            f"QFrame#BottomLeftContainer {{"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        bottom_left_layout = QVBoxLayout(bottom_left_widget)
        bottom_left_layout.setContentsMargins(0, 0, 0, 0)
        bottom_left_layout.setSpacing(0)

        # Instanciación de componentes reales
        self.logs_widget = LogViewer()
        self.terminal_widget = TerminalUART()

        # Aplicar atributo para soportar QSS en widget personalizado
        self.terminal_widget.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.terminal_widget.setStyleSheet(
            f"TerminalUART {{"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        # Agrego Widgets de logs y terminal desde /components a contenedor
        bottom_left_layout.addWidget(self.logs_widget, stretch=80)
        bottom_left_layout.addWidget(self.terminal_widget, stretch=20)
        
        # Insercion final en la columna izquierda
        left_col_layout.addWidget(self.digital_twin_container, stretch=70)
        left_col_layout.addWidget(bottom_left_widget, stretch=30)

        # ENSAMBLADO: COLUMNA DERECHA (25vw)
        # Bloque Superior: Seguridad / E-Stop (15vh)
        self.safety_panel_widget = SafetyPanel()
        
        # Bloque Inferior: Panel de Control (85vh)
        self.control_panel_widget = ControlPanel()

        self.control_panel_widget.setStyleSheet(
            f"ControlPanel {{"
            f"  background-color: {AppConfig.COLORS.BACKGROUND};"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        # Asignar stretch factors para 15vh / 85vh
        right_col_layout.addWidget(self.safety_panel_widget, stretch=15)
        right_col_layout.addWidget(self.control_panel_widget, stretch=85)
    
    
    # ==== ENRUTAMIENTO DE DATOS ==== 
    def update_dashboard(self, cartesian_state: np.ndarray, joints: list[float], current_state: Any, actuator_state: dict, progress: int) -> None:
        """
        Slot principal (Thread-Safe) que actualiza toda la interfaz gráfica.
        Recibe la telemetría del controlador y la distribuye a los widgets secundarios.
        
        Args:
            cartesian_state (np.ndarray): Coordenadas XYZ de todos las articulaciones.
            joints (list[float]): Angulos de todas las articulaciones.
            current_state (SystemState): Estado lógico de la máquina (IDLE, MANUAL, etc.).
            actuator_state (dict): Estado booleano de la bomba de vacío y válvula.
            progress (int): progreso de la trayectoria
        """
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)
        
        #! Detectamos sub-estado de simulacion
        is_simulating = (state_name == "AUTO_READY" and progress > 0)

        # GESTION DE TERMINAL Y SEGURIDAD MANUAL
        is_safe_for_manual = state_name in ["IDLE", "MANUAL", "AUTO_READY", "RECORDING"]
        
        #!
        if is_simulating:
            is_safe_for_manual = False
            
        self.terminal_widget.set_enabled_state(is_safe_for_manual)
        
        # GESTION DE BARRA DE PROGRESO
        is_active = state_name in ["AUTO_RUNNING", "RECORDING"] or is_simulating
        
        if is_active:
            self.progress_bar.show()
            self.progress_bar.setValue(progress)
            # Cambiar color si es ejecución real vs simulación
            color = "#34C759" if state_name == "AUTO_RUNNING" else "#00F0FF"
            self.progress_bar.setStyleSheet(self.progress_bar.styleSheet().replace("#00F0FF", color))
        else:
            self.progress_bar.hide()
            self.progress_bar.setValue(0)


        # ENRUTAMIENTO AL GEMELO DIGITAL
        if hasattr(self, 'digital_twin_widget'):
            self.digital_twin_widget.process_telemetry(cartesian_state, joints, state_name, actuator_state)

        
        # ENRUTAMIENTO Y SOBREESCRITURA DEL PANEL DE CONTROL
        if hasattr(self, 'control_panel_widget'):
            # Llamada estándar para actualizar Segmented Controls y botones base
            self.control_panel_widget.update_machine_state(current_state)
            
            # Enrutamiento de la telemetría dual al panel
            if getattr(cartesian_state, 'shape', (0,))[0] >= 6:
                tcp_coords = cartesian_state[5].tolist()
                self.control_panel_widget.update_telemetry(joints, tcp_coords)
            # --- Inyección de Lógica Visual para la Simulación ---
            # Como la interfaz de simulación vive en el panel, pero el panel 
            # no conoce la variable "progress", lo ajustamos desde aquí:
            if is_simulating:
                # Muestra el botón [DETENER SIM.]
                self.control_panel_widget.stack_abort.setCurrentIndex(1)
                # Bloquea el botón [EJECUTAR] para evitar choques de comandos
                self.control_panel_widget._set_button_enabled(self.control_panel_widget.btn_execute, False)
            
            elif state_name == "AUTO_READY":
                # Si estamos listos, pero NO simulando, restauramos el stack a [SIMULAR]
                self.control_panel_widget.stack_abort.setCurrentIndex(0)
                # Habilitamos [EJECUTAR] ya que hay una trayectoria validada
                self.control_panel_widget._set_button_enabled(self.control_panel_widget.btn_execute, True)
        

        # ENRUTAMIENTO AL PANEL DE SEGURIDAD
        if hasattr(self, 'safety_panel_widget'):
            self.safety_panel_widget.update_safety_status(state_name)