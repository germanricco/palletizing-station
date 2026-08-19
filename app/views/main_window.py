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
        # Contenedor central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        central_widget.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")   # Color de fondo

        # Layout Maestro (Row Flex)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # --- COLUMNA IZQUIERDA ---
        left_col_widget = QWidget()
        left_col_layout = QVBoxLayout(left_col_widget)
        left_col_layout.setContentsMargins(0, 0, 0, 0)
        left_col_layout.setSpacing(0)

        # Fila Superior: Gemelo Digital y Barra de Progreso
        self.digital_twin_container = QWidget()
        dt_layout = QVBoxLayout(self.digital_twin_container)
        dt_layout.setContentsMargins(0, 0, 0, 0)
        dt_layout.setSpacing(0)

        self.digital_twin_widget = DigitalTwin()

        # Barra de progreso
        self.progress_bar = QProgressBar()
        self.progress_bar.setFixedHeight(4)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setStyleSheet(f"""
            QProgressBar {{ background-color: #1A1A1A; border: none; }}
            QProgressBar::chunk {{ background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; }}
        """)
        self.progress_bar.setValue(0)
        self.progress_bar.hide()
        self._last_progress_color = None

        dt_layout.addWidget(self.digital_twin_widget, stretch=1)
        dt_layout.addWidget(self.progress_bar)
        
        # Fila Inferior: Contenedor para Terminal y Logs (30vh)
        bottom_left_widget = QFrame()
        bottom_left_widget.setObjectName("BottomLeftContainer")
        bottom_left_widget.setStyleSheet(f"QFrame#BottomLeftContainer {{ border-top: 1px solid {AppConfig.COLORS.BORDERS}; }}")

        bottom_left_layout = QVBoxLayout(bottom_left_widget)
        bottom_left_layout.setContentsMargins(0, 0, 0, 0)
        bottom_left_layout.setSpacing(0)

        # Instanciación de componentes
        self.logs_widget = LogViewer()
        self.terminal_widget = TerminalUART()
        self.terminal_widget.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.terminal_widget.setStyleSheet(f"TerminalUART {{ border-top: 1px solid {AppConfig.COLORS.BORDERS}; }}")

        # Agrego Widgets desde /components a contenedor
        bottom_left_layout.addWidget(self.logs_widget, stretch=80)
        bottom_left_layout.addWidget(self.terminal_widget, stretch=20)
        
        # Insercion final en la columna izquierda
        left_col_layout.addWidget(self.digital_twin_container, stretch=70)
        left_col_layout.addWidget(bottom_left_widget, stretch=30)

        # --- COLUMNA DERECHA (25vw) ---
        right_col_widget = QFrame()
        right_col_widget.setObjectName("RightColumn")
        
        right_col_layout = QVBoxLayout(right_col_widget)
        right_col_layout.setContentsMargins(1, 0, 0, 0)
        right_col_layout.setSpacing(0)

        right_col_widget.setStyleSheet(f"QFrame#RightColumn {{ border-left: 1px solid {AppConfig.COLORS.BORDERS}; }}")

        self.safety_panel_widget = SafetyPanel()
        self.control_panel_widget = ControlPanel()
        self.control_panel_widget.setStyleSheet(f"ControlPanel {{ background-color: {AppConfig.COLORS.BACKGROUND}; border-top: 1px solid {AppConfig.COLORS.BORDERS}; }}")

        # Asignar stretch factors para 15vh / 85vh
        right_col_layout.addWidget(self.safety_panel_widget, stretch=15)
        right_col_layout.addWidget(self.control_panel_widget, stretch=85)
    
        # Ensamble final
        main_layout.addWidget(left_col_widget, stretch=75)
        main_layout.addWidget(right_col_widget, stretch=25)


        
    
    # ==== ENRUTAMIENTO DE DATOS ==== 
    def update_dashboard(self, cartesian_state: np.ndarray, joints: list[float], current_state: Any, actuator_active: bool, progress: int) -> None:
        """
        Slot principal (Thread-Safe) que actualiza toda la interfaz gráfica.
        Recibe la telemetría del controlador y la distribuye a los widgets secundarios.
        
        Args:
            cartesian_state (np.ndarray): Coordenadas XYZ de todos las articulaciones.
            joints (list[float]): Angulos de todas las articulaciones.
            current_state (SystemState): Estado lógico de la máquina (IDLE, MANUAL, etc.).
            actuator_active (bool): Estado booleano del actuador.
            progress (int): progreso de la trayectoria
        """
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)
        
        # Detectamos sub-estado de simulacion
        is_simulating = (state_name == "AUTO_READY" and progress > 0)

        # 1. GESTION DE TERMINAL Y SEGURIDAD MANUAL
        is_safe_for_manual = state_name in ["IDLE", "MANUAL", "AUTO_READY", "RECORDING"] and not is_simulating
        self.terminal_widget.set_enabled_state(is_safe_for_manual)
        
        # 2. GESTION DE BARRA DE PROGRESO
        is_active = state_name in ["AUTO_RUNNING", "RECORDING"] or is_simulating
        
        if is_active:
            self.progress_bar.show()
            self.progress_bar.setValue(progress)

            # Cambiar color si es ejecución real vs simulación (solo si realmente cambio: evita
            # reaplicar el mismo QSS en cada tick de telemetria mientras la barra esta activa).
            color = "#34C759" if state_name == "AUTO_RUNNING" else "#00F0FF"
            if color != self._last_progress_color:
                self._last_progress_color = color
                self.progress_bar.setStyleSheet(f"""
                    QProgressBar {{ background-color: #1A1A1A; border: none; }}
                    QProgressBar::chunk {{ background-color: {color}; }}
                """)
        else:
            self.progress_bar.hide()
            self.progress_bar.setValue(0)

        # 3. ENRUTAMIENTO AL GEMELO DIGITAL
        if hasattr(self, 'digital_twin_widget'):
            self.digital_twin_widget.process_telemetry(cartesian_state, joints, state_name, actuator_active)

        # 4. ENRUTAMIENTO AL PANEL DE CONTROL
        if hasattr(self, 'control_panel_widget'):
            # Detectar si estamos en modo receta basado en la UI
            current_label_text = self.control_panel_widget.lbl_loaded_file.text()
            is_recipe = current_label_text.startswith("RECETA")
            
            # Delegamos la lógica de botones
            self.control_panel_widget.update_machine_state(current_state, progress, is_recipe_mode=is_recipe)
            self.control_panel_widget.update_actuator_state(actuator_active)

            # Actualizamos los visores de teleoperación
            if cartesian_state is not None and cartesian_state.shape[0] > 0:
                tcp_coords = cartesian_state[-1].tolist()
                self.control_panel_widget.update_telemetry(joints, tcp_coords)
        
        # 5. ENRUTAMIENTO AL PANEL DE SEGURIDAD
        if hasattr(self, 'safety_panel_widget'):
            self.safety_panel_widget.update_safety_status(state_name)