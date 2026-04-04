"""
Módulo de la Ventana Principal (MainWindow).
Estructura el esqueleto responsivo de la interfaz utilizando Layouts de Qt.
"""

from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QFrame
)
import numpy as np
from typing import Any

from PySide6.QtCore import Qt
from app.core.constants import AppConfig

# Componentes de la UI
from app.views.components.terminal_uart import TerminalUART
from app.views.components.log_viewer import LogViewer
from app.views.components.control_panel import ControlPanel
from app.views.components.digital_twin import DigitalTwin

from control.robot_controller import SystemState


class MainWindow(QMainWindow):
    """
    Ventana principal de la aplicación.
    Implementa el Layout Maestro estructurando las divisiones proporcionales
    del Design Handoff usando Stretch Factors.
    """

    def __init__(self) -> None:
        """Inicializa la ventana principal y su estructura de UI."""
        super().__init__()
        self.setWindowTitle("Robot Control GUI")
        self.setMinimumSize(1024, 768)  # Resolución mínima de seguridad
        self.showMaximized()            # Comienza en pantalla completa
        
        self._setup_ui()

    def _setup_ui(self) -> None:
        """
        Configura los widgets y layouts principales de la ventana.
        No utiliza posiciones ni tamaños absolutos, delegando todo al Layout Manager.
        """
        # --- Contenedor Central ---
        # Widget Maestro
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        central_widget.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")   # Color de fondo

        # Layout Maestro (Row Flex)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. --- Creación de Columnas ---
        # Columna Izquierda (Telemetría)
        # a. Widget
        left_col_widget = QWidget()
        # b. Layout
        left_col_layout = QVBoxLayout(left_col_widget)
        # c. Configuración
        left_col_layout.setContentsMargins(0, 0, 0, 0)
        left_col_layout.setSpacing(0)

        # Columna Derecha (Operación)
        right_col_widget = QWidget()
        right_col_layout = QVBoxLayout(right_col_widget)
        right_col_layout.setContentsMargins(0, 0, 0, 0)
        right_col_layout.setSpacing(0)
        
        # Borde divisor de la columna derecha (border-left 1px)
        right_col_widget.setStyleSheet(
            f"border-left: 1px solid {AppConfig.COLORS.BORDERS};"   # Division a la izquierda de la columna derecha
        )

        # 2. --- Agregamos las columnas al layout maestro ---
        main_layout.addWidget(left_col_widget, stretch=75)  # 75%
        main_layout.addWidget(right_col_widget, stretch=25) # 25%

        # --- Ensamblado Columna Izquierda (75vw) ---
        # Fila Superior: Gemelo Digital (70vh)
        # placeholder_frame es para previsalizar la distribucion. Crea un contenedor generico
        self.digital_twin_widget = DigitalTwin()
        
        # Fila Inferior: Contenedor para Terminal y Logs (30vh)
        # 1. Creo Widgets de contenedor
        bottom_left_widget = QFrame()
        bottom_left_widget.setObjectName("BottomLeftContainer")
        bottom_left_widget.setStyleSheet(
            f"QFrame#BottomLeftContainer {{"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        # 2. Creo Layout de contenedor
        bottom_left_layout = QVBoxLayout(bottom_left_widget)
        bottom_left_layout.setContentsMargins(0, 0, 0, 0)
        bottom_left_layout.setSpacing(0)

        # 1. INTEGRACIÓN: Instanciación de componentes reales
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
        
        # Agrego los widgets del digital twin y del contenedor con los otros 2 widgets a la columna izquierda
        #! SE ANIDAN LOS WIDGETS EN EL ORDEN EN QUE SE AGREGAN
        left_col_layout.addWidget(self.digital_twin_widget, stretch=70)
        left_col_layout.addWidget(bottom_left_widget, stretch=30)

        # --- Ensamblado Columna Derecha (25vw) ---
        # Bloque Superior: Seguridad / E-Stop (15vh)
        self.frame_safety = self._create_placeholder_frame(
            name="SafetyEStop",
            bg_color=AppConfig.COLORS.SURFACE
        )
        
        # Bloque Inferior: Panel de Control (85vh)
        self.control_panel_widget = ControlPanel()

        self.control_panel_widget.setStyleSheet(
            f"ControlPanel {{"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        # Asignar stretch factors para 15vh / 85vh
        right_col_layout.addWidget(self.frame_safety, stretch=15)
        right_col_layout.addWidget(self.control_panel_widget, stretch=85)
    
    def _create_placeholder_frame(self, name: str, bg_color: str) -> QFrame:
        """
        Crea un QFrame de prueba para previsualizar la distribución del Layout.
        
        Args:
            name (str): Nombre del objectName para depuración.
            bg_color (str): Código HEX del color de fondo del componente.
            
        Returns:
            QFrame: Frame configurado con colores para previsualización estructural.
        """
        frame = QFrame()
        frame.setObjectName(name)
        # Aplicamos estilo inline temporal para validar el Layout visualmente.
        # En producción, esto será manejado por el StyleManager y los archivos .qss
        frame.setStyleSheet(
            f"QFrame#{name} {{"
            f"  background-color: {bg_color};"
            f"}}"
        )
        return frame
    
    def update_dashboard(self, cartesian_state: np.ndarray, joints: list[float], current_state: Any, actuator_state: dict) -> None:
        """
        Slot principal (Thread-Safe) que actualiza toda la interfaz gráfica.
        Recibe la telemetría del controlador y la distribuye a los widgets secundarios.
        
        Args:
            cartesian_state (np.ndarray): Coordenadas XYZ de todos las articulaciones.
            joints (list[float]): Angulos de todas las articulaciones.
            current_state (SystemState): Estado lógico de la máquina (IDLE, MANUAL, etc.).
            actuator_state (dict): Estado booleano de la bomba de vacío y válvula.
        """
        
        # 1. Terminal (Bloqueo lógico)
        # Si el sistema está en AUTO_RUNNING, deshabilitamos la inyección manual
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)
        is_safe_for_manual = state_name in ["IDLE", "MANUAL", "AUTO_READY", "RECORDING"]
        self.terminal_widget.set_enabled_state(is_safe_for_manual)

        # 2. Gemelo Digital
        if hasattr(self, 'digital_twin_widget'):
            # Enviamos el paquete de datos completo al widget especializado
            self.digital_twin_widget.process_telemetry(cartesian_state, joints, state_name, actuator_state)

        
        # 3. Enrutamiento al Panel de Control (LEDs de estado, botones, actuadores)
        # if hasattr(self, 'control_panel_widget'):
        #     self.control_panel_widget.update_machine_state(current_state)
        #     self.control_panel_widget.update_actuators(actuator_state)

        # 4. Enrutamiento al Panel de Seguridad (Indicador general de E-STOP o FAULT)
        # if hasattr(self, 'safety_panel_widget'):
        #     self.safety_panel_widget.update_safety_status(current_state)