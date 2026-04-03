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
from PySide6.QtCore import Qt
from core.constants import AppConfig
from views.components.terminal_uart import TerminalUART
from views.components.log_viewer import LogViewer


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
        self.showMaximized()
        
        self._setup_ui()

    def _setup_ui(self) -> None:
        """
        Configura los widgets y layouts principales de la ventana.
        No utiliza posiciones ni tamaños absolutos, delegando todo al Layout Manager.
        """
        # --- Contenedor Central ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        central_widget.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")

        # Layout Maestro (Row Flex)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # --- Creación de Columnas ---
        # Columna Izquierda (Telemetría)
        left_col_widget = QWidget()
        left_col_layout = QVBoxLayout(left_col_widget)
        left_col_layout.setContentsMargins(0, 0, 0, 0)
        left_col_layout.setSpacing(0)

        # Columna Derecha (Operación)
        right_col_widget = QWidget()
        right_col_layout = QVBoxLayout(right_col_widget)
        right_col_layout.setContentsMargins(0, 0, 0, 0)
        right_col_layout.setSpacing(0)
        
        # Borde divisor de la columna derecha (border-left 1px)
        right_col_widget.setStyleSheet(
            f"border-left: 1px solid {AppConfig.COLORS.BORDERS};"
        )

        # Agregamos las columnas al layout maestro respetando el 75vw / 25vw
        main_layout.addWidget(left_col_widget, stretch=75)
        main_layout.addWidget(right_col_widget, stretch=25)

        # --- Ensamblado Columna Izquierda (75vw) ---
        # Fila Superior: Gemelo Digital (70vh)
        self.frame_digital_twin = self._create_placeholder_frame(
            name="DigitalTwin",
            bg_color=AppConfig.COLORS.BACKGROUND
        )
        
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

        # 1. INTEGRACIÓN: Instanciación de componentes reales
        self.logs_widget = LogViewer()
        self.terminal_widget = TerminalUART()

        #Aplicar atributo para soportar QSS en widget personalizado
        self.terminal_widget.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.terminal_widget.setStyleSheet(
            f"TerminalUART {{"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        bottom_left_layout.addWidget(self.logs_widget, stretch=80)
        bottom_left_layout.addWidget(self.terminal_widget, stretch=20)
        
        left_col_layout.addWidget(self.frame_digital_twin, stretch=70)
        left_col_layout.addWidget(bottom_left_widget, stretch=30)
        
        # --- Ensamblado Columna Derecha (25vw) ---
        # Bloque Superior: Seguridad / E-Stop (15vh)
        self.frame_safety = self._create_placeholder_frame(
            name="SafetyEStop",
            bg_color=AppConfig.COLORS.SURFACE
        )
        
        # Bloque Inferior: Panel de Control (85vh)
        self.frame_controls = self._create_placeholder_frame(
            name="ControlPanel",
            bg_color=AppConfig.COLORS.BACKGROUND
        )
        # Borde divisor interno del panel de controles (border-top 1px)
        self.frame_controls.setStyleSheet(
            f"QFrame#ControlPanel {{"
            f"  border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  background-color: {AppConfig.COLORS.BACKGROUND};"
            f"}}"
        )

        # Asignar stretch factors para 15vh / 85vh
        right_col_layout.addWidget(self.frame_safety, stretch=15)
        right_col_layout.addWidget(self.frame_controls, stretch=85)
    

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