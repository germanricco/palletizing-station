"""
Módulo del componente Panel de Control (Control Panel).
Contiene la estructura base para la interfaz de operación, dividida en
comunicaciones, modos lógicos, jogging de GDL y gestión de trayectorias.
"""

from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QFrame,
    QLabel
)
from PySide6.QtCore import Qt
from app.core.constants import AppConfig


class ControlPanel(QWidget):
    """
    Widget principal del Panel de Control.
    Estructura dinámicamente las zonas de interacción del operador
    utilizando proporciones (stretch factors).
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el Panel de Control y su layout estructural."""
        super().__init__(parent)
        self.setObjectName("ControlPanelWidget")
        
        # Habilitar el renderizado de estilos QSS en el QWidget base
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")
        
        self._setup_ui()

    def _setup_ui(self) -> None:
        """
        Configura el administrador de diseño vertical y define
        los espacios proporcionales para los sub-módulos.
        """
        layout = QVBoxLayout(self)
        # Aplicamos padding interno al panel y separación entre secciones
        layout.setContentsMargins(
            int(AppConfig.SPACING.SM.replace("px", "")),
            int(AppConfig.SPACING.SM.replace("px", "")),
            int(AppConfig.SPACING.SM.replace("px", "")),
            int(AppConfig.SPACING.SM.replace("px", ""))
        )
        layout.setSpacing(int(AppConfig.SPACING.MD.replace("px", "")))

        # 1. Cabecera Sistema y Comunicaciones (~10%)
        self.frame_comms = self._create_section_placeholder("1. Comunicaciones & Estado")
        
        # 2. Selección de Modos Lógicos (~15%)
        self.frame_modes = self._create_section_placeholder("2. Modos Lógicos (Segmented)")
        
        # 3. Tarjetas de Teleoperación / Jogging 3 GDL (~50%)
        self.frame_jogging = self._create_section_placeholder("3. Teleoperación (GDL 1, 2, 3)")
        
        # 4. Gestión de Trayectorias y Memoria (~25%)
        self.frame_memory = self._create_section_placeholder("4. Trayectorias y Ejecución")

        # Inserción en el Layout Padre con los Stretch Factors del Design Handoff
        layout.addWidget(self.frame_comms, stretch=10)
        layout.addWidget(self.frame_modes, stretch=15)
        layout.addWidget(self.frame_jogging, stretch=50)
        layout.addWidget(self.frame_memory, stretch=25)

    def _create_section_placeholder(self, title: str) -> QFrame:
        """
        Crea un bloque temporal para visualizar la distribución espacial
        antes de implementar los controles definitivos.
        
        Args:
            title (str): Etiqueta descriptiva del placeholder.
            
        Returns:
            QFrame: Contenedor con estilos de depuración.
        """
        frame = QFrame()
        frame.setStyleSheet(
            f"QFrame {{"
            f"  background-color: {AppConfig.COLORS.SURFACE};"
            f"  border: 1px dashed {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 4px;"
            f"}}"
        )
        
        # Layout interno temporal para centrar el título
        layout = QVBoxLayout(frame)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        label = QLabel(title)
        label.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 14px;"
            f"border: none;"
        )
        layout.addWidget(label)
        
        return frame