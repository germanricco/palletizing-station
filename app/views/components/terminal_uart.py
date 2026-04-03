"""
Módulo del componente Terminal UART.
Proporciona la interfaz de línea de comandos (CLI) para enviar instrucciones manuales.
"""

from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QLineEdit,
    QPushButton
)
from PySide6.QtCore import Signal, Qt
from core.constants import AppConfig


class TerminalUART(QWidget):
    """
    Widget que representa la consola de entrada para comandos seriales.
    
    Attributes:
        command_submitted (Signal): Señal emitida cuando el usuario envía un comando.
                                    Transporta el string del comando.
    """
    
    command_submitted = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el widget de la terminal UART."""
        super().__init__(parent)
        self.setObjectName("TerminalUARTWidget")
        self._setup_ui()

    def _setup_ui(self) -> None:
        """Configura el layout y los sub-componentes visuales."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)

        # Campo de entrada (Input Field) - 80% del ancho
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("Ingrese comando manual...")
        self.input_field.setFixedHeight(48)
        self.input_field.setStyleSheet(
            f"background-color: #0A0A0A;"
            f"border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"padding-left: 16px;"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: {AppConfig.TYPOGRAPHY.SIZE_HUD_CONSOLE};"
        )
        self.input_field.returnPressed.connect(self._on_send_clicked)

        # Botón de envío (Send Button) - 20% del ancho
        self.btn_send = QPushButton("SEND")
        self.btn_send.setFixedHeight(48)
        self.btn_send.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_send.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: {AppConfig.COLORS.BORDERS};"
            f"  color: {AppConfig.COLORS.TEXT_HIGH};"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"  font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_SEMIBOLD};"
            f"  border-radius: 4px;"
            f"}}"
            f"QPushButton:hover {{"
            f"  background-color: #444444;"
            f"}}"
        )
        self.btn_send.clicked.connect(self._on_send_clicked)

        # Agregamos al layout con los stretch factors correspondientes (80 / 20)
        layout.addWidget(self.input_field, stretch=80)
        layout.addWidget(self.btn_send, stretch=20)

    def _on_send_clicked(self) -> None:
        """
        Slot interno ejecutado al presionar Enter o hacer clic en el botón Send.
        Emite la señal `command_submitted` y limpia el campo de texto.
        """
        command = self.input_field.text().strip()
        if command:
            self.command_submitted.emit(command)
            self.input_field.clear()

    def set_enabled_state(self, is_enabled: bool) -> None:
        """
        Habilita o deshabilita la entrada de comandos (Ej: para Modo Automático).
        
        Args:
            is_enabled (bool): Estado de habilitación deseado.
        """
        self.input_field.setEnabled(is_enabled)
        self.btn_send.setEnabled(is_enabled)
        opacity = "1.0" if is_enabled else "0.5"
        self.setStyleSheet(f"opacity: {opacity};")