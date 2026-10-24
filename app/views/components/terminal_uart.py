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
from PySide6.QtCore import Signal, Qt, QEvent
from app.core.constants import AppConfig


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
        
        # Buffer para historial de comandos
        self.history = []
        self.history_index = -1

        # Rastreador de estado interno
        self._current_enable_state = True

        self._setup_ui()

    def _setup_ui(self) -> None:
        """Configura el layout y los sub-componentes visuales."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)

        # Campo de entrada (Input Field) - 80% del ancho
        self.input_field = QLineEdit()  # Entrada de texto
        self.input_field.setPlaceholderText("Ingrese comando manual...")
        self.input_field.setFixedHeight(48)
        self.input_field.setStyleSheet(
            f"QLineEdit {{"
            f"  background-color: #0A0A0A;"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  color: {AppConfig.COLORS.TEXT_HIGH};"
            f"  padding-left: 16px;"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"  font-size: {AppConfig.TYPOGRAPHY.SIZE_HUD_CONSOLE};"
            f"}}"
            f"QLineEdit:disabled {{ color: #555555; border-color: #2A2A2A; }}"
        )
        
        # Instalamos un filtro de eventos para capturar flechas direccionales
        self.input_field.installEventFilter(self)
        # Se activa al pulsar enter. Conectamos la senal al metodo / slot
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
            f"QPushButton:disabled {{ background-color: #1A1A1A; color: #555555; }}"
        )

        # Se activa al pulsarse. Conectamos la senal al metodo / slot
        self.btn_send.clicked.connect(self._on_send_clicked)

        # Agregamos al layout con los stretch factors  (80 / 20)
        layout.addWidget(self.input_field, stretch=80)
        layout.addWidget(self.btn_send, stretch=20)

    
    def eventFilter(self, obj, event) -> bool:
        """
        Filtro para interceptar teclas antes de que QLineEdit las procese.
        """
        if obj == self.input_field and event.type() == QEvent.Type.KeyPress:
            if event.key() == Qt.Key.Key_Up:
                self._navigate_history(-1)
                return True
            elif event.key() == Qt.Key.Key_Down:
                self._navigate_history(1)
                return True
        return super().eventFilter(obj, event)
    

    def _navigate_history(self, direction: int) -> None:
        """Navega por el historial de comandos tipeados."""
        if not self.history:
            return

        self.history_index += direction
        
        # Limites del índice
        if self.history_index < 0:
            self.history_index = -1
            self.input_field.clear()
            return
        elif self.history_index >= len(self.history):
            self.history_index = len(self.history) - 1

        # Mostramos el comando historico (invertimos la lista para que el mas reciente sea el índice 0)
        reversed_history = list(reversed(self.history))
        self.input_field.setText(reversed_history[self.history_index])


    def _on_send_clicked(self) -> None:
        """
        Slot interno ejecutado al presionar Enter o hacer clic en el botón Send.
        Emite la señal `command_submitted` y limpia el campo de texto.
        """
        command = self.input_field.text().strip()
        if command:
            if not self.history or self.history[-1] != command:
                self.history.append(command)    # Agregar comando a historial

            self.command_submitted.emit(command)    # emite la senal/comando
            self.input_field.clear()
            self.history_index = -1 # Resetear navegacion del historial

    def set_enabled_state(self, is_enabled: bool) -> None:
        """
        Habilita o deshabilita la entrada de comandos (Ej: para Modo Automático).
        
        Args:
            is_enabled (bool): Estado de habilitación deseado.
        """
        # ¡CRÍTICO! Si el estado que me piden es igual al que ya tengo, no hago nada.
        if self._current_enable_state == is_enabled:
            return

        # Si cambió, actualizo mi registro y aplico los cambios pesados
        self._current_enable_state = is_enabled
        
        # El aspecto deshabilitado lo resuelven los pseudo-estados :disabled del QSS
        # ('opacity' no es una propiedad QSS valida para QWidget).
        self.input_field.setEnabled(is_enabled)
        self.btn_send.setEnabled(is_enabled)