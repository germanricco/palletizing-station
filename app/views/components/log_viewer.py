"""
Módulo del componente Visor de Logs.
Muestra la salida de eventos y errores del sistema con formato semántico y auto-scroll.
"""

import html
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPlainTextEdit
)
from PySide6.QtGui import QTextCursor
from app.core.constants import AppConfig


class LogViewer(QWidget):
    """
    Widget que renderiza el área de logs del sistema de manera performante.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el visor de logs."""
        super().__init__(parent)
        self.setObjectName("LogViewerWidget")
        self._setup_ui()

    def _setup_ui(self) -> None:
        """Configura los filtros superiores y el área de texto con auto-scroll."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # --- Área de Texto de Logs ---
        self.text_area = QPlainTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setStyleSheet(
            f"QPlainTextEdit {{"
            f"  background-color: #0A0A0A;"
            f"  color: {AppConfig.COLORS.TEXT_MED};"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"  font-size: {AppConfig.TYPOGRAPHY.SIZE_LOGS};"
            f"  border: none;"
            f"  padding: {AppConfig.SPACING.XS} {AppConfig.SPACING.SM};"
            f"}}"
        )

        layout.addWidget(self.text_area)

    def append_log(self, timestamp: str, level: str, module: str, message: str) -> None:
        """
        Agrega una nueva línea de log formateada al área de texto y aplica auto-scroll.
        
        Args:
            timestamp (str): Marca de tiempo del evento (ej. '14:05:22.123').
            level (str): Nivel de criticidad ('INFO', 'WARN', 'ERROR').
            module (str): Módulo de origen del log.
            message (str): Descripción del evento.
        """
        # Determinar el color semántico basado en el nivel
        color_map = {
            "INFO": AppConfig.COLORS.SEMANTIC_MANUAL, # #00F0FF (Cian)
            "WARN": AppConfig.COLORS.SEMANTIC_RECORD, # #FF9500 (Ámbar)
            "ERROR": AppConfig.COLORS.SEMANTIC_ESTOP  # #FF3B30 (Rojo)
        }
        level_color = color_map.get(level.upper(), AppConfig.COLORS.TEXT_HIGH)

        # Limpiar el mensaje para evitar rotura de HTML
        # Ejemplo: caracter '<' sera tratado como etiqueta HTML y rompera todo.
        safe_message = html.escape(message)
        safe_module = html.escape(module)

        # Construcción HTML de la línea
        log_line = (
            f"<span style='color:#666666;'>[{timestamp}]</span> | "
            f"<span style='color:{level_color}; font-weight:bold;'>[{level}]</span> | "
            f"<span style='color:{AppConfig.COLORS.TEXT_MED};'>[{safe_module}]</span> | "
            f"<span style='color:{AppConfig.COLORS.TEXT_HIGH};'>{safe_message}</span>"
        )

        # Si es un error, el design handoff requiere un fondo especial,
        # en QPlainTextEdit se simula mejor resaltando todo el texto en HTML.
        if level.upper() == "ERROR":
            log_line = f"<div style='background-color: rgba(255, 59, 48, 0.2);'>{log_line}</div>"

        # Añadir mediante appendHtml para procesar los estilos en línea
        self.text_area.appendHtml(log_line)
        
        # Auto-scroll: Forzar el cursor al final del documento
        cursor = self.text_area.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.text_area.setTextCursor(cursor)