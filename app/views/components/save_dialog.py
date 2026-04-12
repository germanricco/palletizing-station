"""
Módulo para el diálogo de guardado de trayectorias.
"""
from PySide6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
from PySide6.QtCore import Qt
from app.core.constants import AppConfig

class SaveTrajectoryDialog(QDialog):
    """Diálogo modal para renombrar y confirmar el guardado de una trayectoria."""
    
    def __init__(self, default_filename: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Guardar Trayectoria")
        self.setFixedSize(400, 180)
        self.setModal(True)
        
        # Estilo general oscuro para coincidir con la UI
        self.setStyleSheet(
            f"QDialog {{"
            f"  background-color: {AppConfig.COLORS.SURFACE};"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"}}"
            f"QLabel {{"
            f"  color: {AppConfig.COLORS.TEXT_HIGH};"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"  font-size: 14px;"
            f"}}"
        )
        
        self._setup_ui(default_filename)

    def _setup_ui(self, default_filename: str) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        # Instrucción
        lbl_info = QLabel("Ingrese un nombre para la nueva trayectoria:")
        layout.addWidget(lbl_info)

        # Input de texto
        self.input_name = QLineEdit(default_filename)
        self.input_name.setStyleSheet(
            f"QLineEdit {{"
            f"  background-color: #0A0A0A;"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 4px;"
            f"  padding: 8px;"
            f"  color: {AppConfig.COLORS.SEMANTIC_MANUAL};"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"}}"
            f"QLineEdit:focus {{"
            f"  border: 1px solid {AppConfig.COLORS.SEMANTIC_MANUAL};"
            f"}}"
        )
        layout.addWidget(self.input_name)

        # Botones
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()

        # Estilo genérico para botones del diálogo
        btn_base_style = (
            "QPushButton {"
            "  border-radius: 4px; padding: 8px 16px; font-weight: bold;"
            "  font-family: %s;"
            "}"
        ) % AppConfig.TYPOGRAPHY.FAMILY_UI

        self.btn_cancel = QPushButton("CANCELAR")
        self.btn_cancel.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_cancel.setStyleSheet(btn_base_style + f"QPushButton {{ background-color: #2A2A2A; color: {AppConfig.COLORS.TEXT_HIGH}; border: none; }}")
        self.btn_cancel.clicked.connect(self.reject) # Método nativo de QDialog

        self.btn_save = QPushButton("GUARDAR")
        self.btn_save.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_save.setStyleSheet(btn_base_style + f"QPushButton {{ background-color: {AppConfig.COLORS.SEMANTIC_AUTO}; color: #121212; border: none; }}")
        self.btn_save.clicked.connect(self.accept) # Método nativo de QDialog

        btn_layout.addWidget(self.btn_cancel)
        btn_layout.addWidget(self.btn_save)
        
        layout.addLayout(btn_layout)

    def get_filename(self) -> str:
        """Retorna el texto ingresado limpiando espacios en blanco."""
        return self.input_name.text().strip()