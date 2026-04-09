"""
Módulo del componente Panel de Seguridad (Safety Panel).
Proporciona el botón de Parada de Emergencia (E-Stop) con enclavamiento visual
y control de reinicio del sistema.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QSizePolicy
from PySide6.QtCore import Qt, Signal
from app.core.constants import AppConfig


class SafetyPanel(QWidget):
    """
    Widget de seguridad superior derecho. 
    Contiene el botón E-Stop de máxima prioridad.
    """

    # --- SEÑALES HACIA EL BACKEND ---
    request_e_stop = Signal()
    request_e_reset = Signal()  # Preparado para futura implementación de reset en backend

    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el panel de seguridad y su botón E-Stop."""
        super().__init__(parent)
        self.setObjectName("SafetyPanelWidget")
        
        # Fondo Surface (#1E1E1E)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {AppConfig.COLORS.SURFACE};")
        
        self._is_emergency = False
        
        self._setup_ui()

    def _setup_ui(self) -> None:
        """Configura el layout centrando el botón con los márgenes del Handoff."""
        layout = QVBoxLayout(self)
        
        # Padding 40px (XL) alrededor del botón E-Stop
        pad_xl = int(AppConfig.SPACING.XL.replace("px", ""))
        layout.setContentsMargins(pad_xl, pad_xl, pad_xl, pad_xl)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.btn_estop = QPushButton()
        self.btn_estop.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.btn_estop.setCursor(Qt.CursorShape.PointingHandCursor)
        
        # Aplicamos el estilo inicial normal
        self._apply_normal_style()
        
        self.btn_estop.clicked.connect(self._on_click)
        layout.addWidget(self.btn_estop)

    def _apply_normal_style(self) -> None:
        """Estilo de alerta máxima: Fondo rojo sólido, texto blanco."""
        self.btn_estop.setText("E-STOP")
        self.btn_estop.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: {AppConfig.COLORS.SEMANTIC_ESTOP};"
            f"  color: {AppConfig.COLORS.TEXT_HIGH};"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"  font-size: {AppConfig.TYPOGRAPHY.SIZE_ESTOP};"
            f"  font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_BOLD};"
            f"  border-radius: 8px;"
            f"  border: none;"
            f"}}"
            f"QPushButton:hover {{"
            f"  background-color: #FF5C53;"  # Rojo ligeramente más brillante en hover
            f"}}"
        )

    def _apply_emergency_style(self) -> None:
        """Estilo de enclavamiento: Fondo transparente, reborde rojo (parpadeo lógico)."""
        self.btn_estop.setText("RESET SYSTEM")
        self.btn_estop.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: transparent;"
            f"  color: {AppConfig.COLORS.SEMANTIC_ESTOP};"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"  font-size: {AppConfig.TYPOGRAPHY.SIZE_ESTOP};"
            f"  font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_BOLD};"
            f"  border-radius: 8px;"
            f"  border: 2px solid {AppConfig.COLORS.SEMANTIC_ESTOP};"
            f"}}"
            f"QPushButton:hover {{"
            f"  background-color: rgba(255, 59, 48, 0.1);"
            f"}}"
        )

    def _on_click(self) -> None:
        """Enruta el clic del usuario según el estado de la máquina."""
        if not self._is_emergency:
            self.request_e_stop.emit()
        else:
            self.request_e_reset.emit()

    def update_safety_status(self, state_name: str) -> None:
        """
        Alterna la visualización del botón dependiendo si la máquina 
        reporta estado de emergencia.
        """
        is_emg = (state_name == "EMERGENCY")
        
        # Solo repintar si el estado ha cambiado
        if is_emg != self._is_emergency:
            self._is_emergency = is_emg
            if is_emg:
                self._apply_emergency_style()
            else:
                self._apply_normal_style()