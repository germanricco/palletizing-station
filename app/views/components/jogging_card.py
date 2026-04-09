# --- MODIFICACIÓN EN app/views/components/control_panel.py ---

from PySide6.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox, QSizePolicy, QWidget
)
from PySide6.QtCore import Qt, Signal
from app.core.constants import AppConfig


class JoggingCard(QFrame):
    """
    Tarjeta de teleoperación individual, reconfigurable dinámicamente 
    para soportar coordenadas articulares o cartesianas.
    """
    
    request_step = Signal(float)

    def __init__(self, axis_id: int, parent: QWidget | None = None) -> None:
        """
        Inicializa la tarjeta base.
        
        Args:
            axis_id (int): 0 (Base/X), 1 (Hombro/Y), 2 (Codo/Z)
        """
        super().__init__(parent)
        self.axis_id = axis_id
        self._prefix = "θ:"
        self._format_string = "{:+07.2f}"
        
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f"JoggingCard {{"
            f"  background-color: #1A1A1A;"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 8px;"
            f"}}"
        )
        
        self._setup_ui()
        # Iniciar en modo articular por defecto
        self.set_mode("ARTICULAR")

    def _setup_ui(self) -> None:
        """Estructura los componentes internos de la tarjeta."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(8)

        # --- HEADER ---
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_title = QLabel("GDL")
        self.lbl_title.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 14px;"
            f"font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_SEMIBOLD};"
            f"border: none; background: transparent;"
        )
        
        self.lbl_value = QLabel("θ: +000.00°")
        self.lbl_value.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: 14px;"
            f"border: none; background: transparent;"
        )
        
        header_layout.addWidget(self.lbl_title)
        header_layout.addStretch()
        header_layout.addWidget(self.lbl_value)

        # --- CONTROLES FÍSICOS ---
        controls_layout = QHBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(8)

        btn_style = (
            f"QPushButton {{"
            f"  background-color: #2A2A2A;"
            f"  color: {AppConfig.COLORS.TEXT_HIGH};"
            f"  border-radius: 4px;"
            f"  font-size: 18px;"
            f"  font-weight: bold;"
            f"  border: none;"
            f"}}"
            f"QPushButton:hover {{"
            f"  background-color: {AppConfig.COLORS.BORDERS};"
            f"}}"
        )

        btn_minus = QPushButton("−")
        btn_minus.setFixedSize(40, 40)
        btn_minus.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_minus.setStyleSheet(btn_style)
        btn_minus.clicked.connect(self._on_minus_clicked)

        self.spin_step = QDoubleSpinBox()
        self.spin_step.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        self.spin_step.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_step.setDecimals(1)
        self.spin_step.setFixedHeight(40)
        self.spin_step.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.spin_step.setStyleSheet(
            f"QDoubleSpinBox {{"
            f"  background-color: #0A0A0A;"
            f"  color: {AppConfig.COLORS.TEXT_HIGH};"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 4px;"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"  font-size: 14px;"
            f"}}"
        )

        btn_plus = QPushButton("+")
        btn_plus.setFixedSize(40, 40)
        btn_plus.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_plus.setStyleSheet(btn_style)
        btn_plus.clicked.connect(self._on_plus_clicked)

        controls_layout.addWidget(btn_minus)
        controls_layout.addWidget(self.spin_step)
        controls_layout.addWidget(btn_plus)

        layout.addLayout(header_layout)
        layout.addLayout(controls_layout)

    def set_mode(self, mode: str) -> None:
        """
        Reconfigura la tarjeta basándose en el sistema de coordenadas.
        """
        if mode == "ARTICULAR":
            titles = ["GDL 1 - BASE", "GDL 2 - HOMBRO", "GDL 3 - CODO"]
            self.lbl_title.setText(titles[self.axis_id])
            self._prefix = "θ:"
            self._format_string = "{:+07.2f}°"
            # Rango seguro para motores en grados
            self.spin_step.setRange(0.1, 45.0)
            self.spin_step.setValue(1.0)
            self.spin_step.setSuffix("°")
        else:
            titles = ["TCP - EJE X", "TCP - EJE Y", "TCP - EJE Z"]
            prefixes = ["X:", "Y:", "Z:"]
            self.lbl_title.setText(titles[self.axis_id])
            self._prefix = prefixes[self.axis_id]
            self._format_string = "{:+08.2f} mm"
            # Rango seguro para traslación cartesiana en mm
            self.spin_step.setRange(1.0, 100.0)
            self.spin_step.setValue(10.0)
            self.spin_step.setSuffix(" mm")

    def update_value(self, current_val: float) -> None:
        """Actualiza el visor con padding seguro contra saltos visuales."""
        formatted_val = self._format_string.format(current_val)
        self.lbl_value.setText(f"{self._prefix} {formatted_val}")

    def _on_minus_clicked(self) -> None:
        self.request_step.emit(self.spin_step.value() * -1.0)

    def _on_plus_clicked(self) -> None:
        self.request_step.emit(self.spin_step.value())