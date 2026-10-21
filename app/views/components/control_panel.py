"""
Módulo del componente Panel de Control (Control Panel).
"""

import os
from pathlib import Path

from PySide6.QtWidgets import (
    QDialog,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QLabel,
    QPushButton,
    QSizePolicy,
    QGridLayout,
    QStackedWidget,
    QFileDialog
)

from PySide6.QtCore import Qt, Signal
from app.core.constants import AppConfig
from app.views.components.jogging_card import JoggingCard
from app.views.components.recipe_dialog import RecipeBuilderDialog

class ControlPanel(QWidget):
    """
    Widget principal del Panel de Control.
    """

    # --- SENALES HACIA EL BACKEND ---
    # Senal de Conexion
    request_connection = Signal(bool)

    # Senales de Modo
    request_mode_idle = Signal()
    request_mode_manual = Signal()
    request_mode_auto = Signal()

    # Senales de Memoria
    request_start_recording = Signal()
    request_stop_recording = Signal()
    request_load_trajectory = Signal(str) # envia nombre de archivo

    # Senales de Ciclo Automatico (trayectoria unica o receta, disparado por el sensor)
    request_load_recipe = Signal(list)
    request_start_palletizing = Signal()    # INICIAR CICLO
    request_stop_palletizing = Signal()     # DETENER CICLO
    request_unload_memory = Signal()

    # Senales de Ejecucion
    request_simulate_trajectory = Signal()  # Llama a simulate_trajectory()
    request_stop_simulation = Signal()
    request_pause_trajectory = Signal()     # Llama a stop_trajectory()
    request_resume_trajectory = Signal()    # Llama a resume_trajectory()
    request_cancel_trajectory = Signal()    # Llama a cancel_trajectory()

    # Senales de Jogging
    request_jog = Signal(str, int, float)
    request_toggle_actuator = Signal()
    request_toggle_conveyor = Signal()      # MARCHA/PARADA manual de la cinta (M122)

    # Presentacion de los estados de la cinta (EVT:CINTA) -> (texto, color)
    _CONVEYOR_VIEW = {
        "DESCONOCIDO":    ("DESCONOCIDO",    AppConfig.COLORS.TEXT_MED),
        "DESHABILITADA":  ("DETENIDA",       AppConfig.COLORS.TEXT_MED),
        "AVANZANDO":      ("AVANZANDO",      AppConfig.COLORS.SEMANTIC_AUTO),
        "ESPERANDO_PICK": ("ESPERANDO PICK", AppConfig.COLORS.SEMANTIC_RECORD),
        "MANUAL":         ("MARCHA MANUAL",  AppConfig.COLORS.SEMANTIC_MANUAL),
    }

    # Estilo de boton deshabilitado (visible: Qt no soporta 'opacity' en QSS de QPushButton)
    _DISABLED_BTN_STYLE = (
        "background-color: #1A1A1A; color: #555555; border: 1px solid #2A2A2A;"
        "border-radius: 4px; font-weight: bold;"
    )

    # ==== INICIALIZACION Y LAYOUT MAESTRO ====
    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el Panel de Control y su layout estructural."""
        super().__init__(parent)
        self.setObjectName("ControlPanelWidget")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f"ControlPanel {{"
            f"  background-color: {AppConfig.COLORS.BACKGROUND};"
            f"}}"
        )
        
        self._is_connected = False      # Estado logico interno de la UI
        self._jog_mode = "ARTICULAR"    # Estado del sistema de coordenadas

        self._actuator_is_on = False  # Estado de los actuadores (para lógica de botones)

        self._conveyor_state = "DESCONOCIDO"    # Ultimo EVT:CINTA recibido
        self._conveyor_enabled = False          # Boton MARCHA/PARADA habilitado (IDLE/MANUAL)

        self._has_file_loaded = False           # Hay trayectoria o receta en memoria

        # Firma del ultimo render aplicado en update_machine_state, para no reaplicar estilos
        # cuando nada relevante cambio desde la llamada anterior (ver update_machine_state).
        self._last_render_signature = None

        self._setup_ui()

    def _setup_ui(self) -> None:
        """
        Configura el administrador de diseño vertical y define
        los espacios proporcionales para los sub-módulos.
        """
        layout = QVBoxLayout(self)
        spacing_sm = int(AppConfig.SPACING.SM.replace("px", ""))
        layout.setContentsMargins(spacing_sm, spacing_sm, spacing_sm, spacing_sm)
        layout.setSpacing(int(AppConfig.SPACING.MD.replace("px", "")))

        # Cabecera Sistema y Comunicaciones
        self.frame_comms = self._create_communication_section()

        # Selección de Modos Lógicos
        self.frame_modes = self._create_modes_section()

        # Tarjetas de Teleoperación / Jogging 3 GDL
        self.frame_jogging = self._create_jogging_section()

        # Gestión de Trayectorias y Memoria
        self.frame_memory = self._create_memory_section()

        # Inserción en el Layout Principal
        layout.addWidget(self.frame_comms, stretch=10)
        layout.addWidget(self.frame_modes, stretch=15)
        layout.addWidget(self.frame_jogging, stretch=50)
        layout.addWidget(self.frame_memory, stretch=25)

    
    # ==== CREADORES DE SECCIONES (BUILDERS) ====
    def _create_communication_section(self) -> QFrame:
        """
        Construye el bloque de comunicaciones con el semáforo de estado 
        y el botón de control de enlace UART.
        """
        frame = QFrame()
        frame.setStyleSheet(
            f"QFrame {{"
            f"  background-color: {AppConfig.COLORS.SURFACE};"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 6px;"
            f"}}"
        )
        
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # --- Fila Superior: Indicador de Estado ---
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_conn_icon = QLabel("●")
        self.lbl_conn_text = QLabel("ESTADO: OFFLINE")
        
        status_layout.addWidget(self.lbl_conn_icon)
        status_layout.addWidget(self.lbl_conn_text)
        status_layout.addStretch()  # Empuja el texto hacia la izquierda

        # --- Fila Inferior: Botón Toggle ---
        self.btn_toggle_conn = QPushButton()
        self.btn_toggle_conn.setFixedHeight(40)
        self.btn_toggle_conn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_toggle_conn.clicked.connect(self._on_toggle_clicked)

        layout.addLayout(status_layout)
        layout.addWidget(self.btn_toggle_conn)

        # Forzar el renderizado inicial en modo desconectado
        self.set_connection_state(False)
        
        return frame
    
    def _create_modes_section(self) -> QWidget:
        """
        Crea la fila de botones agrupados horizontalmente para la seleccion de modos
        """
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)  # Gap de 8px según requerimiento

        # Creación de botones
        self.btn_idle = QPushButton("IDLE")
        self.btn_manual = QPushButton("MANUAL")
        self.btn_auto = QPushButton("AUTO")

        # Configuración para que ocupen todo el alto y ancho proporcional (1fr 1fr 1fr)
        policy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        for btn in [self.btn_idle, self.btn_manual, self.btn_auto]:
            btn.setSizePolicy(policy)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            layout.addWidget(btn)

        # Conexión de clics a las señales del widget
        self.btn_idle.clicked.connect(self.request_mode_idle.emit)
        self.btn_manual.clicked.connect(self.request_mode_manual.emit)
        self.btn_auto.clicked.connect(self.request_mode_auto.emit)

        # Aplicar estilo inicial (Todo inactivo)
        self._apply_inactive_styles()

        return container

    def _create_jogging_section(self) -> QWidget:
        """Estructura la sección de Jogging con su cabecera y tarjetas."""
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        # --- HEADER (Título y Botón Toggle de Modo) ---
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        
        lbl_section = QLabel("TELEOPERACIÓN")
        lbl_section.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 16px;"
            f"font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_BOLD};"
        )
        
        self.btn_jog_mode = QPushButton("MODO: ARTICULAR")
        self.btn_jog_mode.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_jog_mode.setFixedHeight(28)
        self.btn_jog_mode.setStyleSheet(self._get_jog_btn_style("ARTICULAR"))
        self.btn_jog_mode.clicked.connect(self._on_toggle_jog_mode)

        header_layout.addWidget(lbl_section)
        header_layout.addStretch()
        header_layout.addWidget(self.btn_jog_mode)

        layout.addLayout(header_layout)

        # --- TARJETAS DE EJES ---
        self.cards = [
            JoggingCard(axis_id=0),
            JoggingCard(axis_id=1),
            JoggingCard(axis_id=2)
        ]

        # Conectar señales enrutándolas por el ID y adjuntando el modo actual
        for i, card in enumerate(self.cards):
            card.request_step.connect(lambda step, axis=i: self.request_jog.emit(self._jog_mode, axis, step))
            layout.addWidget(card)
        
        # --- TARJETA DE ACTUADOR FINAL ---
        self.frame_actuator = QFrame()
        self.frame_actuator.setFixedHeight(64) # Altura fija para mantener proporción con las JoggingCards
        self.frame_actuator.setStyleSheet(
            f"QFrame {{"
            f"  background-color: #1A1A1A;"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 8px;"
            f"}}"
        )
        
        actuator_layout = QHBoxLayout(self.frame_actuator)
        actuator_layout.setContentsMargins(16, 0, 16, 0)
        
        # Etiqueta de la herramienta
        lbl_actuator = QLabel("ACTUADOR FINAL")
        lbl_actuator.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 14px;"
            f"font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_SEMIBOLD};"
            f"border: none;"
        )
        
        # Botón Toggle
        self.btn_actuator = QPushButton("APAGADO")
        self.btn_actuator.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_actuator.setFixedSize(100, 36)
        self.btn_actuator.clicked.connect(self._on_actuator_clicked)
        
        # Aplicamos el estilo inicial (Apagado)
        self._update_actuator_button_style()
        
        actuator_layout.addWidget(lbl_actuator)
        actuator_layout.addStretch()
        actuator_layout.addWidget(self.btn_actuator)
        
        layout.addWidget(self.frame_actuator)

        # --- TARJETA DE CINTA TRANSPORTADORA ---
        layout.addWidget(self._create_conveyor_card())

        # Empuja todo hacia arriba si sobra espacion en la seccion
        layout.addStretch()
        return container

    def _create_conveyor_card(self) -> QFrame:
        """
        Tarjeta de la cinta transportadora: estado en vivo (siempre visible) y marcha/parada
        manual, habilitada solo fuera del ciclo automatico (IDLE/MANUAL).
        """
        self.frame_conveyor = QFrame()
        self.frame_conveyor.setFixedHeight(64)  # Misma altura que la tarjeta del actuador
        self.frame_conveyor.setStyleSheet(
            f"QFrame {{"
            f"  background-color: #1A1A1A;"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 8px;"
            f"}}"
        )

        conveyor_layout = QHBoxLayout(self.frame_conveyor)
        conveyor_layout.setContentsMargins(16, 0, 16, 0)
        conveyor_layout.setSpacing(8)

        lbl_conveyor = QLabel("CINTA TRANSPORTADORA")
        lbl_conveyor.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 14px;"
            f"font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_SEMIBOLD};"
            f"border: none;"
        )

        # Indicador de estado (punto + texto)
        self.lbl_conveyor_icon = QLabel("●")
        self.lbl_conveyor_state = QLabel()

        # Boton Toggle MARCHA / PARADA
        self.btn_conveyor = QPushButton("MARCHA")
        self.btn_conveyor.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_conveyor.setFixedSize(100, 36)
        self.btn_conveyor.clicked.connect(self.request_toggle_conveyor.emit)

        # Columna izquierda en dos lineas: titulo arriba, estado abajo (no se trunca en
        # pantallas angostas y no agrega altura respecto de la tarjeta del actuador)
        info_layout = QVBoxLayout()
        info_layout.setContentsMargins(0, 0, 0, 0)
        info_layout.setSpacing(2)

        state_layout = QHBoxLayout()
        state_layout.setContentsMargins(0, 0, 0, 0)
        state_layout.setSpacing(6)
        state_layout.addWidget(self.lbl_conveyor_icon)
        state_layout.addWidget(self.lbl_conveyor_state)
        state_layout.addStretch()

        info_layout.addStretch()
        info_layout.addWidget(lbl_conveyor)
        info_layout.addLayout(state_layout)
        info_layout.addStretch()

        conveyor_layout.addLayout(info_layout)
        conveyor_layout.addStretch()
        conveyor_layout.addWidget(self.btn_conveyor)

        self._render_conveyor()
        return self.frame_conveyor

    def _create_memory_section(self) -> QWidget:
        """
        Crea la seccion de gestion de trayectorias.
        Implementa una cabecera de estado y una cuadricula 2x2 para las acciones
        """
        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(12)

        # Cabecera de Estado (Feedback)
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)

        lbl_section = QLabel("MEMORIA Y EJECUCIÓN")
        lbl_section.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 12px;"
            f"font-weight: bold;"
        )
        
        # Etiqueta dinamica que se actualiza al cargar un archivo
        self.lbl_loaded_file = QLabel("ARCHIVO: [ NINGUNO ]")
        self.lbl_loaded_file.setStyleSheet(
            f"color: {AppConfig.COLORS.SEMANTIC_MANUAL};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: 12px;"
        )
        
        header_layout.addWidget(lbl_section)
        header_layout.addStretch()
        header_layout.addWidget(self.lbl_loaded_file)

        # --- Cuadricula de Acciones ---
        grid_widget = QWidget()
        layout = QGridLayout(grid_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        # --- FILA 1, COLUMNA 0: Stack de grabacion
        self.stack_record = QStackedWidget()

        self.btn_record = self._create_button("GRABAR TRAY.", "#1E1E1E", AppConfig.COLORS.TEXT_HIGH, "#FF9500")
        self.btn_stop_record = self._create_button("TERMINAR GRAB.", "#FF9500", "#121212", "none")

        self.btn_record.clicked.connect(self.request_start_recording.emit)
        self.btn_stop_record.clicked.connect(self.request_stop_recording.emit)

        self.stack_record.addWidget(self.btn_record)
        self.stack_record.addWidget(self.btn_stop_record)

        # --- FILA 1, COLUMNA 1: Stack de Carga / Descarga
        self.stack_load = QStackedWidget()
        
        # Índice 0: Cargar
        self.btn_load = self._create_button("CARGAR TRAY.", "#1E1E1E", AppConfig.COLORS.TEXT_HIGH, AppConfig.COLORS.BORDERS)
        self.btn_load.clicked.connect(self._on_load_clicked)
        
        # Índice 1: Limpiar Memoria (Rojo)
        self.btn_unload = self._create_button("LIMPIAR MEMORIA", "#2A2A2A", "#FF3B30", "#FF3B30")
        self.btn_unload.clicked.connect(self.request_unload_memory.emit)
        
        self.stack_load.addWidget(self.btn_load)
        self.stack_load.addWidget(self.btn_unload)

        # --- FILA 2, COLUMNA 0: Stack de Simulacion y Cancelacion ---
        self.stack_abort = QStackedWidget()
        
        # Índice 0: Botón SIMULAR (Cian)
        self.btn_simulate = self._create_button("SIMULAR", "#1A1A1A", "#00F0FF", "#00F0FF")
        self.btn_simulate.clicked.connect(self.request_simulate_trajectory.emit)
        
        # Índice 1: Botón DETENER SIMULACIÓN (Estado SIMULANDO)
        self.btn_stop_sim = self._create_button("DETENER SIM.", "#2A2A2A", "#FF3B30", "#FF3B30")
        self.btn_stop_sim.clicked.connect(self.request_stop_simulation.emit)

        # Índice 2: Botón Cancelar activo (Solo cuando está pausado)
        self.btn_cancel = self._create_button("CANCELAR", "#2A2A2A", "#FF3B30", "#FF3B30")
        self.btn_cancel.clicked.connect(self.request_cancel_trajectory.emit)
        
        self.stack_abort.addWidget(self.btn_simulate)   # 0
        self.stack_abort.addWidget(self.btn_stop_sim)   # 1
        self.stack_abort.addWidget(self.btn_cancel)     # 2

        # --- FILA 2, COLUMNA 1: Stack de Ejecución (Ciclo automatico por sensor) ---
        self.stack_execute = QStackedWidget()

        # 0: Iniciar Ciclo (Verde). Arma la cinta; cada caja detectada dispara la trayectoria.
        self.btn_start_cycle = self._create_button("INICIAR CICLO", "#1E1E1E", AppConfig.COLORS.TEXT_HIGH, "#34C759")
        self.btn_start_cycle.clicked.connect(self.request_start_palletizing.emit)

        # 1: Pausar (Ambar)
        self.btn_pause = self._create_button("PAUSAR", "#2A2A2A", "#FF9500", "#FF9500")
        self.btn_pause.clicked.connect(self.request_pause_trajectory.emit)

        # 2: Reanudar (Verde)
        self.btn_resume = self._create_button("REANUDAR", "#1E1E1E", AppConfig.COLORS.TEXT_HIGH, "#34C759")
        self.btn_resume.clicked.connect(self.request_resume_trajectory.emit)

        # 3: Detener Ciclo (Rojo). Visible mientras se espera la proxima caja.
        self.btn_stop_cycle = self._create_button("DETENER CICLO", "#2A2A2A", "#FF3B30", "#FF3B30")
        self.btn_stop_cycle.clicked.connect(self.request_stop_palletizing.emit)

        self.stack_execute.addWidget(self.btn_start_cycle)  # 0
        self.stack_execute.addWidget(self.btn_pause)        # 1
        self.stack_execute.addWidget(self.btn_resume)       # 2
        self.stack_execute.addWidget(self.btn_stop_cycle)   # 3

        # Configuramos políticas para que todos llenen su celda
        policy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        for w in [self.stack_record, self.stack_load, self.stack_abort, self.stack_execute]:
            w.setSizePolicy(policy)

        # Ensamblaje del Grid
        layout.addWidget(self.stack_record, 0, 0)
        layout.addWidget(self.stack_load, 0, 1)
        layout.addWidget(self.stack_abort, 1, 0)
        layout.addWidget(self.stack_execute, 1, 1)

        # Ensamblaje Final
        main_layout.addLayout(header_layout)
        main_layout.addWidget(grid_widget, stretch=1)

        return container


    # ==== UTILIDADES Y ESTILOS UI (HELPERS) ====
    def _create_button(self, text: str, bg_color: str, text_color: str, border_color: str) -> QPushButton:
        """Helper para generar botones con estilos estandarizados y reducir código repetitivo."""
        btn = QPushButton(text)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        border = "none" if border_color == "none" else f"1px solid {border_color}"
        style = (
            f"background-color: {bg_color}; color: {text_color}; "
            f"border: {border}; border-radius: 4px; font-weight: bold;"
        )
        btn.setStyleSheet(style)
        btn.setProperty("base_style", style)    # Para restaurarlo al re-habilitar
        return btn
    
    def _apply_inactive_styles(self) -> None:
        """Aplica el estilo base (inactivo) a todos los botones de modo."""
        base_style = (
            f"background-color: #1E1E1E;"
            f"color: #A0A0A0;"
            f"border: 1px solid #333333;"
            f"border-radius: 6px;"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-weight: bold;"
            f"font-size: 14px;"
        )
        self.btn_idle.setStyleSheet(base_style)
        self.btn_manual.setStyleSheet(base_style)
        self.btn_auto.setStyleSheet(base_style)

    def _get_jog_btn_style(self, mode: str) -> str:
        """Retorna el estilo dinámico para el botón de toggle de coordenadas."""
        base_color = AppConfig.COLORS.SEMANTIC_RECORD if mode == "ARTICULAR" else AppConfig.COLORS.SEMANTIC_MANUAL
        return (
            f"QPushButton {{"
            f"  background-color: transparent;"
            f"  color: {base_color};"
            f"  border: 1px solid {base_color};"
            f"  border-radius: 14px;"
            f"  padding: 0px 12px;"
            f"  font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"  font-size: 12px;"
            f"  font-weight: bold;"
            f"}}"
            f"QPushButton:hover {{"
            f"  background-color: {AppConfig.COLORS.SURFACE};"
            f"}}"
        )

    def _set_button_enabled(self, btn: QPushButton, enabled: bool) -> None:
        """
        Activa o desactiva un botón y actualiza su opacidad visualmente.
        Limpia estados previos para evitar acumulación de estilos.
        """
        btn.setEnabled(enabled)

        # Botones de accion (_create_button) guardan su estilo base; los de modo no, y su
        # estilo vigente lo acaba de aplicar update_machine_state.
        base_style = btn.property("base_style") or btn.styleSheet()
        btn.setStyleSheet(base_style if enabled else self._DISABLED_BTN_STYLE)

    def _update_actuator_button_style(self) -> None:
        """Aplica estilos dinámicos dependiendo de si está encendido o apagado."""
        # Bloqueado (fuera de MANUAL): conserva el estilo apagado/gris de update_machine_state
        # aunque el actuador cambie durante una trayectoria automatica.
        if hasattr(self, '_last_render_signature') and self._last_render_signature is not None \
                and not self.btn_actuator.isEnabled():
            return
        if self._actuator_is_on:
            self.btn_actuator.setText("ENCENDIDO")
            self.btn_actuator.setStyleSheet(
                f"background-color: rgba(0, 240, 255, 0.15);" # Fondo cian translúcido
                f"color: {AppConfig.COLORS.SEMANTIC_MANUAL};" # Cian brillante
                f"border: 1px solid {AppConfig.COLORS.SEMANTIC_MANUAL};"
                f"border-radius: 4px;"
                f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
                f"font-weight: bold;"
                f"font-size: 12px;"
            )
        else:
            self.btn_actuator.setText("APAGADO")
            self.btn_actuator.setStyleSheet(
                f"background-color: #2A2A2A;"
                f"color: #A0A0A0;"
                f"border: none;"
                f"border-radius: 4px;"
                f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
                f"font-weight: bold;"
                f"font-size: 12px;"
            )
    
    
    # ==== ACTUALIZACIONES DE ESTADO (SLOTS UI) ====
    def update_machine_state(self, current_state, progress: int = 0) -> None:
        """
        Actualiza el diseño visual del Segmented Control para reflejar el estado actual.
        Llamado asíncronamente cuando llega nueva telemetría (hasta ~20-50 veces/seg).
        """
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)
        has_file_loaded = self._has_file_loaded

        # Firma de todo lo que influye en el resultado visual de este metodo. Si nada de esto
        # cambio desde la ultima llamada, evitamos recalcular y reaplicar decenas de
        # setStyleSheet() en cada tick de telemetria (el costo real del "parpadeo"/lentitud de UI).
        signature = (state_name, progress > 0, has_file_loaded)
        if signature == self._last_render_signature:
            return
        self._last_render_signature = signature

        self._apply_inactive_styles() # Reset general

        # Estilo activo base (Inversión de contraste sin borde)
        active_style = (
            f"color: #121212; border: none; border-radius: 6px;"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-weight: bold; font-size: 14px;"
        )

        # 1. MAPEO DE ESTADOS LOGICOS A LOS 3 BOTONES PRINCIPALES
        if state_name == "IDLE":
            self.btn_idle.setStyleSheet(f"background-color: #E5E5EA; {active_style}")
            
        elif state_name in ["MANUAL", "RECORDING"]:
            self.btn_manual.setStyleSheet(f"background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; {active_style}")
            
        elif state_name in ["AUTO_READY", "AUTO_RUNNING", "AUTO_PAUSED", "AUTO_WAITING_SENSOR"]:
            self.btn_auto.setStyleSheet(f"background-color: #34C759; {active_style}")

        # Logica de UI para Grabacion
        if state_name == "RECORDING":
            self.stack_record.setCurrentIndex(1)
        else:
            self.stack_record.setCurrentIndex(0)

        # 2. LÓGICA DE GESTIÓN DE MEMORIA (CARGA Y DESCARGA)
        # (has_file_loaded ya se calculo arriba, junto con la firma de render)
        # Si hay archivo, mostramos "LIMPIAR MEMORIA" (1), sino "CARGAR" (0)
        self.stack_load.setCurrentIndex(1 if has_file_loaded else 0)
        
        # Solo se puede interactuar con la memoria si el sistema está en AUTO_READY
        can_manage_memory = (state_name == "AUTO_READY")
        self._set_button_enabled(self.stack_load.currentWidget(), can_manage_memory)

        # 3. LÓGICA DE EJECUCIÓN, SIMULACION Y CICLO AUTOMATICO
        if state_name == "AUTO_READY":
            self.stack_execute.setCurrentIndex(0)   # INICIAR CICLO
            if progress > 0:
                # Simulando
                self.stack_abort.setCurrentIndex(1)  # DETENER SIM.
                self._set_button_enabled(self.btn_simulate, False)
                self._set_button_enabled(self.btn_start_cycle, False)
            else:
                self.stack_abort.setCurrentIndex(0)  # SIMULAR
                self._set_button_enabled(self.btn_simulate, has_file_loaded)
                self._set_button_enabled(self.btn_start_cycle, has_file_loaded)

        elif state_name == "AUTO_WAITING_SENSOR":
            # Ciclo armado: la cinta avanza y se espera la proxima caja
            self.stack_abort.setCurrentIndex(0)
            self.stack_execute.setCurrentIndex(3)   # DETENER CICLO
            self._set_button_enabled(self.btn_simulate, False)
            self._set_button_enabled(self.btn_stop_cycle, True)

        elif state_name == "AUTO_RUNNING":
            # Ejecutándose. Muestra PAUSAR. Bloquea Simular.
            self.stack_abort.setCurrentIndex(0)
            self.stack_execute.setCurrentIndex(1)   # PAUSAR
            self._set_button_enabled(self.btn_simulate, False)
            self._set_button_enabled(self.btn_pause, True)

        elif state_name == "AUTO_PAUSED":
            # Pausado. Aparece CANCELAR (termina el ciclo) y REANUDAR.
            self.stack_abort.setCurrentIndex(2)     # CANCELAR
            self.stack_execute.setCurrentIndex(2)   # REANUDAR
            self._set_button_enabled(self.btn_cancel, True)
            self._set_button_enabled(self.btn_resume, True)

        else:
            # Cualquier otro estado (IDLE, MANUAL, RECORDING, FAULT...). Todo bloqueado.
            self.stack_abort.setCurrentIndex(0)
            self.stack_execute.setCurrentIndex(0)
            self._set_button_enabled(self.btn_simulate, False)
            self._set_button_enabled(self.btn_start_cycle, False)

        # 3b. BLOQUEO TOTAL DE MODOS DURANTE FAULT/EMERGENCY
        # Sin esto los 3 botones de modo quedaban clickeables (inofensivo, RobotController los
        # rechaza por guarda de estado antes de tocar el UART, pero engañoso para el operador).
        is_locked = state_name in ("FAULT", "EMERGENCY")
        for btn in (self.btn_idle, self.btn_manual, self.btn_auto):
            self._set_button_enabled(btn, not is_locked)

        # 4. LÓGICA DE BLOQUEO DE TELEOPERACIÓN
        # (Movido fuera de la cadena if/elif de arriba: es independiente del ramo de ejecucion/
        # simulacion/palletizado y antes quedaba "colgado" del branch de AUTO_READY, dejando sin
        # efecto los casos AUTO_WAITING_SENSOR/AUTO_RUNNING/AUTO_PAUSED de arriba en la practica.)
        is_manual = (state_name == "MANUAL")

        # Bloquear/Habilitar tarjetas de ejes (+ / -)
        for card in getattr(self, 'cards', []):
            card.setEnabled(is_manual)

        # Bloquear/Habilitar botón de Modo Articular/Cartesiano
        if hasattr(self, 'btn_jog_mode'):
            self.btn_jog_mode.setEnabled(is_manual)

        # Bloquear/Habilitar botón del Actuador Final
        if hasattr(self, 'btn_actuator'):
            self.btn_actuator.setEnabled(is_manual)

            # Feedback visual extra para el actuador cuando está bloqueado
            if not is_manual:
                self.btn_actuator.setStyleSheet(
                    f"background-color: #1A1A1A; color: #555555; border: none; border-radius: 4px;"
                    f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI}; font-weight: bold; font-size: 12px;"
                )
            else:
                # Si entramos a MANUAL, restauramos su color normal (Encendido o Apagado)
                self._update_actuator_button_style()

        # 5. MARCHA MANUAL DE LA CINTA (solo fuera del ciclo automatico)
        self._conveyor_enabled = state_name in ("IDLE", "MANUAL")
        self._render_conveyor()

    def set_connection_state(self, is_connected: bool) -> None:
        """
        API Pública para actualizar visualmente el estado de comunicación.
        Implementa los estilos dinámicos del Design Handoff.
        """
        self._is_connected = is_connected

        if is_connected:
            # Estado: CONECTADO (Verde y Cian)
            self.lbl_conn_icon.setStyleSheet(f"color: {AppConfig.COLORS.SEMANTIC_AUTO}; font-size: 16px; border: none; background: transparent;")
            self.lbl_conn_text.setStyleSheet(f"color: {AppConfig.COLORS.TEXT_HIGH}; font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI}; font-weight: bold; border: none; background: transparent;")
            self.lbl_conn_text.setText("ESTADO: ONLINE")
            
            self.btn_toggle_conn.setText("ENLACE SERIAL ACTIVO")
            self.btn_toggle_conn.setStyleSheet(
                f"background-color: rgba(0, 240, 255, 0.1);"
                f"border: 1px solid {AppConfig.COLORS.SEMANTIC_MANUAL};"
                f"color: {AppConfig.COLORS.SEMANTIC_MANUAL};"
                f"border-radius: 4px;"
                f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
                f"font-weight: bold;"
            )
        else:
            # Estado: DESCONECTADO (Rojo y Gris)
            self.lbl_conn_icon.setStyleSheet(f"color: {AppConfig.COLORS.SEMANTIC_ESTOP}; font-size: 16px; border: none; background: transparent;")
            self.lbl_conn_text.setStyleSheet(f"color: {AppConfig.COLORS.TEXT_MED}; font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI}; font-weight: bold; border: none; background: transparent;")
            self.lbl_conn_text.setText("ESTADO: OFFLINE")

            self.btn_toggle_conn.setText("CONECTAR STM32")
            self.btn_toggle_conn.setStyleSheet(
                f"background-color: #2A2A2A;"
                f"border: 1px solid {AppConfig.COLORS.BORDERS};"
                f"color: #8E8E93;"
                f"border-radius: 4px;"
                f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
                f"font-weight: bold;"
            )

    def update_loaded_file_name(self, payload: dict) -> None:
        """
        Actualiza el indicador de archivo cargado en la UI.
        Recibe un diccionario con info.
        Ej: {'type': 'SINGLE', 'name': 'caja_1'} 
        o {'type': 'RECIPE', 'count': 4, 'current': 1, 'pallets': 0}
        """
        self._has_file_loaded = bool(payload)
        self._last_render_signature = None      # Fuerza re-render de botones con la nueva carga

        # 1. Si llega None o vacío, reiniciamos el estado
        if not payload:
            self.lbl_loaded_file.setText("ARCHIVO: [ NINGUNO ]")
            self.lbl_loaded_file.setStyleSheet(
                f"color: {AppConfig.COLORS.SEMANTIC_MANUAL};"
                f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
                f"font-size: 12px;"
            )
            return

        # 2. Si hay datos, preparamos el color verde de activo
        self.lbl_loaded_file.setStyleSheet(
            f"color: {AppConfig.COLORS.SEMANTIC_AUTO};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: 12px;"
        )
        
        # 3. Enrutamos según el tipo de carga
        payload_type = payload.get('type')
        
        if payload_type == 'SINGLE':
            filename = payload.get('name', 'DESCONOCIDO')
            boxes = payload.get('boxes', 0)
            text = f"ARCHIVO: [ {filename.upper()} ]"
            if boxes:
                text += f" | CAJAS: {boxes}"
            self.lbl_loaded_file.setText(text)
            
        elif payload_type == 'RECIPE':
            # Muestra feedback de estación de palletizado en vivo
            current = payload.get('current', 0)
            count = payload.get('count', 0)
            pallets = payload.get('pallets', 0)
            
            self.lbl_loaded_file.setText(f"RECETA: [ CAJA {current}/{count} ] | PALLETS: {pallets}")

    def update_conveyor_state(self, state: str) -> None:
        """
        Slot (hilo de la GUI) con el estado de la cinta reportado por el firmware (EVT:CINTA).
        """
        self._conveyor_state = state
        self._render_conveyor()

    def _render_conveyor(self) -> None:
        """Aplica el estado de la cinta y la habilitacion del boton a la tarjeta."""
        text, color = self._CONVEYOR_VIEW.get(self._conveyor_state, (self._conveyor_state, AppConfig.COLORS.TEXT_MED))

        self.lbl_conveyor_icon.setStyleSheet(f"color: {color}; font-size: 14px; border: none; background: transparent;")
        self.lbl_conveyor_state.setText(text)
        self.lbl_conveyor_state.setStyleSheet(
            f"color: {color}; font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: 12px; font-weight: bold; border: none; background: transparent;"
        )

        running_manual = (self._conveyor_state == "MANUAL")
        self.btn_conveyor.setText("PARADA" if running_manual else "MARCHA")
        self.btn_conveyor.setEnabled(self._conveyor_enabled)

        if not self._conveyor_enabled:
            style = "background-color: #1A1A1A; color: #555555; border: none;"
        elif running_manual:
            style = (
                f"background-color: rgba(0, 240, 255, 0.15);"
                f"color: {AppConfig.COLORS.SEMANTIC_MANUAL};"
                f"border: 1px solid {AppConfig.COLORS.SEMANTIC_MANUAL};"
            )
        else:
            style = "background-color: #2A2A2A; color: #A0A0A0; border: none;"

        self.btn_conveyor.setStyleSheet(
            f"{style} border-radius: 4px;"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI}; font-weight: bold; font-size: 12px;"
        )

    def update_telemetry(self, joints: list[float], tcp: list[float]) -> None:
        """
        Alimenta las tarjetas con los datos del hardware, 
        filtrando según el modo activo.
        """
        data_source = joints if self._jog_mode == "ARTICULAR" else tcp
        
        if len(data_source) >= 3:
            for i in range(3):
                self.cards[i].update_value(data_source[i])

    def update_actuator_state(self, is_on: bool) -> None:
        """
        Llamado asíncronamente por el backend si el actuador cambia de estado
        por motivos externos (ej. Telemetría o Rutina automática).
        """
        if getattr(self, '_actuator_is_on', None) != is_on:
            self._actuator_is_on = is_on
            self._update_actuator_button_style()

            
    # ==== EVENTOS DE INTERACCION (CLICKS) ====
    def _on_toggle_jog_mode(self) -> None:
        """Cambia el sistema de coordenadas y actualiza la UI."""
        self._jog_mode = "CARTESIANO" if self._jog_mode == "ARTICULAR" else "ARTICULAR"
        
        self.btn_jog_mode.setText(f"MODO: {self._jog_mode}")
        self.btn_jog_mode.setStyleSheet(self._get_jog_btn_style(self._jog_mode))
        
        for card in self.cards:
            card.set_mode(self._jog_mode)

    def _on_load_clicked(self) -> None:
        """Abre un cuadro de diálogo nativo para seleccionar la trayectoria."""
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../trajectories'))
        
        # Validacion de existencia de directorio
        if not os.path.exists(base_dir): os.makedirs(base_dir)

        dialog = RecipeBuilderDialog(base_dir, self)

        
        if dialog.exec() == QDialog.DialogCode.Accepted:
            load_type, data = dialog.get_results()
            if load_type == 'SINGLE':
                self.request_load_trajectory.emit(data[0])
            elif load_type == 'RECIPE':
                self.request_load_recipe.emit(data)

    def _on_toggle_clicked(self) -> None:
        """Slot interno que emite la intención del usuario hacia el orquestador."""
        # Si NO estamos conectados, solicitamos conexión (True), y viceversa.
        self.request_connection.emit(not self._is_connected)

    def _on_actuator_clicked(self) -> None:
        """Invierte el estado interno y emite la orden al backend."""
        self.request_toggle_actuator.emit()


