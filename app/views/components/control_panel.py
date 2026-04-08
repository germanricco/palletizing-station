"""
Módulo del componente Panel de Control (Control Panel).
Contiene la estructura base para la interfaz de operación, dividida en
comunicaciones, modos lógicos, jogging de GDL y gestión de trayectorias.
"""

import os
from pathlib import Path

from PySide6.QtWidgets import (
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


class ControlPanel(QWidget):
    """
    Widget principal del Panel de Control.
    Estructura dinámicamente las zonas de interacción del operador
    utilizando proporciones (stretch factors).
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

    # Senales de Ejecucion
    request_execute_auto = Signal()
    request_simulate_trajectory = Signal()  # Llama a simulate_trajectory()
    request_stop_simulation = Signal()
    request_pause_trajectory = Signal()     # Llama a stop_trajectory()
    request_resume_trajectory = Signal()    # Llama a resume_trajectory()
    request_cancel_trajectory = Signal()    # Llama a cancel_trajectory()


    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el Panel de Control y su layout estructural."""
        super().__init__(parent)
        self.setObjectName("ControlPanelWidget")
        
        # Habilitar el renderizado de estilos QSS en el QWidget base
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")
        
        self._is_connected = False # Estado logico interno de la UI

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

        # 1. Cabecera Sistema y Comunicaciones (~10%)
        self.frame_comms = self._create_communication_section()
        
        # 2. Selección de Modos Lógicos (~15%)
        self.frame_modes = self._create_modes_section()

        # 3. Tarjetas de Teleoperación / Jogging 3 GDL (~50%)
        self.frame_jogging = self._create_section_placeholder("3. Teleoperación (GDL 1, 2, 3)")
        
        # 4. Gestión de Trayectorias y Memoria (~25%)
        self.frame_memory = self._create_memory_section()

        # Inserción en el Layout Padre con los Stretch Factors del Design Handoff
        layout.addWidget(self.frame_comms, stretch=10)
        layout.addWidget(self.frame_modes, stretch=15)
        layout.addWidget(self.frame_jogging, stretch=50)
        layout.addWidget(self.frame_memory, stretch=25)



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
    
    def _on_toggle_clicked(self) -> None:
        """Slot interno que emite la intención del usuario hacia el orquestador."""
        # Si NO estamos conectados, solicitamos conexión (True), y viceversa.
        self.request_connection.emit(not self._is_connected)

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



    # METODOS RELACIONADOS A SECCION DE MODOS
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

    
    # METODOS RELACIONADOS A SECCION DE TRAYECTORIAS Y MEMORIAS
    def _create_memory_section(self) -> QWidget:
        """Crea la cuadrícula 2x2 para gestión de trayectorias."""
        container = QWidget()
        layout = QGridLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12) # Gap de 12px requerido

        # --- FILA 1: Grabar/Terminar y Cargar ---
        self.stack_record = QStackedWidget()
        self.btn_record = self._create_button("GRABAR TRAYECTORIA", "#1E1E1E", {AppConfig.COLORS.TEXT_HIGH}, "#FF9500")
        self.btn_stop_record = self._create_button("TERMINAR GRABACIÓN", "#FF9500", "#121212", "none")
        self.btn_record.clicked.connect(self.request_start_recording.emit)
        self.btn_stop_record.clicked.connect(self.request_stop_recording.emit)
        self.stack_record.addWidget(self.btn_record)
        self.stack_record.addWidget(self.btn_stop_record)

        self.btn_load = self._create_button("CARGAR TRAYECTORIA", "#1E1E1E", {AppConfig.COLORS.TEXT_HIGH}, AppConfig.COLORS.BORDERS)
        self.btn_load.clicked.connect(self._on_load_clicked)

        # --- FILA 2, COLUMNA 0: Stack de Abortar/Cancelar ---
        self.stack_abort = QStackedWidget()
        
        # Índice 0: Botón SIMULAR (Reemplaza al dummy bloqueado)
        self.btn_simulate = self._create_button("SIMULAR", "#1A1A1A", "#00F0FF", "#00F0FF")
        self.btn_simulate.clicked.connect(self.request_simulate_trajectory.emit)
        
        # Índice 1: Botón DETENER SIMULACIÓN (Estado SIMULANDO)
        self.btn_stop_sim = self._create_button("DETENER SIM.", "#2A2A2A", "#FF3B30", "#FF3B30")
        self.btn_stop_sim.clicked.connect(self.request_stop_simulation.emit)

        # Índice 1: Botón Cancelar activo (Solo cuando está pausado)
        self.btn_cancel = self._create_button("CANCELAR", "#2A2A2A", "#FF3B30", "#FF3B30")
        self.btn_cancel.clicked.connect(self.request_cancel_trajectory.emit)
        
        self.stack_abort.addWidget(self.btn_simulate)   # 0
        self.stack_abort.addWidget(self.btn_stop_sim)   # 1
        self.stack_abort.addWidget(self.btn_cancel)     # 2

        # --- FILA 2, COLUMNA 1: Stack de Ejecución ---
        self.stack_execute = QStackedWidget()
        
        # Índice 0: Ejecutar (Requiere AUTO_READY)
        self.btn_execute = self._create_button("EJECUTAR", "#1E1E1E", {AppConfig.COLORS.TEXT_HIGH}, "#34C759")
        self.btn_execute.clicked.connect(self.request_execute_auto.emit)
        
        # Índice 1: Pausar (Aparece en AUTO_RUNNING)
        self.btn_pause = self._create_button("PAUSAR", "#2A2A2A", "#FF9500", "#FF9500")
        self.btn_pause.clicked.connect(self.request_pause_trajectory.emit)
        
        # Índice 2: Reanudar (Aparece en AUTO_PAUSED)
        self.btn_resume = self._create_button("REANUDAR", "#1E1E1E", {AppConfig.COLORS.TEXT_HIGH}, "#34C759")
        self.btn_resume.clicked.connect(self.request_resume_trajectory.emit)
        
        self.stack_execute.addWidget(self.btn_execute)
        self.stack_execute.addWidget(self.btn_pause)
        self.stack_execute.addWidget(self.btn_resume)

        # Configuramos políticas para que todos llenen su celda
        policy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        for w in [self.stack_record, self.btn_load, self.stack_abort, self.stack_execute]:
            w.setSizePolicy(policy)

        layout.addWidget(self.stack_record, 0, 0)
        layout.addWidget(self.btn_load, 0, 1)
        layout.addWidget(self.stack_abort, 1, 0)
        layout.addWidget(self.stack_execute, 1, 1)

        return container


    def _create_button(self, text: str, bg_color: str, text_color: str, border_color: str) -> QPushButton:
        """Helper para generar botones con estilos estandarizados y reducir código repetitivo."""
        btn = QPushButton(text)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        border = "none" if border_color == "none" else f"1px solid {border_color}"
        btn.setStyleSheet(
            f"background-color: {bg_color}; color: {text_color}; "
            f"border: {border}; border-radius: 4px; font-weight: bold;"
        )
        return btn


    def _on_load_clicked(self) -> None:
        """Abre un cuadro de diálogo nativo para seleccionar la trayectoria."""
        # Se asume que la carpeta 'trajectories' está en el directorio base
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../trajectories'))
        
        # Si la carpeta no existe, la crea (prevención de crash)
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)

        # Abre el popup filtrando solo los formatos soportados
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Seleccionar Trayectoria",
            base_dir,
            "Archivos NumPy (*.npy);;Todos los archivos (*)"
        )

        if file_path:
            # Extrae solo el nombre del archivo sin extensión (ej: 'record_171000234' de 'record_171000234.npy')
            # Esto se adapta a cómo tu TrajectoryRepository suele guardar y cargar.
            trajectory_name = Path(file_path).stem 
            self.request_load_trajectory.emit(trajectory_name)


    def update_machine_state(self, current_state) -> None:
        """
        Actualiza el diseño visual del Segmented Control para reflejar el estado actual.
        Llamado asíncronamente cuando llega nueva telemetría.
        """
        self._apply_inactive_styles() # Reset general
        state_name = current_state.name if hasattr(current_state, 'name') else str(current_state)

        # Estilo activo base (Inversión de contraste sin borde)
        active_style = (
            f"color: #121212; border: none; border-radius: 6px;"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-weight: bold; font-size: 14px;"
        )

        # Mapeo de estados lógicos a los 3 botones principales
        if state_name == "IDLE":
            # Blanco o gris muy claro para IDLE
            self.btn_idle.setStyleSheet(f"background-color: #E5E5EA; {active_style}")
            
        elif state_name in ["MANUAL", "RECORDING"]:
            # Cian para operaciones manuales/grabación
            self.btn_manual.setStyleSheet(f"background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; {active_style}")
            
        elif state_name in ["AUTO_READY", "AUTO_RUNNING", "AUTO_PAUSED"]:
            # Verde para automático (Ajustar color si existe uno específico en AppConfig)
            self.btn_auto.setStyleSheet(f"background-color: #34C759; {active_style}")

        # Logica de UI para Grabacion
        if state_name == "RECORDING":
            self.stack_record.setCurrentIndex(1)
        else:
            self.stack_record.setCurrentIndex(0)

        # 2. LÓGICA DE EJECUCIÓN DINÁMICA
        # Solo permitimos abrir el explorador de archivos si estamos listos en AUTO
        can_load = (state_name == "AUTO_READY")
        self._set_button_enabled(self.btn_load, can_load)

        # 4. LÓGICA DE EJECUCIÓN DINÁMICA Y ABORTO
        if state_name == "AUTO_READY":
            # Trayectoria cargada/lista. SIMULAR y EJECUTAR activos.
            self.stack_abort.setCurrentIndex(0)
            self.stack_execute.setCurrentIndex(0)
            
            # Bloqueamos Simular si no hay nada cargado (lo controlamos desde afuera, pero lo habilitamos visualmente si llegó a AUTO_READY)
            self._set_button_enabled(self.btn_simulate, True)
            self._set_button_enabled(self.btn_execute, True)
            
        elif state_name == "AUTO_RUNNING":
            # Ejecutándose. Cambia Ejecutar por PAUSAR. Bloquea Simular.
            self.stack_abort.setCurrentIndex(0)
            self.stack_execute.setCurrentIndex(1)
            self._set_button_enabled(self.btn_simulate, False)
            
        elif state_name == "AUTO_PAUSED":
            # Pausado. Aparece CANCELAR y REANUDAR.
            self.stack_abort.setCurrentIndex(1)
            self.stack_execute.setCurrentIndex(2)
            
        else:
            # Cualquier otro estado (IDLE, MANUAL, RECORDING). Todo bloqueado.
            self.stack_abort.setCurrentIndex(0)
            self.stack_execute.setCurrentIndex(0)
            self._set_button_enabled(self.btn_simulate, False)
            self._set_button_enabled(self.btn_execute, False)

    def _set_button_enabled(self, btn: QPushButton, enabled: bool) -> None:
        """
        Activa o desactiva un botón y actualiza su opacidad visualmente.
        Limpia estados previos para evitar acumulación de estilos.
        """
        btn.setEnabled(enabled)
        # Limpiamos cualquier opacidad previa
        base_style = btn.styleSheet().replace(" opacity: 0.5;", "")
        
        # Aplicamos la opacidad solo si está deshabilitado
        if not enabled:
            base_style += " opacity: 0.5;"
            
        btn.setStyleSheet(base_style)


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