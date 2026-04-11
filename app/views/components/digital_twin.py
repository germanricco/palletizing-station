"""
Módulo del componente Gemelo Digital (Digital Twin).
Actúa como un lienzo de renderizado con paneles HUD superpuestos mediante
anclaje absoluto y simulación de Z-Index para no obstruir la telemetría.
"""
import logging
import numpy as np

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame
from PySide6.QtGui import QResizeEvent
from PySide6.QtCore import Qt

# Componentes de la UI
from app.core.constants import AppConfig
from app.views.components.wireframe_canvas import WireframeCanvas


class HUDPanel(QFrame):
    """
    Clase base para los paneles flotantes (HUD).
    Implementa la estética de cristal ahumado y dimensiones fijas.
    """
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedWidth(280)

        # Habilitar renderizado de estilos en el sub-frame
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        # Simulamos el cristal ahumado con un color base translúcido (0.8 opacidad).
        self.setStyleSheet(
            f"QFrame {{"
            f"  background-color: rgba(20, 20, 20, 204);"
            f"  border: 1px solid {AppConfig.COLORS.BORDERS};"
            f"  border-radius: 6px;"
            f"}}"
        )


class DigitalTwin(QWidget):
    """
    Contenedor principal del Gemelo Digital.
    Gestiona el renderizado del fondo 3D y la posición absoluta de sus capas HUD.
    """
    # ==== INICIALIZACION Y LAYOUT MAESTRO ====
    def __init__(self, parent: QWidget | None = None) -> None:
        """Inicializa el lienzo y las capas superpuestas."""
        super().__init__(parent)
        self.setObjectName("DigitalTwinCanvas")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {AppConfig.COLORS.BACKGROUND};")
        
        # Capa 0: Layout de fondo exclusivo
        self.bg_layout = QVBoxLayout(self)
        self.bg_layout.setContentsMargins(0, 0, 0, 0)

        # Instanciar y enviar al fondo el Renderizador 3D
        self.canvas_3d = WireframeCanvas()
        self.bg_layout.addWidget(self.canvas_3d)

        # Capa 1 y 2: Componentes Flotantes
        self._setup_huds()
        self._setup_progress_bar()

    
    # ==== CONSTRUCCION DE COMPONENTES ====
    def _setup_huds(self) -> None:
        """Instancia los paneles HUD izquierdo y derecho (Capa 1)."""
        # --- HUD Izquierdo: Sistema y Actuadores ---
        self.hud_left = HUDPanel(self)
        hud_left_layout = QVBoxLayout(self.hud_left)
        hud_left_layout.setContentsMargins(16, 16, 16, 16)
        hud_left_layout.setSpacing(12)

        # Semáforo de Link
        self.lbl_link = QLabel("● STM32 LINK")
        self.lbl_link.setStyleSheet(
            f"color: {AppConfig.COLORS.SEMANTIC_AUTO};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-weight: {AppConfig.TYPOGRAPHY.WEIGHT_BOLD};"
            f"border: none; background: transparent;"
        )
        
        # Estado Lógico
        self.lbl_state = QLabel("ESTADO: IDLE")
        self.lbl_state.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"border: none; background: transparent;"
        )

        # Actuadores (Pills)
        self.lbl_act1 = self._create_actuator_pill("ACT 1: ENABLED", AppConfig.COLORS.SEMANTIC_MANUAL)
        self.lbl_act2 = self._create_actuator_pill("ACT 2: ENABLED", AppConfig.COLORS.SEMANTIC_MANUAL)

        # Agregamos los widgets al Layout del HUD Izquierdo
        for widget in [self.lbl_link, self.lbl_state, self.lbl_act1, self.lbl_act2]:
            hud_left_layout.addWidget(widget)

        self.hud_left.adjustSize()

        # --- HUD Derecho: Cinemática ---
        self.hud_right = HUDPanel(self)
        hud_right_layout = QVBoxLayout(self.hud_right)
        hud_right_layout.setContentsMargins(16, 16, 16, 16)
        hud_right_layout.setSpacing(12)

        # Título Cinemática
        lbl_kin_title = QLabel("POSICION ACTUAL")
        lbl_kin_title.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"border: none; background: transparent;"
            f"border-bottom: 1px solid {AppConfig.COLORS.BORDERS};"
            f"padding-bottom: 8px;"
        )

        # Valores Articulares
        self.lbl_articular = QLabel()
        self.lbl_articular.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: {AppConfig.TYPOGRAPHY.SIZE_HUD_CONSOLE};"
            f"border: none; background: transparent;"
            f"line-height: 1.5;"
        )
        
        # Valores Cartesianos
        self.lbl_cartesian = QLabel()
        self.lbl_cartesian.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: {AppConfig.TYPOGRAPHY.SIZE_HUD_CONSOLE};"
            f"border: none; background: transparent;"
            f"border-top: 1px solid {AppConfig.COLORS.BORDERS};"
            f"padding-top: 8px; line-height: 1.5;"
        )

        # Agregamos widgets al Layout del HUD Derecho
        for widget in [lbl_kin_title, self.lbl_articular, self.lbl_cartesian]:
            hud_right_layout.addWidget(widget)

        # Valores por defecto para visualización inicial
        self._update_kinematics([45.2, -15.0, 90.5], [120.5, -45.2, 210.0])
        self.hud_right.adjustSize()

    def _setup_progress_bar(self) -> None:
        """Instancia la línea de tiempo/progreso inferior (Capa 2)."""
        self.timeline = QFrame(self)
        self.timeline.setFixedHeight(4) # Estado Reposo
        self.timeline_active = False
        
        # Ocultamos texto inicialmente
        self.timeline_layout = QVBoxLayout(self.timeline)
        self.timeline_layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_timeline = QLabel("TRAYECTORIA ACTIVA: WAYPOINT 0/0 - 0%")
        self.lbl_timeline.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_timeline.setStyleSheet(
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};"
            f"font-size: 12px;"
            f"background: transparent;"
        )
        self.lbl_timeline.hide()

        self.timeline_layout.addWidget(self.lbl_timeline)
        self._update_timeline_style(0)
    
    def _create_actuator_pill(self, text: str, border_color: str) -> QLabel:
        """Crea una etiqueta redondeada (pill) para el estado de los actuadores."""
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(
            f"background-color: {AppConfig.COLORS.SURFACE};"
            f"border: 1px solid {border_color};"
            f"border-radius: 12px;"
            f"padding: 4px 8px;"
            f"color: {AppConfig.COLORS.TEXT_HIGH};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 12px;"
        )
        return lbl

    
    # ==== API PUBLICA (SLOTS) ====
    def process_telemetry(self, cartesian_state: np.ndarray, joints: list[float], state_name: str, actuator_state: dict) -> None:
        """
        Procesa el pipeline completo de actualización visual para el Gemelo Digital.
        
        Args:
            cartesian_state (np.ndarray): Matriz (6, 3) con posiciones X, Y, Z.
            joints (list[float]): Lista con los ángulos [J1, J2, J3].
            state_name (str): Nombre del estado lógico actual (ej. "IDLE").
            actuator_state (dict): Diccionario con estados booleanos {'valve': bool, 'pump': bool}.
        """
        try:
            # Actualizar Canvas 3D
            self.canvas_3d.draw_robot(cartesian_state)

            # Extraer el TCP
            if getattr(cartesian_state, 'shape', (0,))[0] >= 6:
                tcp_coords = cartesian_state[5].tolist()
                self._update_kinematics(joints, tcp_coords)
            else:
                # Si array está incompleto, pasamos listas vacías
                self._update_kinematics(joints, [0.0, 0.0, 0.0])

            # Actualizar HUD Izquierdo (Estados y Actuadores)
            self._update_hud_status(state_name, actuator_state)

        except Exception as e:
            logging.error(f"Error procesando telemetría en DigitalTwin: {e}")

    def set_timeline_state(self, is_active: bool, progress: int = 0, text: str = "TRAYECTORIA ACTIVA") -> None:
        """
        Controla la expansión, texto y progreso de la barra inferior de trayectorias.
        """
        self.timeline_active = is_active
        
        if is_active:
            self.timeline.setFixedHeight(24)
            self.lbl_timeline.setText(text)
            self.lbl_timeline.show()
        else:
            self.timeline.setFixedHeight(4)
            self.lbl_timeline.hide()
            
        self._update_timeline_style(progress)
        self.resizeEvent(QResizeEvent(self.size(), self.size()))

    
    # ==== ACTUALIZADORES INTERNOS (HELPERS) ====
    def _update_hud_status(self, state_name: str, actuator_state: dict) -> None:
        """
        Actualiza el panel izquierdo (Modo de la máquina y actuadores finales).
        """
        # Lógica de colores
        color_map = {
            "IDLE": AppConfig.COLORS.TEXT_MED,
            "MANUAL": AppConfig.COLORS.SEMANTIC_MANUAL,
            "RECORDING": AppConfig.COLORS.SEMANTIC_RECORD,
            "AUTO_RUNNING": AppConfig.COLORS.SEMANTIC_AUTO,
            "E_STOP": AppConfig.COLORS.SEMANTIC_ESTOP
        }
        state_color = color_map.get(state_name, AppConfig.COLORS.TEXT_HIGH)

        # Actualizar Etiqueta de Estado
        self.lbl_state.setText(f"ESTADO: {state_name}")
        self.lbl_state.setStyleSheet(f"color: {state_color}; font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI}; border: none; background: transparent;")

        # Actualizar Pills de Actuadores
        valve_active = actuator_state.get('valve', False)
        pump_active = actuator_state.get('pump', False)

        self._refresh_actuator_pill(self.lbl_act1, "VÁLVULA (A1)", valve_active)
        self._refresh_actuator_pill(self.lbl_act2, "BOMBA VAC. (A2)", pump_active)

    def _refresh_actuator_pill(self, label: QLabel, name: str, is_active: bool) -> None:
        """
        Aplica los estilos dinámicos a las etiquetas de los actuadores.
        """
        state_text = "ENABLED" if is_active else "DISABLED"
        border_color = AppConfig.COLORS.SEMANTIC_MANUAL if is_active else AppConfig.COLORS.BORDERS
        
        label.setText(f"{name}: {state_text}")
        label.setStyleSheet(
            f"background-color: {AppConfig.COLORS.SURFACE};"
            f"border: 1px solid {border_color};"
            f"border-radius: 12px;"
            f"padding: 4px 8px;"
            f"color: {AppConfig.COLORS.TEXT_HIGH if is_active else AppConfig.COLORS.TEXT_MED};"
            f"font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};"
            f"font-size: 12px;"
        )

    def _update_kinematics(self, joints: list[float], tcp: list[float]) -> None:
        """
        Actualiza los valores de la cinemática forzando el zero-padding estricto
        para evitar el jitter visual en la fuente monospace.
        
        Args:
            joints (list[float]): Ángulos [J1, J2, J3] en grados.
            tcp (list[float]): Coordenadas cartesianas [X, Y, Z] en mm.
        """
        if len(joints) >= 3:
            self.lbl_articular.setText(
                f"J1: {joints[0]:+07.2f}°\n"
                f"J2: {joints[1]:+07.2f}°\n"
                f"J3: {joints[2]:+07.2f}°")
            
        if len(tcp) >= 3:
            self.lbl_cartesian.setText(
                f"X: {tcp[0]:+08.2f} mm\n"
                f"Y: {tcp[1]:+08.2f} mm\n"
                f"Z: {tcp[2]:+08.2f} mm")
    
    def _update_timeline_style(self, progress: int) -> None:
        """Actualiza el gradiente de progreso de la barra."""
        # Asegurar de que el progreso esté entre 0 y 100
        progress = max(0, min(100, progress))
        
        if not self.timeline_active:
            self.timeline.setStyleSheet(f"background-color: {AppConfig.COLORS.BORDERS};")
        else:
            color_active = AppConfig.COLORS.SEMANTIC_AUTO
            color_bg = AppConfig.COLORS.BORDERS
            
            # Protección matemática: si el progreso es 0, no aplicamos gradiente para evitar fallos de render en Qt
            if progress <= 0:
                self.timeline.setStyleSheet(f"background-color: {color_bg};")
            else:
                ratio = progress / 100.0
                # Se suma un delta mínimo (0.001) al stop para lograr un corte duro (sólido) en lugar de un difuminado
                self.timeline.setStyleSheet(
                    f"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, "
                    f"stop:0 {color_active}, stop:{ratio} {color_active}, "
                    f"stop:{min(1.0, ratio + 0.001)} {color_bg}, stop:1 {color_bg});"
                )


    # ==== EVENTOS NATIVOS (OVERRIDES) ====
    def resizeEvent(self, event: QResizeEvent) -> None:
        """
        Asegura el anclaje absoluto y la jerarquia Z de los componentes flotantes.
        """
        super().resizeEvent(event)
        
        self.hud_left.move(24, 24)
        self.hud_right.move(self.width() - self.hud_right.width() - 24, 24)
        self.timeline.setGeometry(0, self.height() - self.timeline.height(), self.width(), self.timeline.height())

        self.hud_left.raise_()
        self.hud_right.raise_()
        self.timeline.raise_()
