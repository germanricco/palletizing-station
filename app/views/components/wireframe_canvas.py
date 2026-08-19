"""
Módulo del Renderizador 3D.
Adapta Matplotlib como un widget nativo de PySide6 para visualizar
la cinemática del robot en tiempo real.
"""

import time

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtWidgets import QSizePolicy
from app.core.constants import AppConfig


class WireframeCanvas(FigureCanvasQTAgg):
    """
    Widget de PySide6 que aloja una figura 3D de Matplotlib para renderizar
    el brazo robótico de forma dinámica.
    """

    def __init__(self, workspace_limits: tuple = (-100, 300, -200, 200, 0, 350)) -> None:
        """
        Inicializa el canvas 3D y aplica el estilizado global de la GUI.
        
        Args:
            workspace_limits (tuple): Límites absolutos (X_min, X_max, Y_min, Y_max, Z_min, Z_max).
        """

        # Configuración de la Figura Base (Fondo oscuro)
        self.fig = Figure(facecolor=AppConfig.COLORS.BACKGROUND)
        super().__init__(self.fig)

        # Widget se expanda a toda la ventana
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.updateGeometry()
        
        # Configuración de Ejes 3D
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(AppConfig.COLORS.BACKGROUND)
        
        # Limites del espacio de trabajo
        self.ax.set_xlim([workspace_limits[0], workspace_limits[1]])
        self.ax.set_ylim([workspace_limits[2], workspace_limits[3]])
        self.ax.set_zlim([workspace_limits[4], workspace_limits[5]])
        
        # Estilización global de los números de la cuadrícula (Ticks)
        self.ax.tick_params(colors=AppConfig.COLORS.TEXT_MED)

        # Estilización de paneles, líneas y texto
        for axis in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            # Hacer transparente el fondo del plano (método moderno con Alpha 0)
            axis.set_pane_color((0.0, 0.0, 0.0, 0.0))
            # Color de las líneas del eje principal
            axis.line.set_color(AppConfig.COLORS.BORDERS)
            # Color de las etiquetas (X, Y, Z)
            axis.label.set_color(AppConfig.COLORS.TEXT_MED)
            
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")
        self.ax.set_zlabel("Z [mm]")

        # 3. Pre-asignación de memoria gráfica
        # Eslabones (Línea Continua - Color Cian)
        self.line_wireframe, = self.ax.plot(
            [], [], [], color=AppConfig.COLORS.SEMANTIC_MANUAL, linewidth=3
        )
        # Articulaciones (Nodos Blancos)
        self.scatter_joints, = self.ax.plot(
            [], [], [], color=AppConfig.COLORS.TEXT_HIGH, marker='o', 
            linestyle='None', markersize=6
        )
        # TCP (Marcador Rojo)
        self.scatter_tcp, = self.ax.plot(
            [], [], [], color=AppConfig.COLORS.SEMANTIC_ESTOP, marker='X', 
            linestyle='None', markersize=10
        )

        # Ajuste de márgenes internos para maximizar área de dibujo
        self.fig.tight_layout()

        # El redibujado 3D (draw_idle) es, por lejos, la operacion mas cara de todo el
        # dashboard (Axes3D no soporta blitting como el 2D). Se throttlea de forma independiente
        # a la tasa de telemetria/UI (20Hz) para no cargar el hilo de la GUI innecesariamente.
        self._last_draw_time = 0.0
        self._min_draw_interval = 1.0 / 15.0  # cap a ~15Hz

        # Inyectar Posición "Home" por defecto para que se vea el brazo
        # al arrancar la UI (Matriz de 6 eslabones x 3 coordenadas X, Y, Z)
        default_home_state = np.array([
            [0.0,     0.0,   0.0],    # 0. GCS (Global Coordinate System)
            [0.0,     0.0,   0.0],    # 1. MCS (Machine Coordinate System)
            [0.0,     0.0,  92.0],    # 2. J1  (Altura L1)
            [0.0,     0.0, 227.0],    # 3. J2  (Hombro vertical: L1 + L2 = 92 + 135)
            [0.0,   147.0, 227.0],    # 4. WP  (Codo extendido en X: L3 = 147)
            [0.0,   117.0, 227.0],    # 5. FLANGE (Caída hacia la herramienta: WP_z - L4 = 227 - 25)
            [0.0,    87.0, 182.0]     # 6. Tool (Posición de la herramienta)
        ], dtype=np.float32)

        self.draw_robot(default_home_state)

    def _update_joints(self, frame_state: np.ndarray) -> None:
        """Actualiza memoria gráfica de nodos y TCP."""
        # ARTICULACIONES Y BRIDAS (Todos los nodos EXCEPTO el último)
        # Al usar [:-1], nos aseguramos de que los puntos blancos lleguen
        # exactamente hasta el Flange/WP, sin importar el tamaño de la matriz.
        X_joints = frame_state[:-1, 0]
        Y_joints = frame_state[:-1, 1]
        Z_joints = frame_state[:-1, 2]
        
        self.scatter_joints.set_data(X_joints, Y_joints)
        self.scatter_joints.set_3d_properties(Z_joints)

        # TCP (Únicamente el ÚLTIMO nodo de la matriz)
        # Al usar [-1:], extraemos dinámicamente la punta de la herramienta.
        X_tcp = frame_state[-1:, 0]
        Y_tcp = frame_state[-1:, 1]
        Z_tcp = frame_state[-1:, 2]
        
        self.scatter_tcp.set_data(X_tcp, Y_tcp)
        self.scatter_tcp.set_3d_properties(Z_tcp)

        # TCP (Índice 5)
        X_tcp, Y_tcp, Z_tcp = frame_state[5:6, 0], frame_state[5:6, 1], frame_state[5:6, 2]
        self.scatter_tcp.set_data(X_tcp, Y_tcp)
        self.scatter_tcp.set_3d_properties(Z_tcp)

    def _update_wireframe(self, frame_state: np.ndarray) -> None:
        """Actualiza memoria gráfica de los eslabones."""
        X_wire = frame_state[:, 0]
        Y_wire = frame_state[:, 1]
        Z_wire = frame_state[:, 2]
        
        self.line_wireframe.set_data(X_wire, Y_wire)
        self.line_wireframe.set_3d_properties(Z_wire)

    def draw_robot(self, cartesian_state: np.ndarray) -> None:
        """
        Punto de entrada de alta frecuencia. Procesa el array y repinta.
        
        Args:
            cartesian_state (np.ndarray): Matriz (N, 6, 3) o (6, 3) provista por el backend.
        """
        # Manejo preventivo de dimensiones (si nos envían un batch de N frames, tomamos el último)
        if cartesian_state.ndim == 3:
            frame_state = cartesian_state[-1]  # Último frame del array (6, 3)
        else:
            frame_state = cartesian_state

        if frame_state.shape != (7, 3):
            raise ValueError(f"Dimensión incorrecta. Se requiere (6, 3), recibido: {frame_state.shape}")

        self._update_wireframe(frame_state)
        self._update_joints(frame_state)

        # Throttle del repintado real: los datos de arriba ya quedaron actualizados (son baratos,
        # solo reemplazan arrays chicos), pero el redibujado en si (draw_idle) se limita aparte.
        now = time.monotonic()
        if (now - self._last_draw_time) < self._min_draw_interval:
            return
        self._last_draw_time = now

        # Refresco asíncrono optimizado para integración en Event Loop de Qt
        self.draw_idle()