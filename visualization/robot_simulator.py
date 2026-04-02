import matplotlib.pyplot as plt
import numpy as np

class WorkspaceVisualizer:
    """
    Clase dedicada exclusivamente a la visualización de datos espaciales.
    Actúa como consumidor pasivo de matrices cartesianas, respetando el SRP.
    """

    def __init__(self, figure_size: tuple = (12, 6)):
        """
        Inicializa el motor de visualización y prepara la estructura de la figura.
        """
        self.fig = plt.figure(figsize=figure_size)
        self.fig.canvas.manager.set_window_title("Validación de Cinemática Directa - EEZYbotARM Mk2")
        
        # Definición de la grilla de subplots (1 fila, 2 columnas)
        self.ax_xy = self.fig.add_subplot(1, 2, 1)
        self.ax_xz = self.fig.add_subplot(1, 2, 2)

    def plot_static_workspace(self, point_cloud: np.ndarray, point_size: float = 1.0, alpha: float = 0.5) -> None:
        """
        Genera una visualización estática de proyecciones ortogonales de una nube de puntos.
        
        Parámetros:
            point_cloud: Matriz de dimensiones (N, 3) con coordenadas [X, Y, Z].
            point_size: Tamaño del marcador para el scatter plot.
            alpha: Opacidad de los puntos para facilitar la visualización de densidad.
        """
        if point_cloud.ndim != 2 or point_cloud.shape[1] != 3:
            raise ValueError("La matriz de la nube de puntos debe tener dimensiones (N, 3).")

        # Extracción vectorizada de coordenadas
        X = point_cloud[:, 0]
        Y = point_cloud[:, 1]
        Z = point_cloud[:, 2]

        # ---------------------------------------------------------
        # Subplot 1: Vista Superior (Plano XY)
        # ---------------------------------------------------------
        self.ax_xy.clear()
        self.ax_xy.scatter(X, Y, s=point_size, c='blue', alpha=alpha, edgecolors='none')
        self.ax_xy.set_title("Vista Superior (Plano XY) - Restricción Base")
        self.ax_xy.set_xlabel("Eje X [mm]")
        self.ax_xy.set_ylabel("Eje Y [mm]")
        self.ax_xy.grid(True, linestyle='--', alpha=0.7)
        self.ax_xy.set_aspect('equal', adjustable='datalim')
        
        # Marcador del origen (GCS/MCS)
        self.ax_xy.plot(0, 0, marker='+', color='red', markersize=10, label="Origen (0,0)")
        self.ax_xy.legend(loc="upper right")

        # ---------------------------------------------------------
        # Subplot 2: Vista Lateral (Plano XZ)
        # ---------------------------------------------------------
        self.ax_xz.clear()
        self.ax_xz.scatter(X, Z, s=point_size, c='green', alpha=alpha, edgecolors='none')
        self.ax_xz.set_title("Vista Lateral (Plano XZ) - Envolvente de Alcance")
        self.ax_xz.set_xlabel("Eje X [mm]")
        self.ax_xz.set_ylabel("Eje Z [mm]")
        self.ax_xz.grid(True, linestyle='--', alpha=0.7)
        self.ax_xz.set_aspect('equal', adjustable='datalim')
        
        # Marcador del origen (GCS/MCS)
        self.ax_xz.plot(0, 0, marker='+', color='red', markersize=10, label="Origen (0,0)")
        self.ax_xz.legend(loc="upper right")

        # Ajuste de layout para evitar solapamiento de etiquetas
        self.fig.tight_layout()

    def show(self) -> None:
        """Despliega la ventana de Matplotlib de forma bloqueante."""
        plt.show()



class WireframeVisualizer3D:
    """
    Simulador 3D Dinámico para el manipulador EEZYbotARM Mk2.
    Implementa actualización de propiedades de objetos para renderizado en tiempo real.
    """

    def __init__(self, workspace_limits: tuple = (-200, 200, -200, 200, 0, 350)):
        """
        Inicializa el entorno de simulación 3D.
        
        Parámetros:
            workspace_limits: Tupla con los límites absolutos del espacio de trabajo 
                              (X_min, X_max, Y_min, Y_max, Z_min, Z_max) en mm.
        """
        # Habilitar modo interactivo para simulación no bloqueante
        plt.ion()
        
        self.fig = plt.figure(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title("Simulador Dinámico - Wireframe 3D")
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 1. Configuración rígida del entorno espacial
        self.ax.set_xlim([workspace_limits[0], workspace_limits[1]])
        self.ax.set_ylim([workspace_limits[2], workspace_limits[3]])
        self.ax.set_zlim([workspace_limits[4], workspace_limits[5]])
        
        self.ax.set_xlabel("Eje X [mm]")
        self.ax.set_ylabel("Eje Y [mm]")
        self.ax.set_zlabel("Eje Z [mm]")
        self.ax.set_title("Cinemática Directa en Tiempo Real")
        
        # 2. Pre-asignación de memoria gráfica (Instanciación de objetos Line3D)
        # Objeto para los eslabones (línea continua azul)
        self.line_wireframe, = self.ax.plot([], [], [], color='blue', linewidth=3, label='Eslabones')
        
        # Objeto para las articulaciones intermedias (marcadores negros)
        self.scatter_joints, = self.ax.plot([], [], [], color='black', marker='o', 
                                            linestyle='None', markersize=6, label='Articulaciones')
        
        # Objeto exclusivo para el TCP (marcador rojo prominente)
        self.scatter_tcp, = self.ax.plot([], [], [], color='red', marker='X', 
                                         linestyle='None', markersize=10, label='TCP')
        
        self.ax.legend()

    def _update_joints(self, cartesian_state: np.ndarray) -> None:
        """
        Extrae y actualiza las coordenadas de las articulaciones y el TCP.
        Implementa el Requerimiento 1.
        """
        # Las articulaciones corresponden a las filas 0 a 4
        X_joints = cartesian_state[:5, 0]
        Y_joints = cartesian_state[:5, 1]
        Z_joints = cartesian_state[:5, 2]
        
        self.scatter_joints.set_data(X_joints, Y_joints)
        self.scatter_joints.set_3d_properties(Z_joints)

        # El TCP corresponde a la fila 5
        X_tcp = cartesian_state[5:6, 0]
        Y_tcp = cartesian_state[5:6, 1]
        Z_tcp = cartesian_state[5:6, 2]
        
        self.scatter_tcp.set_data(X_tcp, Y_tcp)
        self.scatter_tcp.set_3d_properties(Z_tcp)

    def _update_wireframe(self, cartesian_state: np.ndarray) -> None:
        """
        Extrae y actualiza la topología de los eslabones rígidos.
        Implementa el Requerimiento 2.
        """
        # Se requiere todo el vector para trazar las líneas de 0 a 5
        X_wire = cartesian_state[:, 0]
        Y_wire = cartesian_state[:, 1]
        Z_wire = cartesian_state[:, 2]

        self.line_wireframe.set_data(X_wire, Y_wire)
        self.line_wireframe.set_3d_properties(Z_wire)

    def draw_robot(self, cartesian_state: np.ndarray) -> None:
        """
        Punto de entrada para la simulación dinámica. Integra todos los componentes
        y fuerza el refresco del buffer gráfico.
        Implementa los Requerimientos 3 y 4.
        
        Parámetros:
            cartesian_state: Matriz de dimensiones (6, 3) provista por get_cartesian_state()
        """
        if cartesian_state.shape != (6, 3):
            raise ValueError("Dimensión de matriz incorrecta. Se requiere (6, 3).")

        self._update_wireframe(cartesian_state)
        self._update_joints(cartesian_state)

        # Refresco asíncrono de alto rendimiento
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()



class LateralWireframeVisualizer:
    """
    Visualizador bidimensional de alto rendimiento para el Plano XZ.
    Diseñado para auditar el seguimiento de trayectorias en la frontera del espacio de trabajo.
    """
    def __init__(self, boundary_tcp_points: np.ndarray, workspace_limits: tuple = (-50, 400, -50, 350)):
        """
        Inicializa el entorno gráfico ortogonal.
        
        Parámetros:
            boundary_tcp_points: Matriz (N, 3) con las coordenadas cartesianas de la frontera.
            workspace_limits: Tupla (X_min, X_max, Z_min, Z_max) en mm.
        """
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Validación Dinámica de Frontera (Plano XZ)")
        
        # 1. Configuración rígida del espacio (proyección ortogonal)
        self.ax.set_xlim([workspace_limits[0], workspace_limits[1]])
        self.ax.set_ylim([workspace_limits[2], workspace_limits[3]])
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel("Eje X (Radial) [mm]")
        self.ax.set_ylabel("Eje Z (Elevación) [mm]")
        self.ax.set_title("Recorrido Dinámico sobre Envolvente Máxima")
        self.ax.grid(True, linestyle='--', alpha=0.7)

        # 2. Renderizado estático de la frontera analítica
        X_bound = boundary_tcp_points[:, 0]
        Z_bound = boundary_tcp_points[:, 2]
        self.ax.scatter(X_bound, Z_bound, c='lightgray', s=15, label='Frontera Teórica (TCP)')

        # 3. Pre-asignación de memoria para objetos dinámicos (Wireframe)
        self.line_wireframe, = self.ax.plot([], [], color='blue', linewidth=3, label='Eslabones')
        self.scatter_joints, = self.ax.plot([], [], color='black', marker='o', 
                                            linestyle='None', markersize=6, label='Articulaciones')
        self.scatter_tcp, = self.ax.plot([], [], color='red', marker='X', 
                                         linestyle='None', markersize=10, label='TCP Dinámico')
        
        # Marcador fijo del origen (MCS)
        self.ax.plot(0, 0, marker='+', color='red', markersize=12)
        self.ax.legend(loc="upper right")

    def draw_robot_lateral(self, cartesian_state: np.ndarray) -> None:
        """
        Actualiza las propiedades de los objetos 2D extrayendo los ejes X y Z 
        de la matriz de estado espacial (6, 3).
        """
        if cartesian_state.shape != (6, 3):
            raise ValueError("Dimensión de matriz incorrecta. Se requiere (6, 3).")

        # Extracción de coordenadas del plano de trabajo
        X_wire = cartesian_state[:, 0]
        Z_wire = cartesian_state[:, 2]

        # Mutación de datos en memoria gráfica
        self.line_wireframe.set_data(X_wire, Z_wire)
        self.scatter_joints.set_data(X_wire[:5], Z_wire[:5])
        self.scatter_tcp.set_data(X_wire[5:6], Z_wire[5:6])

        # Refresco de buffer
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


