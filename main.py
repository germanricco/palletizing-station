#!/usr/bin/env python3
"""
Punto de entrada principal de la aplicación.
Ensambla el controlador del hardware con la interfaz gráfica de usuario.
"""

import sys
import logging
from PySide6.QtWidgets import QApplication

# Importaciones de los módulos del proyecto
from control.robot_controller import RobotController
from utils.qt_log_handler import QtLogHandler
from app.RobotGUIApp import RobotGUIApp  # Tu archivo modificado arriba

def graceful_shutdown(controller: RobotController) -> None:
    """
    Rutina de cierre seguro disparada cuando la ventana principal se destruye.
    Asegura que el microcontrolador detenga los motores y se libere el puerto COM.
    """
    logging.info("Señal de cierre detectada. Ejecutando Graceful Shutdown...")
    try:
        # Forzamos la detención inmediata de cualquier cinemática en curso
        controller.trigger_emergency_stop()
        # Desconectamos los hilos de RX/TX y cerramos el puerto serial
        if hasattr(controller, 'uart'):
            controller.uart.disconnect()
        logging.info("Hardware asegurado exitosamente. Saliendo...")

    except Exception as e:
        logging.error(f"Fallo durante la secuencia de cierre: {e}")

def main():
    # 1. Inicializar la aplicación Qt
    app = QApplication(sys.argv)

    # Configuracion de Logging centralizada
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] | [%(levelname)s] | [%(module)s] | %(message)s",
        datefmt="%H:%M:%S"
    )

    # 2. Configurar el puente de Qt y añadirlo al root logger
    qt_logger = QtLogHandler()
    qt_logger.setFormatter(logging.Formatter('%(message)s'))
    
    # Interceptamos todos los logs desde el nivel raíz y los enviamos a Qt
    root_logger = logging.getLogger()
    root_logger.addHandler(qt_logger)

    logging.info("Inicializando Robot Control GUI...")

    # 3. Instanciar el Controlador de Hardware
    TARGET_PORT = "COM6" #"/dev/ttyUSB0"  # Adaptar según el SO (ej. COM3 en Windows)
    BAUD_RATE = 115200
    CHUNK_SIZE = 15
    LOG_LEVEL = logging.INFO
    robot_controller = RobotController(port=TARGET_PORT, baudrate=BAUD_RATE, chunk_size=CHUNK_SIZE, log_level=LOG_LEVEL)

    # 4. Instanciar la Interfaz de Usuario
    gui_app = RobotGUIApp(app, robot_controller)

    # 5. Conexión del puente visual
    # El qt_logger emite de forma segura al LogViewer
    qt_logger.log_emitted.connect(gui_app.window.logs_widget.append_log)

    # 6. Cierre seguroC
    app.aboutToQuit.connect(lambda: graceful_shutdown(robot_controller))

    # 8. Desplegar la ventana y entregar el control al Event Loop
    gui_app.run()

if __name__ == "__main__":
    main()