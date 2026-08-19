#!/usr/bin/env python3
"""
Punto de entrada principal de la aplicación.
Ensambla el controlador del hardware con la interfaz gráfica de usuario.
"""

import sys
import logging
import queue
from logging.handlers import QueueHandler, QueueListener
from PySide6.QtWidgets import QApplication

# Importaciones de los módulos del proyecto
from control.robot_controller import RobotController
from utils.qt_log_handler import QtLogHandler
from app.RobotGUIApp import RobotGUIApp

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

    # Configuracion de Logging centralizada.
    # La consola (stderr) usa un StreamHandler estandar, que hace write()+flush() SINCRONO en
    # el hilo que llama a logger.X(...). Si la terminal que la muestra (ej. la integrada de
    # VSCode) no puede seguir el ritmo de renderizado, ese write() se bloquea y con el bloquea
    # el hilo llamador (GUI o RX) — con nivel DEBUG y trafico continuo esto es exactamente el
    # tipo de estancamiento que se observo. Se delega la escritura real a un hilo dedicado via
    # QueueListener, y se baja el nivel de consola a INFO (el detalle DEBUG sigue disponible en
    # el archivo de log y en el visor de la GUI).
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter(
        "[%(asctime)s] | [%(levelname)s] | [%(module)s] | %(message)s", datefmt="%H:%M:%S"
    ))

    log_queue: "queue.Queue" = queue.Queue(-1)
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    root_logger.addHandler(QueueHandler(log_queue))

    console_listener = QueueListener(log_queue, console_handler, respect_handler_level=True)
    console_listener.start()

    # 2. Configurar el puente de Qt y añadirlo al root logger.
    # QtLogHandler.emit() no hace I/O bloqueante (solo emite una señal Qt: encolada si se llama
    # desde otro hilo, directa pero barata si se llama desde el propio hilo de la GUI), asi que
    # no necesita pasar por la cola de arriba.
    qt_logger = QtLogHandler()
    qt_logger.setFormatter(logging.Formatter('%(message)s'))
    root_logger.addHandler(qt_logger)

    logging.info("Inicializando Robot Control GUI...")

    # 3. Instanciar el Controlador de Hardware
    TARGET_PORT = "/dev/ttyUSB0" #"/dev/ttyUSB0"  # Adaptar según el SO (ej. COM3 en Windows)
    BAUD_RATE = 115200
    CHUNK_SIZE = 32
    LOG_LEVEL = logging.DEBUG
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