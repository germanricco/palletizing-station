import numpy as np
import logging
import time
from communication.fake_uart import FakeUARTCommunicator
from control.trajectory_streamer import TrajectoryStreamer

from utils.system_logger import SystemLogger
from utils.trajectory_repository import TrajectoryRepository

# FUNCION PARA GENERAR TRAYECTORIA DE PRUEBA
def generar_trayectoria_dummy(num_puntos: int) -> np.ndarray:
    """Genera una trayectoria senoidal de prueba de Nx3."""
    t = np.linspace(0, 10, num_puntos)
    q1 = np.sin(t) * 90      # Articulación base
    q2 = np.cos(t) * 45      # Hombro
    q3 = np.sin(t * 0.5) * 30 # Codos
    return np.column_stack((q1, q2, q3))

def main():
    # 1. Ejecución única para configurar la raíz del logging.
    SystemLogger(log_dir="logs", log_level=logging.DEBUG)
    
    # 2. Reclamamos el logger para este hilo de orquestación
    logger = logging.getLogger("STM32_Controller.Main")

    # 3. Instanciamos el Mock de Comunicación (Simulando el STM32)
    stm32_mock = FakeUARTCommunicator(consumption_rate_hz=50, chunk_size=15)
    stm32_mock.connect()

    # 4. Instanciamos la Lógica de Dominio (El Streamer)
    streamer = TrajectoryStreamer(comm=stm32_mock, chunk_size=15)
    
    logger.info("Iniciando software de control del brazo robótico...")

    # El constructor queda limpio, inyección cero.
    repository = TrajectoryRepository(storage_dir="trajectories")

    # Simulamos la captura
    trayectoria_capturada = generar_trayectoria_dummy(60)
    
    nombre_archivo = "soldadura_prueba_1"
    logger.info(f"Delegando guardado de la trayectoria: {nombre_archivo}")
    
    # Aquí el control pasa a TrajectoryRepository
    exito = repository.save_trajectory(nombre_archivo, trayectoria_capturada)
    
    if exito:
        logger.info("El flujo de guardado retornó sin errores.")
    else:
        logger.error("Fallo crítico reportado por la capa de persistencia.")

    logger.info("Solicitando carga de matriz para modo Automático...")
    matriz_cargada = repository.load_trajectory("soldadura_prueba_1")
    
    if matriz_cargada.size > 0:
        logger.info("Matriz validada en orquestador. Lista para el Streamer.")
    else:
        logger.error("Abortando transición a M101 por ausencia de datos.")

    logger.info("Iniciando transmisión de trayectoria al STM32...")
    streamer.start_streaming(matriz_cargada)

    print("Hilo Principal: Libre para renderizar GUI o leer otras entradas!\n")
    while streamer.state != "IDLE":
        time.sleep(0.1)

    logger.info("Transmision terminada")
    stm32_mock.disconnect()

if __name__ == "__main__":
    main()