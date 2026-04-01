import numpy as np
import logging
from utils.system_logger import SystemLogger
from utils.trajectory_repository import TrajectoryRepository

def main():
    # 1. Ejecución única para configurar la raíz del logging.
    # Aquí parametrizamos el nivel a DEBUG para observar todo el flujo de desarrollo.
    SystemLogger(log_dir="logs", log_level=logging.INFO)
    
    # 2. Reclamamos el logger para este hilo de orquestación
    logger = logging.getLogger("STM32_Controller.Main")
    
    logger.info("Iniciando software de control del brazo robótico...")
    
    # Este mensaje ahora se mostrará o se ocultará en todo el sistema 
    # dependiendo del log_level definido arriba.
    logger.debug("Infraestructura de persistencia en red inicializada.")

    # El constructor queda limpio, inyección cero.
    repository = TrajectoryRepository(storage_dir="trajectories")

    # Simulamos la captura
    trayectoria_capturada = np.random.rand(100, 4).astype(np.float32) 
    
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

if __name__ == "__main__":
    main()