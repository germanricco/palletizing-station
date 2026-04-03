#!/usr/bin/env python3
"""
CLI Tester Asíncrono para RobotController
Dependencias adicionales requeridas: pip install prompt_toolkit
"""

import sys
import os
import time
import asyncio
import logging
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Asumiendo que las clases están disponibles en el PYTHONPATH
from control.robot_controller import RobotController, SystemState

# Configuración del logger principal
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)]
)

logger = logging.getLogger("CLI_Tester")

# Variables para controlar el flujo de información en la consola
_last_ui_update = 0.0
UI_REFRESH_RATE = 1.0  # Estrangulamiento de telemetría a 1Hz

def cli_ui_callback(cartesian_state, current_state: SystemState, actuator_state: dict):
    """
    Observer del RobotController. Muestra el estado del sistema en pantalla,
    limitando la frecuencia de dibujado para no saturar la interfaz.
    """
    global _last_ui_update
    now = time.time()
    
    if (now - _last_ui_update > UI_REFRESH_RATE) or current_state in (SystemState.FAULT, SystemState.EMERGENCY):
        logger.info(
            f"[TELEMETRÍA] Estado: {current_state.value} | "
            f"TCP: {cartesian_state} | "
            f"Actuadores: {actuator_state}"
        )
        _last_ui_update = now

async def run_interactive_cli(controller: RobotController):
    """
    Bucle principal asíncrono para la recepción de comandos.
    """
    session = PromptSession()
    
    # patch_stdout() redirige mágicamente todos los print() de los hilos de AsyncUARTCommunicator
    with patch_stdout():
        logger.info("Consola bidireccional lista. Escriba G-Codes (ej. M100, G28) o 'exit'.")
        
        while True:
            try:
                # Entrada no bloqueante. El Event Loop sigue procesando telemetría
                user_input = await session.prompt_async("RobotCLI> ")
                cmd = user_input.strip().upper()
                
                if not cmd:
                    continue
                    
                if cmd in ['EXIT', 'QUIT', 'Q', 'SALIR']:
                    logger.info("Secuencia de cierre iniciada por el usuario...")
                    break
                    
                # Mapeo de pruebas para comandos rápidos de estado
                elif cmd == 'M99':
                    controller.request_idle_state()
                elif cmd == 'M100':
                    controller.request_manual_mode()
                elif cmd == 'M110':
                    controller.start_recording()
                elif cmd == 'M111':
                    controller.stop_recording()
                elif cmd == 'ESTOP' or cmd == 'M00':
                    controller.trigger_emergency_stop()
                else:
                    # Envío de cualquier otra cadena directamente al buffer TX
                    controller._dispatch_command(cmd, "Inyección manual desde CLI")
                    
            except KeyboardInterrupt:
                logger.warning("Interrupción de teclado (Ctrl+C). Abortando...")
                break
            except EOFError:
                break
            except Exception as e:
                logger.error(f"Excepción en bucle de entrada: {e}")

def shutdown_system(controller: RobotController):
    """
    Apagado coordinado de software y hardware.
    """
    logger.info("Iniciando Graceful Shutdown...")
    try:
        # 1. Parada de emergencia de hardware (Intenta enviar M00)
        controller.trigger_emergency_stop()
        time.sleep(0.1) # Breve margen para que el hilo TX despache el mensaje M00
        
        # 2. Cerrar hilos y puerto mediante la interfaz implementada
        logger.info("Deteniendo hilos UART y cerrando puerto...")
        controller.uart.disconnect()
        
        logger.info("Sistema cerrado de forma segura.")
    except Exception as e:
        logger.error(f"Fallo durante la secuencia de cierre: {e}")

if __name__ == "__main__":
    TARGET_PORT = "/dev/ttyUSB0" 
    BAUD_RATE = 115200
    
    try:
        # 1. Instanciar controlador central
        robot = RobotController(
            port=TARGET_PORT, 
            baudrate=BAUD_RATE, 
            log_level=logging.INFO
        )
        
        # 2. Configurar la reactividad visual
        robot.ui_update_callback = cli_ui_callback
        
        # 3. Conectar el hardware (Levanta los hilos de tu AsyncUARTCommunicator)
        if not robot.uart.connect():
            logger.critical("No se pudo establecer el enlace físico con el STM32. Abortando.")
            sys.exit(1)
            
        # 4. Lanzar la interfaz asíncrona
        asyncio.run(run_interactive_cli(robot))
        
    except Exception as e:
        logger.critical(f"Error fatal inicializando el entorno: {e}")
    finally:
        # 5. Garantizar la liberación de recursos sin procesos zombies
        if 'robot' in locals():
            shutdown_system(robot)
        sys.exit(0)