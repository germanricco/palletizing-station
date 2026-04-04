import logging
import os
import time
import numpy as np
from enum import Enum
from typing import Callable, Optional, List

# IMPORTAMOS MODULOS
from utils.system_logger import SystemLogger
from utils.trajectory_repository import TrajectoryRepository
from control.robot_EEZYbotMK2 import RoboticArmMK2
from communication.async_uart import AsyncUARTCommunicator
from control.trajectory_streamer import TrajectoryStreamer
#from control.trajectory_streamer import StreamState

class SystemState(Enum):
    """
    Enumeracion de la Maquina de Estados (Tiene que se espejo del STM32)
    """
    BOOT = "BOOT",
    INIT = "INIT",
    IDLE = "IDLE",
    MANUAL = "MANUAL",
    RECORDING = "RECORDING",
    AUTO_READY = "AUTO_READY",
    AUTO_RUNNING = "AUTO_RUNNING",
    AUTO_PAUSED = "AUTO_PAUSED",
    FAULT = "FAULT",
    EMERGENCY = "EMERGENCY"

class RobotController:
    """
    Controlador Maestro del sistema robotico.
    Orquesta la maquina de estados local, sincroniza la telemetria con el gemelo digital,
    gestiona la persistencia y dirige el streaming de trayectorias.
    """
    def __init__(self, port: str, baudrate: int = 115200, chunk_size: int = 15, log_level: int = logging.INFO):
        """
        Instancia y ensambla todos los subsistemas del controlador

        Parametros:

        """
        # Instancia de SystemLogger
        self.system_logger = SystemLogger(log_dir="logs", log_level=log_level)
        self.logger = logging.getLogger("STM32_Controller.Orquestador")
        self.logger.info("Iniciando secuencia de arranque de RobotController...")

        # Sistema inicia asumiendo que el hardware ha completado su boot y esta listo.
        self.current_state: SystemState= SystemState.IDLE

        # Buffer dinamico en RAM para modo RECORDING
        self.recording_buffer: List[List[float]] = []

        # Callback expuesto para la actualización reactiva de la Interfaz de Usuario (UI)
        self.ui_update_callback: Optional[Callable[[np.ndarray, SystemState], None]] = None

        # COMPONENTES
        # Repositorio de trayectorias
        self.logger.debug("Instanciando capa de persistencia (TrajectoryRepository).")
        self.repository = TrajectoryRepository(storage_dir="trajectories")

        # Gemelo Digital
        self.logger.debug("Instanciando Gemelo Digital (RoboticArmMK2).")
        self.digital_twin = RoboticArmMK2()

        # Interfaz de Hardware
        self.logger.debug("Configurando enlace UART en {port} @ {baudrate}")
        self.uart = AsyncUARTCommunicator(port=port, baudrate=baudrate)

        # Gestor de Trayectorias
        self.logger.debug("Instanciando TrajectoryStreamer con chunk_size={chunk_size}.")
        self.streamer = TrajectoryStreamer(comm=self.uart, chunk_size=chunk_size)

        # Recepcion de callbacks del UART.
        self.uart.set_rx_callback(self._route_serial_message)

        # Atributo para rastrear el estado discreto de la herramienta final
        self.actuator_state = {
            'valve': False,
            'pump': False
        }

        # Callback de UI.
        #! ACTUALIZAR FIRMA DE CALLBACK
        self.ui_update_callback = None

        self.logger.info("RobotController ensamblado con exito. Estado actual: {self.current_state.value}.")
    
    
    # GESTION DE EVENTOS. LOS VA DIRIGIENDO DONDE SEAN NECESARIOSss
    def _route_serial_message(self, raw_message: str) -> None:
        """
        Enrutador Central. Todo lo que llega del STM32 pasa por este metodo.
        Se ejecuta en el hilo de recepcion (RX Thread) del UART.
        """
        if not raw_message:
            return
        
        try:
            # Decodificar telemetria (Ej: "T:45.0, 30.0, 15.0, 1, 1")
            if raw_message.startswith("T:"):
                self._handle_telemetry(raw_message[2:])

            elif raw_message.startswith("EVT:"):
                self._handle_async_event(raw_message[4:])

            # Eventos de cambio de estado
            elif raw_message.startswith("ACK_M"):
                self._handle_state_ack(raw_message)

                # Streamer tambien requiere enterarse de confirmacion ACK_M112
                if raw_message == "ACK_M112":
                    self.streamer.process_message(raw_message)

            # Interaccion con el Streamer 
            elif raw_message == "REQ_CHUNK":
                # Si el mensaje pertenece al protocolo de streaming, lo delegamos al TrajectoryStreamer
                self.streamer.process_message(raw_message)

            elif raw_message.startswith("EVT:"):
                pass

            # Mensaje no reconocido
            else:
                self.logger.debug(f"Mensaje no reconocido del STM32: {raw_message}")

        except Exception as e:
            self.logger.error(f"Error procesando mensaje RX '{raw_message}': {e}")

    
    # PROCESA TELEMETRIA A 50HZ Y ACTUALIZA DIGITAL_TWIN + UI
    def _handle_telemetry(self, data_str: str) -> None:
        """
        Procesa la trama de telemetria asincrona a 50Hz.
        Formato de entrada esperado (data_str): "J1,J2,J3,A1,A1"
        """
        try:
            # Extraccion y Validacion Estructural
            partes = data_str.split(',')
            # Defensa contra truncamiento de trama UART
            if len(partes) != 5:
                self.logger.warning(f"Descarte de Telemetria. Longitud de trama anomala: '{data_str}'.")
                return
            
            # Conversion de tipos
            q1, q2, q3 = float(partes[0]), float(partes[1]), float(partes[2])
            a1_valve, a2_pump = bool(int(partes[3])), bool(int(partes[4]))

            # Actualizar estado de Digital Twin
            self.digital_twin.update_state(q1, q2, q3)

            # Almacenar estado logico de actuadores
            self.actuator_state['valve'] = a1_valve
            self.actuator_state['pump'] = a2_pump

            # Persistencia en Memoria (Modo RECORDING)
            if self.current_state == SystemState.RECORDING:
                self.recording_buffer.append([
                    q1, q2, q3,
                    float(a1_valve), float(a2_pump)
                ])

            # Propagacion hacia la UI (Observer Pattern)
            if self.ui_update_callback is not None:
                # Se asume que callback implementa mecanismos de Thread-Safe
                self.ui_update_callback(
                    self.digital_twin.get_cartesian_state(),
                    [q1, q2, q3],
                    self.current_state,
                    self.actuator_state
                )
            
        except ValueError as ve:
            self.logger.warning(f"Corrupcion de datos en telemetria. '{data_str}': {ve}")

        except Exception as e:
            self.logger.error(f"Fallo critico no gestionado en pipeline de telemetria: {e}")


    def _handle_async_event(self, event_msg: str) -> None:
        """
        Enrutador para eventos iniciados por el microcontrolador.
        """
        if event_msg == "ESTOP":
            self.logger.critical("Hardware reporto activacion de parada de emergencia.")
            self._force_emergency_stop("Boton fisico de emergencia presionado.")

        elif event_msg == "COMM_TIMEOUT":
            # Evento hipotetico a implementar
            self.logger.error("Hardware reporto perdida de comunicacion de seguridad.")
            self._change_internal_state(SystemState.FAULT)
        
        else:
            self.logger.warning(f"Evento de hardware no reconocido: {event_msg}")


    def _handle_state_ack(self, ack_msg: str) -> None:
        """
        Mapea los codigos de confirmacion del STM32 a la enumeracion SystemState
        """
        mapping = {
            "ACK_M99": SystemState.IDLE,
            "ACK_M100": SystemState.MANUAL,
            "ACK_M101": SystemState.AUTO_READY,
            "ACK_M110": SystemState.RECORDING,       # Inicio de grabacion
            "ACK_M111": SystemState.MANUAL,          # Fin de grabacion
            "ACK_M112": SystemState.AUTO_RUNNING,    # Inicio de trayectoria
            "ACK_M113": SystemState.AUTO_READY,      # Fin o cancelacion de trayectoria
            "ACK_M114": SystemState.AUTO_PAUSED,     # Pausa de trayectoria
            "ACK_M115": SystemState.AUTO_RUNNING     # Reanudacion de trajectoria
        }

        new_state = mapping.get(ack_msg)

        if new_state:
            self.logger.info(f"Confirmacion recibida: {ack_msg}. Sincronizando estado.")
            self._change_internal_state(new_state)

            if ack_msg == "ACK_M115":
                self.logger.debug("Trayectoria reanudada. Se reactiva REQ_CHUNK del microcontrolador.")
        else:
            self.logger.warning(f"ACK de estado desconocido o no gestionado: {ack_msg}.")

    
    # ACTUALIZA EL ESTADO INTERNO
    def _change_internal_state(self, new_state: SystemState) -> None:
        """
        Actualiza el estado actual, gestionando transiciones logicas y notifica a la UI.
        """
        if self.current_state == new_state:
            return
        
        # Acciones de salida del estado anterior
        if self.current_state == SystemState.RECORDING and new_state != SystemState.RECORDING:
            self.logger.debug("Finalizando sesion de grabacion. Procesando datos.")

            if len(self.recording_buffer) > 0:
                # Se delega el guardado al TrajectoryRepository.
                matrix_to_save = np.array(self.recording_buffer, dtype=np.float32)
                filename = f"record_{int(time.time())}"
                self.repository.save_trajectory(filename, matrix_to_save)
                self.logger.info(f"Trayectoria '{filename}' guardada exitosamente. Shape: {matrix_to_save.shape}.")
            else:
                self.logger.warning("No se recopilaron datos.")

        
        # Acciones de entrada al nuevo estado
        if new_state == SystemState.RECORDING:
            # Limpiar buffer
            self.logger.debug("Inicializando bufer de grabacion.")
            self.recording_buffer.clear()

        elif new_state == SystemState.IDLE:
            # Se garantiza que no queden datos residuales en memoria
            if getattr(self, 'active_trajectory', None) is not None:
                self.logger.debug("Purgando cache de trayectoria automatica.")
                self.active_trajectory = None

            self.streamer.reset()

        # Cambio de estado efectivo
        self.current_state = new_state
        self.logger.info(f"Sistema en modo: {self.current_state.value}.")

        # Notificacion a la Interfaz de Usuario
        if self.ui_update_callback:
            # Se envía el estado cartesiano actual y el nuevo estado de la máquina
            self.ui_update_callback(
                self.digital_twin.get_cartesian_state(), 
                self.current_state,
                self.actuator_state
            )


    # RECIBE UN EVENTO DE ESTADO DEL STM32
    def _handle_state_event(self, event: str) -> None:
        pass


    def send_manual_command(self, raw_command: str) -> bool:
        """
        Procesa comandos de texto libre ingresados por el usuario desde GUI.
        Intercepta comandos de estado para mantener el orquestados sincronizado y
        bloques G-Codes arbitrarios si la maquina no esta en un estado seguro.
        """

        cmd_full = raw_command.strip().upper()
        if not cmd_full:
            return False
        
        self.logger.info(f"Comando recibido desde UI: {cmd_full}")

        # Extraemos el comando base
        base_cmd = cmd_full.split()[0] if " " in cmd_full else cmd_full

        # Intercepcion de Emergencia
        if base_cmd in ["M00", "ESTOP"]:
            self.trigger_emergency_stop()
            return True
        
        # Intercepcion de maquina de estados
        state_commands_map = {
            "M99": self.request_idle_state,
            "M100": self.request_manual_mode,
            "M101": self.request_auto_ready,
            "M110": self.start_recording,
            "M111": self.stop_recording,
            "M113": self.cancel_trajectory,
            "M114": self.stop_trajectory,
            "M115": self.resume_trajectory
        }

        if base_cmd in state_commands_map:
            self.logger.debug(f"Interceptado comando de estado '{base_cmd}'. Enrutando a método interno.")
            # Ejecutamos el método correspondiente y retornamos su resultado
            return state_commands_map[base_cmd]()

        # 3. Comandos Arbitrarios (G-Codes de movimiento, actuadores, etc.)
        # Evaluamos la Condición de Guarda: ¿El estado actual permite inyección manual?
        allowed_states_for_manual_gcode = [SystemState.IDLE, SystemState.MANUAL]
        
        if self.current_state not in allowed_states_for_manual_gcode:
            self.logger.warning(
                f"Comando arbitrario '{cmd_full}' denegado. "
                f"El estado actual ({self.current_state.value}) no admite inyección directa."
            )
            return False

        # 4. Despacho seguro al hardware
        return self._dispatch_command(cmd_full, "Inyección manual desde GUI")

    
    # MANUAL/AUTO_READY -> IDLE
    def request_idle_state(self) -> bool:
        """
        Solicita al microcontrolador la transicion al estado IDLE.
        Inicia la rutina de retorno suave (Soft Homing) en el hardware.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state not in [SystemState.MANUAL, SystemState.AUTO_READY]:
            self.logger.warning(f"Transicion a IDLE denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M99", "Soft Homing / IDLE")

    # IDLE -> MANUAL
    def request_manual_mode(self) -> bool:
        """
        Solicita al microcontrolador la transicion al modo de operacion MANUAL.
        Verifica las reglas de negocio de la maquina de estados antes de emitir el comando M100.

        Retorno:
            bool: True si el comando fue enviado y encolado exitosamente
                  False si la transicion es invalida o el buffer de transmision esta lleno.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.IDLE:
            self.logger.warning(f"Transicion a MANUAL denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M100", "Enable Manual Mode")

    
    # MANUAL -> RECORDING
    def start_recording(self) -> bool:
        """
        Solicita al microcontrolador la transicion al modo RECORDING
        Verifica que el sistema este operando en modo MANUAL antes de admitir el comando M110.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.MANUAL:
            self.logger.warning(f"Transicion a RECORDING denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M110", "Start Recording")


    # RECORDING -> MANUAL (SALIDA CON PERSISTENCIA)
    def stop_recording(self) -> bool:
        """
        Solicita al microcontrolador finalizar la sesion de grabacion actual.
        Verifica que el sistema este operando en modo RECORDGING.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.RECORDING:
            self.logger.warning(f"Finalizacion de grabacion denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M111", "Stop Recording")
        

    def request_auto_ready(self) -> bool:
        """
        Solicita la habilitacion del modo de reproduccion automatica.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.IDLE:
            self.logger.warning(f"Transicion a AUTO_READY denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M101", "Enable Auto Mode")


    # IDLE -> AUTO_READY
    def load_trajectory(self, trajectory_name: str) -> bool:
        """
        Carga de memoria RAM.
        Requiere validacion de estado
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.AUTO_READY:
            self.logger.warning(f"Carga de archivo denegada. Incompatibilidad de estado.")
            return False
        
        # Interaccion con la capa de persistencia
        matrix = self.repository.load_trajectory(trajectory_name)
        if matrix.size == 0:
            self.logger.error(f"Fallo de E/S: Imposible cargar la trayectoria: '{trajectory_name}'.")
            return False
        
        # Almacenamiento en cache local del orquestador
        self.active_trajectory = matrix
        self.logger.info(f"Trayectoria '{trajectory_name}' cargada exitosamente. Cant. de puntos: {self.active_trajectory.shape[0]}")

        #! NOTA. ACA SE PUEDE INVOCAR CALLBACK AUXILIAR
        #! PARA QUE SE RENDERICE LA TRAYECTORIA PROYECTADA EN ROJO
        return True

    
    def execute_auto(self) -> bool:
        """
        Delega la ejecución de la matriz cacheada al TrajectoryStreamer.
        """
        if self.current_state != SystemState.AUTO_READY:
            self.logger.warning(f"Ejecucion denegada. Incompatibilidad de estado.")
            return False

        if getattr(self, 'active_trajectory', None) is None or self.active_trajectory.size == 0:
            self.logger.warning("Ejecución denegada. No existe matriz de trayectoria cargada en memoria.")
            return False

        self.logger.debug("Delegando matriz cargada a TrajectoryStreamer...")
        
        # El TrajectoryStreamer se encarga de inyectar 'M112' en el UART
        self.streamer.start_streaming(self.active_trajectory)
        return True

    
    def stop_trajectory(self) -> bool:
        """
        Solicita la pausa inmediata de la ejecucion.
        El STM32 debe frenar los motores y dejar de solicitar nuevos puntos
        """
        if self.current_state != SystemState.AUTO_RUNNING:
            self.logger.warning(f"No se puede pausar: El sistema no esta en ejecucion.")
            return False
        
        return self._dispatch_command("M114", "Feed Hold / Pause")
    

    def resume_trajectory(self) -> bool:
        """
        Solicita reanudar la ejecucion desde el punto exacto de interrupcion.
        """
        if self.current_state != SystemState.AUTO_PAUSED:
            self.logger.warning(f"No se puede reanudar: El sistema no esta pausado.")
            return False

        return self._dispatch_command("M115", "Resume Trajectory")
    

    def cancel_trajectory(self) -> bool:
        """
        Cancela la trayectoria pausada y limpia los buffers
        """
        if self.current_state != SystemState.AUTO_PAUSED:
            self.logger.warning(f"No se puede cancelar: El sistema no esta pausado.")
            return False
        
        return self._dispatch_command("M113", "Cancel Trajectory")
    

    # GESTION DE SEGURIDAD Y EMERGENCIAS
    def trigger_emergency_stop(self) -> None:
        """
        Inicia la secuencia de Parada de Emergencia desde el Host (UI)
        Emite el comando de maxima prioridad y fuerza el bloqueo sin esperar confirmacion.
        """
        self.logger.info("Emitiendo comando de parada de emergencia de hardware.")

        # Inyeccion del comando a la cola de transmision
        transmission_success = self.uart.send_message("M00")

        if not transmission_success:
            self.logger.error("FALLO CATASTROFICO. Buffer UART colapsado. Imposible emitir M00.")

        # Bloqueo incondicional de la maquina de estados local
        self._force_emergency_stop("Activacion de emergencia por software.")

        
    def _force_emergency_stop(self, reason: str) -> None:
        """
        Bloquea el orquestador, purga operaciones y notifica la anomalia
        """
        self.logger.critical(f"SISTEMA BLOQUEADO. Motivo {reason}")

        # Bloqueo de maquina de Estados
        if self.current_state != SystemState.EMERGENCY:
            self._change_internal_state(SystemState.EMERGENCY)

        # Cascada de Abortos: TrajectoryStreamer
        self.logger.warning("Abortando inyeccion asincrona de trayectoria")
        self.streamer.reset()

        # Cascada de Aborto: Datos Volatiles
        if len(self.recording_buffer) > 0:
            self.logger.warning("Purgando bufer de grabacion. Datos descartados.")
            self.recording_buffer.clear()

        #! NOTA. Para reanudar, se requiere secuencia estricta de reinicio. (ej M99)


    
    #COMANDOS AUXILIARES
    def _dispatch_command(self, cmd: str, action_desc: str = "") -> bool:
        """
        Centraliza el despacho asincrono de comandos UART y la validacion de la
        capa de transporte.
        (PRINCIPIO DRY Dont-Repeat-Yourself)

        Parametros:
            * cmd: Comando a enviar
            * action_desc: Descripcion legible para el registro de logs.

        Retorno:
            * bool: True si se encolo correctamente, False si el buffer fallo
        """
        self.logger.debug(f"Condicion de guarda superada, enviando comando '{cmd}' ({action_desc}).")
        transmission_success = self.uart.send_message(cmd)

        if not transmission_success:
            self.logger.error(f"Fallo de comunicacion TX: Buffer saturado al intentar enviar {cmd}.")
            return False
        
        self.logger.info(f"Comando {cmd} despachado. Aguardando confirmación...")
        return True