import logging
import threading
import time
import numpy as np
from enum import Enum
from typing import Callable, Optional, List

from utils.system_logger import SystemLogger
from utils.trajectory_repository import TrajectoryRepository
from control.robot_EEZYbotMK2 import RoboticArmMK2
from communication.async_uart import AsyncUARTCommunicator
from control.trajectory_streamer import TrajectoryStreamer

class SystemState(Enum):
    BOOT = "BOOT"
    INIT = "INIT"
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    RECORDING = "RECORDING"
    AUTO_READY = "AUTO_READY"
    AUTO_RUNNING = "AUTO_RUNNING"
    AUTO_PAUSED = "AUTO_PAUSED"
    FAULT = "FAULT"
    EMERGENCY = "EMERGENCY"


class RobotController:
    """
    Controlador Maestro del sistema robotico.
    """
    # ==== INICIALIZACION Y CICLO DE VIDA ====
    def __init__(self, port: str, baudrate: int = 115200, chunk_size: int = 15, log_level: int = logging.INFO):
        """
        Instancia y ensambla todos los subsistemas del controlador

        Parametros:
            * port: Puerto UART a utilizar
            * baudrate: Velocidad de transmision en baudios
            * chunk_size: Tamaño de los paquetes de transmision
            * log_level: Nivel de logging
        """
        # Instancia de SystemLogger
        self.system_logger = SystemLogger(log_dir="logs", log_level=log_level)
        self.logger = logging.getLogger("STM32_Controller.Orquestador")
        self.logger.info("Iniciando secuencia de arranque de RobotController...")

        # Estado inicial del sistema
        self.current_state: SystemState= SystemState.INIT
        self.recording_buffer: List[List[float]] = []   # Buffer dinamico en RAM para modo RECORDING
        self._pending_save_matrix: Optional[np.ndarray] = None  # Cache para posponer el guardado de trayectoria

        # Componentes
        self.repository = TrajectoryRepository(storage_dir="trajectories", expected_columns=4)
        self.robot = RoboticArmMK2()
        self.uart = AsyncUARTCommunicator(port=port, baudrate=baudrate)
        self.streamer = TrajectoryStreamer(comm=self.uart, chunk_size=chunk_size, precision=4, num_dofs=3)

        # Callback para la recepcion de mensajes serial
        self.uart.set_rx_callback(self._route_serial_message)

        # Estado de actuador
        self.actuator_active: bool = False

        # Callbacks
        self.uart.set_rx_callback(self._route_serial_message)
        self.streamer.progress_callback = self._on_streamer_progress
        self.ui_update_callback = None
        self.prompt_save_callback: Optional[Callable[[str], None]] = None

        # Variables para simulacion
        self.active_trajectory = None
        self._is_simulating = False
        self._simulation_thread = None

        self.logger.info("RobotController ensamblado con exito.")
    
    @property
    def is_connected(self) -> bool:
        """
        Propiedad de solo lectura para verificar si el enlace esta activo.
        """
        return self.uart._running and self.uart.serial_conn is not None and self.uart.serial_conn.is_open
    
    def start_connection(self, max_retries: int = 3, delay_seconds: float = 1.0) -> bool:
        """
        Intenta establecer la conexion UART con el microcontrolador

        Parametros:
            max_retries: Numero de intentos antes de abortar.
            delay_seconds: Tiempo de espera entre intentos fallidos.

        Retorno:
            bool: True si la conexion fue exitosa, False si se agotaron los intentos.
        """
        if self.is_connected:
            self.logger.warning("El sistema ya se encuentra conectado.")
            return True
        
        self.logger.info(f"Iniciando proceso de conexion (Max. intentos: {max_retries})...")

        for attempt in range(1, max_retries+1):
            self.logger.debug(f"Intento de conexion {attempt}/{max_retries} en puerto {self.uart.port}.")

            if self.uart.connect():
                self.logger.info("Conexión UART establecida exitosamente.")
                
                # Sincronización del Espejo: Forzamos el estado a IDLE al conectar
                # Esto limpia cualquier estado residual previo.

                self.request_idle_state()
                return True

            if attempt < max_retries:
                self.logger.warning(f"Fallo en el intento {attempt}. Reintentando en {delay_seconds}s...")
                time.sleep(delay_seconds)

        self.logger.error("Se agotaron los intentos de conexión. Hardware no accesible.")
        
        # Si la conexión falla, retrocedemos al estado INIT (semánticamente correcto 
        # dentro de tu Enum para "esperando inicialización del hardware").
        self._change_internal_state(SystemState.INIT)
        return False
    
    def finish_connection(self) -> None:
        """
        Cierra la conexión UART de forma segura, aborta operaciones en curso 
        y limpia el estado de las variables volátiles.
        """
        self.logger.info("Solicitud de desconexión. Abortando procesos en curso...")

        # Parada de seguridad a nivel de software (Evita fugas de memoria y bloqueos lógicos)
        self.streamer.reset()
        self.recording_buffer.clear()

        # Handshake de Desconexión (Hardware Sleep)
        if self.is_connected:
            self.logger.debug("Enviando comando M98 (Hardware Sleep / Go to INIT)...")
            
            # No esperaremos el ACK_M98. Vamos a cortar la conexión inmediatamente.
            self.uart.send_message("M98")
            
            # Retardo bloqueante mínimo (100ms) para permitir que el hilo de TX
            import time
            time.sleep(0.1)

        # Desconexión física a través de la capa de comunicación
        self.uart.disconnect()

        # Forzar el estado local a INIT para reflejar la pérdida de enlace
        self._change_internal_state(SystemState.INIT)
        self.logger.info("Desconexión finalizada de manera segura. Sistema Offline.")


    # ==== ENRUTAMIENTO Y PARESO UART (RX) ====
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

            # Mensaje no reconocido
            else:
                self.logger.debug(f"Mensaje no reconocido del STM32: {raw_message}")

        except Exception as e:
            self.logger.error(f"Error procesando mensaje RX '{raw_message}': {e}")

    def _handle_telemetry(self, data_str: str) -> None:
        """
        Procesa la trama de telemetria asincrona a 50Hz.
        Formato de entrada esperado (data_str): "J1,J2,J3,A1,A1"
        """
        try:
            # Extraccion y Validacion Estructural
            partes = data_str.split(',')

            # Defensa contra truncamiento de trama UART
            if len(partes) != 4:
                self.logger.warning(f"Descarte de Telemetria. Longitud de trama anomala: '{data_str}'.")
                return
            
            # Conversion de tipos
            s1, s2, s3 = float(partes[0]), float(partes[1]), float(partes[2])
            is_actuator_on = bool(int(partes[3]))

            # Si estamos simulando ignoramos la telemetria
            if self._is_simulating:
                return
            
            # Crea vector de estado articular
            servos_array = np.array([s1, s2, s3], dtype=np.float32)
            # Convierte a valores cinematicos
            kin_array = self.robot.servo_to_kinematic(servos_array)

            # Actualizar estado del Digital Twin
            self.robot.update_state(kin_array[0], kin_array[1], kin_array[2])

            # Almacenar estado logico de actuadores en diccionario
            self.actuator_active = is_actuator_on

            # Persistencia en Memoria (Modo RECORDING)
            if self.current_state == SystemState.RECORDING:
                self.recording_buffer.append([
                    kin_array[0], kin_array[1], kin_array[2],
                    float(is_actuator_on)
                ])

            # Propagacion de telemetria hacia la UI (Observer Pattern)
            if self.ui_update_callback is not None:
                # Se asume que callback implementa mecanismos de Thread-Safe
                self.ui_update_callback(
                    self.robot.get_cartesian_state(),
                    kin_array.tolist(),
                    self.current_state,
                    self.actuator_active
                )
            
        except ValueError as ve:
            self.logger.warning(f"Corrupcion de datos en telemetria. '{data_str}': {ve}")

        except Exception as e:
            self.logger.error(f"Fallo critico no gestionado en pipeline de telemetria: {e}")

    def _handle_async_event(self, event_msg: str) -> None:
        """
        Enrutador para eventos iniciados por el microcontrolador.

        Parametros:
            event_msg: Codigo de evento de hardware
        """
        if event_msg == "EMERGENCY":
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
        Mapea los codigos de confirmacion del STM32 a la enumeracion 
        
        Parametros:
            ack_msg: Codigo de confirmacion de comando
        """
        mapping = {
            "ACK_M98": SystemState.INIT,
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

        # ACKs que son válidos pero no alteran la máquina de estados principal
        ignored_state_acks = ["ACK_M3", "ACK_M5"]

        if ack_msg in ignored_state_acks:
            self.logger.debug(f"Confirmación de actuador recibida: {ack_msg}")
            return

        new_state = mapping.get(ack_msg)

        if new_state:
            self.logger.debug(f"Confirmacion recibida: {ack_msg}. Sincronizando estado.")
            self._change_internal_state(new_state)

            if ack_msg == "ACK_M115":
                self.logger.debug("Trayectoria reanudada. El flujo de REQ_CHUNK se reactivará desde el hardware.")
        else:
            self.logger.warning(f"ACK no reconocido: {ack_msg}.")

    
    # ==== MAQUINA DE ESTADOS (STATE MANAGER) ====
    def _change_internal_state(self, new_state: SystemState) -> None:
        """
        Actualiza el estado actual, gestionando transiciones logicas y notifica a la UI.

        Parametros:
            new_state: Nuevo estado
        """
        if self.current_state == new_state:
            return
        
        # Acciones de salida del estado anterior
        if self.current_state == SystemState.RECORDING and new_state != SystemState.RECORDING:
            self.logger.debug("Finalizando grabacion. Procesando datos.")

            if len(self.recording_buffer) > 0:
                # 1. Cachamos la matriz en RAM
                self._pending_save_matrix = np.array(self.recording_buffer, dtype=np.float32)
                default_filename = f"record_{int(time.time())}"

                # 2. Solicitamos a la UI que pregunte que hacer
                if self.prompt_save_callback is not None:
                    self.prompt_save_callback(default_filename)
                else:
                    # Si la UI se desconecto que se guarde por defecto
                    self.confirm_save_trajectory(default_filename, save=True)
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

                if self.trajectory_loaded_callback is not None:
                    self.trajectory_loaded_callback("")

            self.streamer.reset()

        # Cambio de estado efectivo
        self.current_state = new_state
        self.logger.info(f"Sistema en modo: {self.current_state.value}.")


    # ==== COMANDOS DE USUARIO Y JOGGING (TX) ====
    def send_manual_command(self, raw_command: str) -> bool:
        """
        Procesa comandos de texto libre ingresados por el usuario desde GUI.
        Intercepta comandos de estado para mantener el orquestados sincronizado y
        bloques G-Codes arbitrarios si la maquina no esta en un estado seguro.
        """

        # Convierte todo a mayusculas
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
        allowed_states_for_manual_gcode = [SystemState.IDLE, SystemState.MANUAL,SystemState.AUTO_READY]
        
        if self.current_state not in allowed_states_for_manual_gcode:
            self.logger.warning(
                f"Comando arbitrario '{cmd_full}' denegado. "
                f"El estado actual ({self.current_state.value}) no admite inyección directa."
            )
            return False

        # 4. Despacho seguro al hardware
        return self._dispatch_command(cmd_full, "Inyección manual desde GUI")
    
    def execute_jog(self, mode: str, axis: int, step: float) -> bool:
        """
        Enruta la solicitud de movimiento iterativo de la UI al hardware o cinamatica
        """
        if self.current_state != SystemState.AUTO_READY:
            self.logger.warning("Jogging denegado. El sistema debe estar en AUTO_READY.")
            return False
        
        # TODO: Implementar lógica de cinemática inversa si mode == "CARTESIANO"
        # y generar la trama G-Code correspondiente para inyectar en _dispatch_command
        self.logger.info(f"Comando Jogging Recibido: Modo={mode}, Eje={axis}, Paso={step}")
        return True

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
        if not self.is_connected:
            self.logger.error(f"Imposible enviar comando. El sistema esta desconectado.")
            return False
        
        self.logger.debug(f"Condicion de guarda superada, enviando comando '{cmd}' ({action_desc}).")
        transmission_success = self.uart.send_message(cmd)

        if not transmission_success:
            self.logger.error(f"Fallo de comunicacion TX: Buffer saturado al intentar enviar {cmd}.")
            return False
        
        self.logger.debug(f"Comando {cmd} despachado. Aguardando confirmación...")
        return True


    # ==== TRANSICIONES DE MODOS LOGICOS ====
    def request_idle_state(self) -> bool:
        """
        Solicita al microcontrolador la transicion al estado IDLE.
        Inicia la rutina de retorno suave (Soft Homing) en el hardware.

        Retorno:
            bool: True si el comando fue enviado y encolado exitosamente
                  False si la transicion es invalida o el buffer de transmision esta lleno.
        """
        # Evaluacion de la Condicion de Guarda
        valid_states = [SystemState.MANUAL, SystemState.AUTO_READY, SystemState.INIT]

        if self.current_state not in valid_states:
            self.logger.warning(f"Transicion a IDLE denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M99", "Soft Homing / IDLE")

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

    def start_recording(self) -> bool:
        """
        Solicita al microcontrolador la transicion al modo RECORDING
        Verifica que el sistema este operando en modo MANUAL antes de admitir el comando M110.

        Retorno:
            bool: True si el comando fue enviado y encolado exitosamente
                  False si la transicion es invalida o el buffer de transmision esta lleno.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.MANUAL:
            self.logger.warning(f"Transicion a RECORDING denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M110", "Start Recording")

    def stop_recording(self) -> bool:
        """
        Solicita al microcontrolador finalizar la sesion de grabacion actual.
        Verifica que el sistema este operando en modo RECORDGING.

        Retorno:
            bool: True si el comando fue enviado y encolado exitosamente
                  False si la transicion es invalida o el buffer de transmision esta lleno.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.RECORDING:
            self.logger.warning(f"Finalizacion de grabacion denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M111", "Stop Recording")
        
    def request_auto_ready(self) -> bool:
        """
        Solicita la habilitacion del modo de reproduccion automatica.

        Retorno:
            bool: True si el comando fue enviado y encolado exitosamente
                  False si la transicion es invalida o el buffer de transmision esta lleno.
        """
        # Evaluacion de la Condicion de Guarda
        if self.current_state != SystemState.IDLE:
            self.logger.warning(f"Transicion a AUTO_READY denegada. Incompatibilidad de estado.")
            return False
        
        return self._dispatch_command("M101", "Enable Auto Mode")


    # ==== GESTION DE TRAYECTORIAS EN MEMORIA ====
    def confirm_save_trajectory(self, filename: str, save: bool) -> None:
        """
        Invocado por la UI luego de que el usuario toma una decisión en el diálogo.
        O automaticamente por seguridad si la UI se desconecta inesperadamente.
        """
        if not save or self._pending_save_matrix is None:
            self.logger.info("Grabación descartada por el usuario (o matriz vacía).")
            self._pending_save_matrix = None # Liberar RAM
            return
        
        # Si el usuario dejó el campo vacío, usamos un default
        if not filename:
            filename = f"record_{int(time.time())}"

        self.repository.save_trajectory(filename, self._pending_save_matrix)
        self.logger.info(f"Trayectoria '{filename}' guardada exitosamente.")
        self._pending_save_matrix = None # Liberar RAM

    def load_trajectory(self, trajectory_name: str) -> bool:
        """
        Carga de memoria RAM. Requiere validacion de estado

        Retorno:
            bool: True si la carga fue exitosa, False si hubo un fallo
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

        # Invocacion de callback hacia UI para notificar que la trayectorai esta cargada.
        if self.trajectory_loaded_callback is not None:
            # Enviamos el nombre del archivo (puedes enviar también la matriz si quieres dibujar la línea proyectada en rojo)
            self.trajectory_loaded_callback(trajectory_name)

        return True

    def execute_auto(self) -> bool:
        """
        Delega la ejecución de la matriz cacheada al TrajectoryStreamer.
        """
        # Validacion de estado simulacion
        if self._is_simulating:
            self.logger.info("Abortando simulación para iniciar ejecución real.")
            self._is_simulating = False
            time.sleep(0.05) # Pequeño margen para que muera el hilo

        # Validacion de estados validos
        if self.current_state != SystemState.AUTO_READY:
            self.logger.warning(f"Ejecucion denegada. Incompatibilidad de estado.")
            return False

        # Validacion de trayectoria
        if getattr(self, 'active_trajectory', None) is None or self.active_trajectory.size == 0:
            self.logger.warning("Ejecución denegada. No existe matriz de trayectoria cargada en memoria.")
            return False

        self.logger.debug("Delegando envio de trayectoria al TrajectoryStreamer.")
        
        # Copia de seguridad para evitar modificaciones de memoria cache
        hardware_trajectory = np.copy(self.active_trajectory)

        try:
            # Iterar y transformar columnas de posicion (q1, q2, q3)
            for i in range(hardware_trajectory.shape[0]):
                q1_kin = hardware_trajectory[i, 0]
                q2_kin = hardware_trajectory[i, 1]
                q3_kin = hardware_trajectory[i, 2]
                
                # Invocamos la transformación inversa (Ajusta el nombre del método según tu clase)
                s1, s2, s3 = self.robot.kinematic_to_servo(q1_kin, q2_kin, q3_kin)
                
                # Sobreescribimos con los valores crudos para el STM32
                hardware_trajectory[i, 0] = s1
                hardware_trajectory[i, 1] = s2
                hardware_trajectory[i, 2] = s3

        except Exception as e:
            self.logger.error(f"Fallo critico en la transformacion cinematica: {e}")
            return False

        self.logger.debug("Delegando envio de trayectoria al TrajectoryStreamer.")

        self.streamer.start_streaming(hardware_trajectory)
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
    
    # ==== CONTROL DE HERRAMIENTA ====
    def toggle_actuator(self) -> bool:
        """
        Alterna el estado lógico del actuador principal.
        
        Retorno:
            bool: True si el comando fue encolado, False en caso de error o bloqueo de seguridad.
        """
        # Condición de Guarda
        estados_permitidos = [SystemState.IDLE, SystemState.MANUAL, SystemState.AUTO_READY]
        if self.current_state not in estados_permitidos:
            self.logger.warning(
                "Accionamiento de herramienta denegado. "
                f"El estado '{self.current_state.value}' no permite control manual."
            )
            return False

        if not self.is_connected:
            self.logger.warning("Comando denegado: El sistema está desconectado.")
            return False

        if self.actuator_active:
            cmd = "M5"
            action_desc = "Apagar Actuador (Soltar)"
        else:
            cmd = "M3"
            action_desc = "Encender Actuador (Agarrar)"

        self.logger.info(f"Solicitando {action_desc} (Comando {cmd})...")
        
        # Nota: Usamos uart.send_message directamente y no _dispatch_command() 
        # porque M3/M5 no representan una transición de la Máquina de Estados
        transmission_success = self.uart.send_message(cmd)
        
        if not transmission_success:
            self.logger.error(f"Fallo de comunicación: Buffer TX lleno al intentar enviar {cmd}.")
            return False

        return True

        if not self.is_connected:
            self.logger.warning("Comando denegado: El sistema está desconectado.")
            return False

        if self.actuator_active:
            cmd = "M5"
            action_desc = "Apagar Actuador (Soltar)"
        else:
            cmd = "M3"
            action_desc = "Encender Actuador (Agarrar)"

        self.logger.info(f"Solicitando {action_desc} (Comando {cmd})...")
        
        # Nota: Usamos uart.send_message directamente y no _dispatch_command() 
        # porque M3/M5 no representan una transición de la Máquina de Estados
        transmission_success = self.uart.send_message(cmd)
        
        if not transmission_success:
            self.logger.error(f"Fallo de comunicación: Buffer TX lleno al intentar enviar {cmd}.")
            return False

        return True

    
    # ==== SIMULACION DE TRAYECTORIAS ====
    def simulate_trajectory(self) -> bool:
        """
        Inicia el recorrido virtual de la trayectoria en la memoria.
        """
        if self.current_state != SystemState.AUTO_READY:
            self.logger.warning("Simulación denegada: El sistema no está en AUTO_READY.")
            return False

        if getattr(self, 'active_trajectory', None) is None or self.active_trajectory.size == 0:
            self.logger.warning("Simulación denegada: No hay trayectoria cargada.")
            return False

        # Si ya hay una simulación corriendo, la reiniciamos
        if self._is_simulating:
            self._is_simulating = False
            if self._simulation_thread:
                self._simulation_thread.join(timeout=0.5)

        self.logger.info("Iniciando previsualización del Gemelo Digital...")
        self._is_simulating = True
        self._simulation_thread = threading.Thread(target=self._simulation_worker, daemon=True)
        self._simulation_thread.start()
        return True

    def _simulation_worker(self) -> None:
        """
        Hilo dedicado a iterar la matriz sin bloquear el orquestador.
        """
        try:
            total_points = len(self.active_trajectory)
            # Asumimos que los datos se grabaron a 50Hz (0.02 segundos por punto)
            dt = 0.02 

            for i, point in enumerate(self.active_trajectory):
                if not self._is_simulating:
                    break

                q1, q2, q3 = point[0], point[1], point[2]
                is_actuator_on = bool(point[3])

                # Asegura que siempre envie al menos 1% de progreso
                progress = max(1, int(((i + 1) / total_points) * 100))

                self.robot.update_state(q1, q2, q3)

                # Notificamos a la UI inyectando las posiciones simuladas
                if self.ui_update_callback:
                    self.ui_update_callback(
                        self.robot.get_cartesian_state(),
                        [q1, q2, q3],
                        self.current_state,
                        is_actuator_on,
                        progress
                    )     
                # Simulamos el paso del tiempo de la grabación
                time.sleep(dt)

            self.logger.info("Previsualización finalizada.")
        
        except Exception as e:
            self.logger.error(f"Error durante simulación: {e}")
        
        finally:
            # Apagamos el flag. En los próximos 20 milisegundos, la telemetría viva del 
            # hardware volverá a entrar, el brazo "saltará" a la realidad, indicando 
            # visualmente que la simulación terminó.
            self._is_simulating = False

    def stop_simulation(self):
        """
        Detiene el hilo de simulación y limpia la UI.
        """
        self._is_simulating = False
        self.logger.info("Simulación detenida por el usuario.")
        # Enviamos un último callback con progreso 0 para limpiar la barra en la UI
        if self.ui_update_callback:
            self.ui_update_callback(
                self.robot.get_cartesian_state(),
                self.robot.get_joint_state(),
                self.current_state,
                self.actuator_active,
                0 
            )

    # ==== SEGURIDAD Y EMERGENCIAS ====
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


    # ==== METODOS AUXILIARES ====
    def _on_streamer_progress(self, progress: int) -> None:
        if self.ui_update_callback:
            self.ui_update_callback(
                self.robot.get_cartesian_state(),
                self.robot.get_angular_state(),
                self.current_state,
                self.actuator_active,             # AHORA usa el booleano
                progress
            )