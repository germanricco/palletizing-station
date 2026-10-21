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
from control.trajectory_streamer import TrajectoryStreamer, StreamState

class SystemState(Enum):
    BOOT = "BOOT"
    INIT = "INIT"
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    RECORDING = "RECORDING"
    AUTO_READY = "AUTO_READY"
    AUTO_WAITING_SENSOR = "AUTO_WAITING_SENSOR"
    AUTO_RUNNING = "AUTO_RUNNING"
    AUTO_PAUSED = "AUTO_PAUSED"
    FAULT = "FAULT"
    EMERGENCY = "EMERGENCY"


# Mapeo de los nombres de estado tal como los serializa el firmware (Controller::toString en
# StationController.cpp, usado en EVT:STATE,actual,previo) a los miembros de SystemState. Los
# nombres no coinciden 1:1 por convencion de mayusculas/guiones bajos (ej. "AutoReady" en
# firmware vs. SystemState.AUTO_READY aca).
_FIRMWARE_STATE_NAMES = {
    "Boot": SystemState.BOOT,
    "Init": SystemState.INIT,
    "Idle": SystemState.IDLE,
    "Manual": SystemState.MANUAL,
    "Recording": SystemState.RECORDING,
    "AutoReady": SystemState.AUTO_READY,
    "AutoRunning": SystemState.AUTO_RUNNING,
    "AutoPaused": SystemState.AUTO_PAUSED,
    "Fault": SystemState.FAULT,
    "Emergency": SystemState.EMERGENCY,
}


class RobotController:
    """
    Controlador Maestro del sistema robotico.
    """
    # ==== INICIALIZACION Y CICLO DE VIDA ====
    def __init__(self, port: str, baudrate: int = 115200, chunk_size: int = 32, log_level: int = logging.INFO):
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

        # Ultimo progreso conocido de trayectoria/simulacion. Se propaga tambien en los frames de
        # telemetria pura (ver _handle_telemetry) para no pisarlo con 0 en cada uno de ellos.
        self._current_progress: int = 0

        # Ultimos angulos de telemetria procesados por la cinematica. Con el robot quieto la
        # telemetria repite los mismos valores a 50 Hz: no hace falta recalcular la cinematica
        # directa (~0.7 ms con el GIL tomado) en cada trama.
        self._last_telemetry_joints: Optional[tuple] = None

        # Gestor de Recetas (Palletizado)
        self.active_recipe: List[str] = []      # Lista de archivos de trayectorias en orden
        self.current_recipe_step: int = 0       # Indice de la caja actual (0 = Caja A)
        self.pallets_completed: int = 0         # Contador global de ciclos completados

        # Ciclo automatico disparado por el sensor de fin de cinta. Funciona igual con una receta
        # o con una trayectoria unica (receta de 1 paso que se repite en cada caja).
        self._cycle_active: bool = False        # True entre INICIAR CICLO y su fin (stop/cancel/fault)
        # Serializa el ciclo entre el hilo RX (EVT:SENSOR/AUTO_COMPLETE/ERR) y el hilo de la GUI
        # (INICIAR/DETENER). Reentrante: los handlers del RX llaman a stop_palletizing().
        self._cycle_lock = threading.RLock()
        # Comando del ciclo enviado y aun sin ACK ("M120"/"M112"). Permite atribuir un ERR:* del
        # firmware al ciclo (y no a, por ej., un M113 tardio) antes de abortarlo.
        self._pending_cycle_cmd: Optional[str] = None
        self.boxes_completed: int = 0           # Cajas procesadas en modo trayectoria unica
        self.active_trajectory_name: Optional[str] = None   # Nombre de la trayectoria en RAM

        # Estado de la cinta transportadora reportado por el firmware (EVT:CINTA,<estado>)
        self.conveyor_state: str = "DESCONOCIDO"

        # Componentes del sistema
        self.repository = TrajectoryRepository(storage_dir="trajectories", expected_columns=4)
        self.robot = RoboticArmMK2()
        self.uart = AsyncUARTCommunicator(port=port, baudrate=baudrate)
        self.streamer = TrajectoryStreamer(comm=self.uart, chunk_size=chunk_size, precision=4, num_dofs=3)

        self.actuator_active: bool = False

        # Origen del bloqueo actual de FAULT/EMERGENCY ("FIRMWARE" o "LINK_LOST"). Determina
        # como reset_fault() debe recuperarse (ver _enter_lockdown_state).
        self._fault_source: Optional[str] = None

        # Callbacks
        self.uart.set_rx_callback(self._route_serial_message)
        self.uart.set_disconnect_callback(self._on_unexpected_disconnect)
        self.streamer.progress_callback = self._on_streamer_progress
        self.ui_update_callback = None
        self.prompt_save_callback: Optional[Callable[[str], None]] = None
        self.trajectory_loaded_callback = None
        self.conveyor_update_callback: Optional[Callable[[str], None]] = None

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
                self.request_idle_state()
                # Sincroniza estado de la FSM y de la cinta (EVT:STATE + EVT:CINTA). Sin esto la
                # tarjeta de la cinta quedaria en DESCONOCIDO hasta su primer cambio de estado.
                # En FAULT/EMERGENCY no: la salida del bloqueo la decide el operador con RESET
                # (reset_fault), que ya resincroniza via STATUS o M999 segun corresponda.
                if self.current_state not in (SystemState.FAULT, SystemState.EMERGENCY):
                    self._dispatch_command("STATUS", "Sincronizacion inicial de estado")
                return True
            if attempt < max_retries:
                self.logger.debug(f"Fallo en el intento {attempt}. Reintentando en {delay_seconds}s...")
                time.sleep(delay_seconds)

        self.logger.error("Se agotaron los intentos de conexión. Hardware no accesible.")
        self._change_internal_state(SystemState.INIT)
        return False
    
    def finish_connection(self) -> None:
        """
        Cierra la conexión UART de forma segura, aborta operaciones en curso 
        y limpia el estado de las variables volátiles.
        """
        self.logger.info("Solicitud de desconexión. Abortando procesos en curso...")

        # Parada de seguridad a nivel de software
        self.streamer.reset()
        self.recording_buffer.clear()
        self.stop_palletizing()

        # Desconexión (Hardware Sleep)
        if self.is_connected:
            # El firmware solo acepta M98 desde Idle/Manual/AutoReady: si hay una trayectoria
            # en curso o pausada, primero se la saca con M99 (AutoRunning/AutoPaused -> Idle).
            if self.current_state in (SystemState.AUTO_RUNNING, SystemState.AUTO_PAUSED):
                self.uart.send_message("M99")
            self.logger.debug("Enviando comando M98 (Hardware Sleep / Go to INIT)...")
            self.uart.send_message("M98")
            time.sleep(0.1)

        # Desconexión física a través de la capa de comunicación
        self.uart.disconnect()
        self._change_internal_state(SystemState.INIT)
        self._set_conveyor_state("DESCONOCIDO")
        self.logger.info("Desconexión finalizada de manera segura. Sistema Offline.")


    # ==== ENRUTAMIENTO Y PARESO UART (RX) ====
    def _route_serial_message(self, raw_message: str) -> None:
        """
        Enrutador Central. Todo lo que llega del STM32 pasa por este metodo.
        Se ejecuta en el hilo de recepcion (RX Thread) del UART.
        """
        if not raw_message: return

        # Sanitizacion
        clean_msg = raw_message.strip()
        if not clean_msg: return
        
        try:
            # Telemetria
            if clean_msg.startswith("T:"):
                self._handle_telemetry(clean_msg[2:])

            # Eventos Asincronos
            elif clean_msg.startswith("EVT:"):
                self._handle_async_event(clean_msg[4:])

            # Eventos de cambio de estado
            elif clean_msg.startswith("ACK_M"):
                if clean_msg == f"ACK_{self._pending_cycle_cmd}":
                    self._pending_cycle_cmd = None
                self._handle_state_ack(clean_msg)
                if clean_msg == "ACK_M112":
                    self.logger.debug("ACK_M112 Recibido desde firmware. Notificando al Streamer.")
                    self.streamer.process_message(clean_msg)
            
            # Interaccion con el Streamer
            elif clean_msg == "REQ_CHUNK":
                self.logger.debug("REQ_CHUNK Recibido. Delegando al Streamer.")
                self.streamer.process_message(clean_msg)

            # Rechazos del firmware (ej. ERR:RESET_TOO_SOON, ERR:INVALID_STATE). Sin este caso
            # quedaban invisibles (nivel debug generico) y un reset rechazado parecia que el
            # boton "no hacia nada".
            elif clean_msg.startswith("ERR:"):
                self.logger.warning(f"Firmware rechazo el comando: {clean_msg}")
                self._on_firmware_error()

            # Mensaje no reconocido
            else:
                self.logger.debug(f"Mensaje no reconocido del STM32: {clean_msg}")

        except Exception as e:
            self.logger.error(f"Error procesando mensaje RX '{clean_msg}': {e}")

    def _handle_telemetry(self, data_str: str) -> None:
        """
        Procesa la trama de telemetria asincrona a 50Hz.
        Formato de entrada esperado (data_str): "J1,J2,J3,A1"
        """
        try:
            # Extraccion y Validacion estructural de Trama
            partes = data_str.split(',')
            if len(partes) != 4: return

            s1_deg, s2_deg, s3_deg = float(partes[0]), float(partes[1]), float(partes[2])
            is_actuator_on = bool(int(partes[3]))

            # Si estamos simulando salimos sin procesar la telemetria real
            if self._is_simulating: return

            # Entregar datos al modelo cinematico y actualizar estado (solo si cambiaron)
            joints_deg = (s1_deg, s2_deg, s3_deg)
            if joints_deg != self._last_telemetry_joints:
                s1_rad, s2_rad, s3_rad = np.deg2rad(s1_deg), np.deg2rad(s2_deg), np.deg2rad(s3_deg)
                self.robot.update_state(s1_rad, s2_rad, s3_rad)
                self._last_telemetry_joints = joints_deg
            self.actuator_active = is_actuator_on
            kin_array = self.robot.get_angular_state()

            # Persistencia en Memoria (Modo RECORDING). El regreso a Home (disparado por el
            # joystick izquierdo en firmware) se graba como parte normal de la trayectoria, sin
            # ningun tratamiento especial aca: es justamente lo que permite cerrar la trayectoria
            # grabada en el mismo punto en que arranco.
            if self.current_state == SystemState.RECORDING:
                self.recording_buffer.append([kin_array[0], kin_array[1], kin_array[2], float(is_actuator_on)])

            # Propagacion de telemetria hacia la UI (Observer Pattern)
            # Se propaga el ultimo progreso conocido (no 0 a secas) para no pisar el valor real
            # reportado por el streamer y disparar un falso "cambio critico" en cada frame.
            if self.ui_update_callback is not None:
                self.ui_update_callback(
                    self.robot.get_cartesian_state(),
                    np.rad2deg(kin_array).tolist(),
                    self.current_state,
                    self.actuator_active,
                    self._current_progress
                )
            
        except ValueError as ve:
            self.logger.warning(f"Corrupcion de datos en telemetria. '{data_str}': {ve}")

        except Exception as e:
            self.logger.error(f"Fallo critico  en pipeline de telemetria: {e}")

    def _handle_async_event(self, event_msg: str) -> None:
        """
        Enrutador para eventos iniciados por el microcontrolador.

        Parametros:
            event_msg: Codigo de evento de hardware
        """
        if event_msg == "EMERGENCY":
            self.logger.critical("Hardware reporto activacion de parada de emergencia.")
            self._force_emergency_stop("Boton fisico de emergencia presionado.")

        elif event_msg == "FAULT":
            self.logger.critical("Hardware reporto activacion de estado de Falla (FAULT).")
            self._enter_lockdown_state(
                SystemState.FAULT,
                "Falla detectada por firmware (ver EVT:FAULT / logs de FSM).",
                source="FIRMWARE"
            )

        elif event_msg.startswith("STATE,"):
            self._handle_state_event(event_msg[len("STATE,"):])

        elif event_msg == "SENSOR":
            self.logger.info("Interrupcion EVT:SENSOR recibida.")

            # Ejecutamos solo si estamos esperando
            with self._cycle_lock:
                if self.current_state != SystemState.AUTO_WAITING_SENSOR or not self._cycle_active:
                    self.logger.debug("EVT:SENSOR ignorado. Sistema no esta en ciclo de espera.")
                elif self.streamer.state != StreamState.IDLE:
                    # Proteccion ante doble disparo: el estado pasa a AUTO_RUNNING recien con ACK_M112
                    self.logger.debug("EVT:SENSOR ignorado. Ya hay una trayectoria en lanzamiento.")
                else:
                    total = len(self._cycle_sequence())
                    self.logger.info(f"Objeto detectado. Iniciando trayectoria [{self.current_recipe_step + 1}/{total}].")
                    self._pending_cycle_cmd = "M112"
                    if not self.execute_auto():
                        self._pending_cycle_cmd = None
                        self.logger.error("No se pudo lanzar la trayectoria. Deteniendo ciclo automatico.")
                        self.stop_palletizing()

        elif event_msg.startswith("CINTA,"):
            self._set_conveyor_state(event_msg[len("CINTA,"):])

        elif event_msg == "AUTO_COMPLETE":
            # El firmware termino de drenar la FIFO tras confirmar M116 (fin de envio) y
            # transiciono solo AUTO_RUNNING -> AUTO_READY. Esta notificacion es asincrona
            # (no responde a un comando puntual de la PC), por eso llega como evento y no como ACK.
            self.logger.info("Firmware confirmo finalizacion de trayectoria (EVT:AUTO_COMPLETE).")

            if self.current_state != SystemState.AUTO_RUNNING:
                self.logger.debug("EVT:AUTO_COMPLETE ignorado. Sistema no esta en AUTO_RUNNING.")
            else:
                with self._cycle_lock:
                    if self._cycle_active:
                        self.logger.debug("Segmento de palletizado concluido. Avanzando ciclo...")
                        self._advance_recipe()
                    else:
                        self._change_internal_state(SystemState.AUTO_READY)

        else:
            self.logger.warning(f"Evento de hardware no reconocido: {event_msg}")

    def _handle_state_event(self, payload: str) -> None:
        """
        Procesa EVT:STATE,actual,previo (respuesta a STATUS). Sirve para resincronizar
        current_state con el estado real reportado por el firmware -- en particular, es el
        mecanismo de recuperacion tras una perdida de enlace (ver reset_fault/_fault_source),
        donde el firmware probablemente nunca dejo de operar normalmente.

        Parametros:
            payload: contenido tras "STATE," (ej. "Manual,Idle")
        """
        partes = payload.split(',')
        if not partes or not partes[0]:
            self.logger.warning(f"EVT:STATE con payload vacio o mal formado: '{payload}'.")
            return

        actual_state = _FIRMWARE_STATE_NAMES.get(partes[0])
        if actual_state is None:
            self.logger.warning(f"EVT:STATE con estado no reconocido: '{partes[0]}'.")
            return

        self.logger.info(f"Resincronizando estado con el firmware: {actual_state.value}.")

        if actual_state in (SystemState.FAULT, SystemState.EMERGENCY):
            # El bloqueo es real en el firmware: la salida correcta es M999 (ver reset_fault),
            # aunque se haya llegado aca desde una perdida de enlace (LINK_LOST).
            self._fault_source = "FIRMWARE"
            self.streamer.reset()

        self._change_internal_state(actual_state)

        if actual_state == SystemState.AUTO_READY and not self._cycle_active:
            # La PC no tiene un ciclo activo (p. ej. tras una perdida de enlace): la cinta no
            # debe quedar armada en el firmware sin nadie que atienda sus EVT:SENSOR.
            self._dispatch_command("M121", "Desarmar cinta tras resincronizacion")

    def _handle_state_ack(self, ack_msg: str) -> None:
        """
        Mapea los codigos de confirmacion del STM32 a la enumeracion

        Parametros:
            ack_msg: Codigo de confirmacion de comando
        """
        # ACKs que son válidos pero no alteran la máquina de estados.
        # ACK_M116 no cambia de estado aca: la transicion real (AUTO_RUNNING -> AUTO_READY)
        # llega despues, de forma asincrona, como EVT:AUTO_COMPLETE (ver _handle_async_event),
        # porque el firmware puede tardar en drenar la FIFO restante antes de completar.
        # ACK_M120/M121/M122 (cinta) tampoco: el estado de la cinta llega aparte como EVT:CINTA.
        # ACK_M00 llega tras la parada de emergencia, que ya se proceso localmente (EVT:EMERGENCY).
        ignored_state_acks = ["ACK_M00", "ACK_M3", "ACK_M5", "ACK_M116", "ACK_M120", "ACK_M121", "ACK_M122"]
        if ack_msg in ignored_state_acks:
            return

        # ACK_M113 es ambiguo por diseño en el firmware: el mismo codigo confirma tanto una
        # pausa real (AUTO_RUNNING -> AUTO_PAUSED) como una cancelacion (AUTO_PAUSED ->
        # AUTO_READY). El texto del ACK no distingue el caso, asi que usamos el estado actual
        # (el mismo que ya validamos como guarda antes de despachar el comando) para desambiguar.
        if ack_msg == "ACK_M113":
            if self.current_state == SystemState.AUTO_RUNNING:
                new_state = SystemState.AUTO_PAUSED
            elif self.current_state == SystemState.AUTO_PAUSED:
                new_state = SystemState.AUTO_READY
                # Cancelar una trayectoria pausada termina el ciclo automatico (el firmware ya
                # desarmo la cinta al entrar a AutoReady desde AutoPaused). El streamer queda a
                # mitad de envio (STREAMING): sin el reset, la siguiente ejecucion se rechazaria
                # por "streaming en progreso" y el ciclo ignoraria cada EVT:SENSOR posterior.
                self.streamer.reset()
                with self._cycle_lock:
                    if self._cycle_active:
                        self._end_cycle()
                        self._rewind_recipe_view()
            else:
                self.logger.warning(f"ACK_M113 recibido en estado inesperado: {self.current_state.value}.")
                return

            self.logger.debug(f"Confirmacion recibida: {ack_msg}. Sincronizando estado.")
            self._change_internal_state(new_state)
            return

        mapping = {
            "ACK_M98": SystemState.INIT,
            "ACK_M99": SystemState.IDLE,
            "ACK_M100": SystemState.MANUAL,
            "ACK_M101": SystemState.AUTO_READY,
            "ACK_M110": SystemState.RECORDING,       # Inicio de grabacion
            "ACK_M111": SystemState.MANUAL,          # Fin de grabacion
            "ACK_M112": SystemState.AUTO_RUNNING,    # Inicio de trayectoria
            "ACK_M114": SystemState.AUTO_RUNNING,    # Reanudacion de trayectoria
            "ACK_M999": SystemState.INIT,            # Reset controlado tras Fault/Emergency
        }

        new_state = mapping.get(ack_msg)

        if new_state:
            self.logger.debug(f"Confirmacion recibida: {ack_msg}. Sincronizando estado.")
            self._change_internal_state(new_state)
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

        
        # Cualquier salida del modo automatico hacia un estado no-AUTO termina el ciclo (el
        # firmware desarma la cinta por su cuenta al entrar a esos estados).
        if new_state in (SystemState.INIT, SystemState.IDLE, SystemState.MANUAL,
                         SystemState.RECORDING, SystemState.FAULT, SystemState.EMERGENCY):
            self._end_cycle()

        # Acciones de entrada al nuevo estado
        if new_state == SystemState.RECORDING:
            # Limpiar buffer
            self.logger.debug("Inicializando bufer de grabacion.")
            self.recording_buffer.clear()

        elif new_state == SystemState.IDLE:
            # Purgamos la trayectoria si no estamos en una receta de palletizado
            if len(self.active_recipe) == 0:
                if getattr(self, 'active_trajectory', None) is not None:
                    self.logger.debug("Purgando cache de trayectoria automatica.")
                    self.active_trajectory = None
                    self.active_trajectory_name = None

                    if self.trajectory_loaded_callback is not None:
                        self.trajectory_loaded_callback({})

            self.streamer.reset()

        elif new_state == SystemState.AUTO_READY:
            # El progreso de una ejecucion/simulacion anterior no debe persistir al volver a
            # AUTO_READY (por ej. tras cancelar o completar una trayectoria): si quedara en un
            # valor > 0, la UI lo interpreta como "simulando" (ver is_simulating en main_window).
            self._current_progress = 0

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
        # Nota: M113 es ambiguo por diseño en el firmware (pausa desde AUTO_RUNNING, o
        # cancela desde AUTO_PAUSED), asi que se resuelve segun el estado actual.
        state_commands_map = {
            "M99": self.request_idle_state,
            "M100": self.request_manual_mode,
            "M101": self.request_auto_ready,
            "M110": self.start_recording,
            "M111": self.stop_recording,
            "M113": self._dispatch_pause_or_cancel,
            "M114": self.resume_trajectory,
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


    # ==== GESTOR DE ESTACION DE PALLETIZADO ====
    def load_recipe(self, filenames: List[str]) -> bool:
        if self.current_state not in [SystemState.IDLE, SystemState.AUTO_READY, SystemState.MANUAL]:
            self.logger.warning(f"Carga de receta denegada. Incompatibilidad de estado.")
            return False
        
        if not filenames:
            self.logger.warning("Receta vacia proporcionada.")
            return False
        
        self.active_recipe = filenames
        self.current_recipe_step = 0
        self.pallets_completed = 0
        self.logger.info(f"Receta cargada con {len(filenames)} posiciones")

        if self.trajectory_loaded_callback is not None:
             self.trajectory_loaded_callback({
                 'type': 'RECIPE',
                 'count': len(self.active_recipe),
                 'current': 1,
                 'pallets': 0
             })

        return True
    
    def _cycle_sequence(self) -> List[str]:
        """
        Secuencia de trayectorias del ciclo automatico: la receta cargada o, si no hay receta,
        la trayectoria unica en memoria (que se repite en cada caja).
        """
        if self.active_recipe:
            return list(self.active_recipe)
        if self.active_trajectory_name and getattr(self, 'active_trajectory', None) is not None:
            return [self.active_trajectory_name]
        return []

    def start_palletizing(self) -> bool:
        """
        Inicia el ciclo automatico: arma la cinta (M120) y queda a la espera de EVT:SENSOR.
        Cada caja detectada dispara la trayectoria del paso actual (ver _handle_async_event).
        """
        with self._cycle_lock:
            # Validacion de Estado
            if self.current_state != SystemState.AUTO_READY:
                self.logger.warning(f"Fallo al iniciar ciclo. Requiere modo AUTO_READY.")
                return False

            if self._is_simulating:
                self.logger.warning("Fallo al iniciar ciclo. Detenga la simulacion primero.")
                return False

            # Validacion de Trayectoria / Receta
            sequence = self._cycle_sequence()
            if not sequence:
                self.logger.warning("Fallo al iniciar ciclo. No hay trayectoria ni receta cargada.")
                return False

            # Validacion RTH de TODOS los pasos al iniciar (y no recien con una caja frente al
            # robot a mitad de pallet)
            for name in sequence:
                matrix = self.repository.load_trajectory(name)
                if matrix.size == 0:
                    self.logger.error(f"Fallo al iniciar ciclo. No se pudo leer la trayectoria '{name}'.")
                    return False
                if not self._validate_rth(matrix):
                    self.logger.error(f"Fallo al iniciar ciclo. La trayectoria '{name}' no vuelve a Home.")
                    return False

            self.current_recipe_step = 0
            if not self.active_recipe:
                self.boxes_completed = 0

            self.logger.info("Armando ciclo automatico...")
            if not self.load_trajectory(sequence[0]):
                return False

            # El orden importa: primero el estado y despues M120, para que un EVT:SENSOR
            # inmediato (caja ya presente al armar) sea aceptado.
            self._cycle_active = True
            self._change_internal_state(SystemState.AUTO_WAITING_SENSOR)

            self._pending_cycle_cmd = "M120"
            if not self._dispatch_command("M120", "Armar cinta transportadora"):
                self._pending_cycle_cmd = None
                self.stop_palletizing()
                return False
            return True

    def stop_palletizing(self) -> None:
        """
        Detiene el ciclo automatico de forma segura (desarma la cinta) y rebobina la receta.
        Si la trayectoria de la caja actual ya fue lanzada (M112 en vuelo), se deja terminar:
        abortar el streamer ahi dejaria al firmware en AutoRunning sin datos (-> Fault por
        timeout de FIFO). Al completar, EVT:AUTO_COMPLETE vuelve a AUTO_READY (ciclo inactivo).
        """
        with self._cycle_lock:
            was_active = self._cycle_active
            self._end_cycle()
            if was_active:
                self._rewind_recipe_view()

            # Tambien si la cinta quedo armada sin ciclo (p. ej. tras una resincronizacion)
            conveyor_armed = self.conveyor_state in ("AVANZANDO", "ESPERANDO_PICK")
            if (was_active or conveyor_armed) and self.is_connected:
                self._dispatch_command("M121", "Desarmar cinta transportadora")

            if self.current_state == SystemState.AUTO_WAITING_SENSOR:
                if self.streamer.state != StreamState.IDLE:
                    self.logger.info("Trayectoria ya lanzada: se completa la caja en curso y el ciclo termina.")
                else:
                    self._change_internal_state(SystemState.AUTO_READY)

    def _end_cycle(self) -> None:
        """
        Marca el fin del ciclo automatico y rebobina la receta. No envia comandos: cada llamador
        decide si hace falta desarmar la cinta (en Fault/Emergency/Idle el firmware ya lo hace).
        """
        if not self._cycle_active:
            return
        self._cycle_active = False
        self.current_recipe_step = 0
        self.logger.info("Ciclo automatico finalizado. Receta rebobinada.")

    def _rewind_recipe_view(self) -> None:
        """
        Con receta, la RAM y la etiqueta vuelven al paso 0 (si no, SIMULAR previsualizaria el
        ultimo paso ejecutado y la UI mostraria "CAJA n/N" de un ciclo que ya no existe).
        Solo en detenciones pedidas por el usuario: lee de disco, y no debe demorar el hilo RX
        en la cascada de Fault/Emergency.
        """
        if self.active_recipe:
            self._load_trajectory_file(self.active_recipe[0])

    def _on_firmware_error(self) -> None:
        """
        Atiende un ERR:* del firmware solo si responde a un comando del ciclo pendiente de ACK
        (M120 o M112): sin esto la PC quedaria esperando una caja o un ACK_M112 que nunca llegan.
        Se aborta el ciclo (aunque el usuario ya lo hubiera detenido con la trayectoria en vuelo)
        y se resincroniza con el firmware. Un ERR ajeno (ej. M113 tardio) no afecta al ciclo.
        """
        with self._cycle_lock:
            if self._pending_cycle_cmd is None:
                return
            if self.current_state != SystemState.AUTO_WAITING_SENSOR:
                self._pending_cycle_cmd = None
                return

            self.logger.error(
                f"El firmware rechazo {self._pending_cycle_cmd} del ciclo automatico. "
                "Deteniendo y resincronizando."
            )
            self._pending_cycle_cmd = None
            self.streamer.reset()
            self.stop_palletizing()
            if self.current_state == SystemState.AUTO_WAITING_SENSOR:
                self._change_internal_state(SystemState.AUTO_READY)
            self._dispatch_command("STATUS", "Resincronizacion tras rechazo en ciclo automatico")

    def unload_recipe(self) -> None:
        """
        Llamado explícitamente por la GUI cuando el usuario cierra la receta 
        o presiona un botón de 'Limpiar'.
        """
        if self._cycle_active:
            self.logger.warning("Descarga de memoria denegada. Detenga el ciclo automatico primero.")
            return

        self.logger.info("Descargando receta de palletizado de la memoria.")
        self.active_recipe.clear()
        self.current_recipe_step = 0
        self.pallets_completed = 0
        self.boxes_completed = 0
        self.active_trajectory = None
        self.active_trajectory_name = None
        
        # Avisar a la GUI que ya no hay archivo cargado
        if self.trajectory_loaded_callback is not None:
            self.trajectory_loaded_callback({})

    def _advance_recipe(self) -> None:
        """
        Lógica interna de avance del ciclo al completar una trayectoria.
        Con receta avanza al siguiente paso; con trayectoria unica la reutiliza.
        """
        # Modo trayectoria unica: la matriz ya esta en RAM, solo se cuenta la caja
        if not self.active_recipe:
            self.boxes_completed += 1
            self.logger.info(f"Caja completada. Total: {self.boxes_completed}.")

            self._current_progress = 0
            if self.trajectory_loaded_callback is not None:
                self.trajectory_loaded_callback({
                    'type': 'SINGLE',
                    'name': self.active_trajectory_name or "",
                    'boxes': self.boxes_completed
                })
            self._change_internal_state(SystemState.AUTO_WAITING_SENSOR)
            return

        self.current_recipe_step += 1
        
        # Se completó el pallet?
        if self.current_recipe_step >= len(self.active_recipe):
            self.pallets_completed += 1
            self.current_recipe_step = 0
            self.logger.info(f"¡PALLET COMPLETADO! Total Pallets Listos: {self.pallets_completed}")
        
        # Cargar siguiente caja en el buffer local
        next_trajectory = self.active_recipe[self.current_recipe_step]
        self.logger.debug(f"Precargando siguiente bloque en RAM: '{next_trajectory}'")
        
        # Carga directa (sin pasar por un AUTO_READY transitorio, que abriria una ventana para
        # comandos de la GUI a mitad del ciclo)
        self._current_progress = 0
        if self._load_trajectory_file(next_trajectory):
            # Quedamos a la espera del próximo evento de barrera
            self._change_internal_state(SystemState.AUTO_WAITING_SENSOR)
        else:
            self.logger.error("Fallo irrecuperable al precargar. Abortando pallet.")
            self._change_internal_state(SystemState.AUTO_READY)
            self.stop_palletizing()

    # ==== GESTION DE TRAYECTORIAS Y EJECUCION ====
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

    def load_single_trajectory(self, trajectory_name: str) -> bool:
        """
        Wrapper para cargas de trayectoria única solicitadas desde la UI.
        Garantiza que cualquier receta residual en memoria sea purgada.
        """
        if len(self.active_recipe) > 0:
            self.logger.debug("Purgando receta activa por carga de trayectoria manual.")
            self.active_recipe.clear()

        self.boxes_completed = 0
            
        # Llama al método base para cargar en RAM y notificar a la UI
        return self.load_trajectory(trajectory_name)
    
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

        return self._load_trajectory_file(trajectory_name)

    def _load_trajectory_file(self, trajectory_name: str) -> bool:
        """
        Carga efectiva en RAM y notificacion a la UI, sin guarda de estado. Uso interno del
        ciclo automatico (avance de receta y rebobinado), que ya garantiza la coherencia.
        """
        # Interaccion con la capa de persistencia
        matrix = self.repository.load_trajectory(trajectory_name)
        if matrix.size == 0:
            self.logger.error(f"Fallo de E/S: Imposible cargar la trayectoria: '{trajectory_name}'.")
            return False
        
        # Almacenamiento en cache local del orquestador
        self.active_trajectory = matrix
        self.active_trajectory_name = trajectory_name
        self.logger.info(f"Trayectoria '{trajectory_name}' cargada exitosamente. Cant. de puntos: {self.active_trajectory.shape[0]}")

        # Invocacion de callback hacia UI para notificar que la trayectorai esta cargada.
        if self.trajectory_loaded_callback is not None:
            if len(self.active_recipe) > 0:
                # Estamos en modo receta, actualizamos el status completo
                self.trajectory_loaded_callback({
                    'type': 'RECIPE',
                    'count': len(self.active_recipe),
                    'current': self.current_recipe_step + 1,
                    'pallets': self.pallets_completed
                })
            else:
                # Modo normal
                self.trajectory_loaded_callback({
                    'type': 'SINGLE',
                    'name': trajectory_name,
                    'boxes': self.boxes_completed
                })

        return True
    
    def _validate_rth(self, matrix: np.ndarray) -> bool:
        """
        Validación por software "Return-To-Home" (RTH) Obligatorio.
        Compara el inicio y el fin de la trayectoria para asegurar bucle cerrado.
        """
        if matrix.shape[0] < 2: return False
            
        inicio = matrix[0, :3]
        fin = matrix[-1, :3]
        
        # Tolerancia geométrica de ~2.0 grados en radianes (0.035 rad)
        # Permite imprecisiones mínimas del joystick al soltarlo en Home
        tolerancia = np.deg2rad(2.0)
        
        if not np.allclose(inicio, fin, atol=tolerancia):
            self.logger.error(
                "¡PREVENCIÓN DE RIESGO! La trayectoria no cumple el requisito "
                "de 'Return-To-Home'. El punto inicial y final difieren. "
                "Ejecución bloqueada por seguridad."
            )
            return False
        return True

    def execute_auto(self) -> bool:
        """
        Delega la ejecución de la matriz cacheada al TrajectoryStreamer.
        """
        # Validacion de estado simulacion
        if self._is_simulating:
            self.logger.debug("Abortando simulación para iniciar ejecución real.")
            self._is_simulating = False
            time.sleep(0.05) # Pequeño margen para que muera el hilo

        # Validacion de estados validos
        if self.current_state not in [SystemState.AUTO_READY, SystemState.AUTO_WAITING_SENSOR]:
            self.logger.warning(f"Ejecucion denegada. Incompatibilidad de estado.")
            return False

        # Validacion de trayectoria
        if getattr(self, 'active_trajectory', None) is None or self.active_trajectory.size == 0:
            self.logger.warning("Ejecución denegada. No existe matriz de trayectoria cargada en memoria.")
            return False
        
        # Validacion de Return-To-Home (RTH)
        if not self._validate_rth(self.active_trajectory):
            return False

        # Transformacion cinematica desde espacio cinemático a espacio de servos
        hardware_trajectory = np.copy(self.active_trajectory)

        try:
            q_kin_matrix = hardware_trajectory[:, :3]
            q_servo_matrix = self.robot.map_kinematic_trajectory_to_servos(q_kin_matrix)  
            # Reinyectamos los valores transformados
            hardware_trajectory[:, :3] = q_servo_matrix

        except Exception as e:
            self.logger.error(f"Fallo critico en la transformacion cinematica: {e}")
            return False

        self.logger.debug("Delegando envio de trayectoria al TrajectoryStreamer.")
        return self.streamer.start_streaming(hardware_trajectory)

    def stop_trajectory(self) -> bool:
        """
        Solicita la pausa inmediata de la ejecucion.
        El STM32 debe frenar los motores y dejar de solicitar nuevos puntos
        """
        if self.current_state != SystemState.AUTO_RUNNING:
            self.logger.warning(f"No se puede pausar: El sistema no esta en ejecucion.")
            return False

        return self._dispatch_command("M113", "Feed Hold / Pause")

    def resume_trajectory(self) -> bool:
        """
        Solicita reanudar la ejecucion desde el punto exacto de interrupcion.
        """
        if self.current_state != SystemState.AUTO_PAUSED:
            self.logger.warning(f"No se puede reanudar: El sistema no esta pausado.")
            return False

        return self._dispatch_command("M114", "Resume Trajectory")
    
    def cancel_trajectory(self) -> bool:
        """
        Cancela la trayectoria pausada y limpia los buffers
        """
        if self.current_state != SystemState.AUTO_PAUSED:
            self.logger.warning(f"No se puede cancelar: El sistema no esta pausado.")
            return False
        
        return self._dispatch_command("M113", "Cancel Trajectory")

    def _dispatch_pause_or_cancel(self) -> bool:
        """
        Resuelve el comando manual 'M113' segun el estado actual, ya que el firmware
        reutiliza el mismo codigo para pausar (desde AUTO_RUNNING) o cancelar (desde
        AUTO_PAUSED). Uso exclusivo del enrutador de comandos de texto libre.
        """
        if self.current_state == SystemState.AUTO_RUNNING:
            return self.stop_trajectory()
        if self.current_state == SystemState.AUTO_PAUSED:
            return self.cancel_trajectory()

        self.logger.warning(f"Transicion M113 denegada. Incompatibilidad de estado.")
        return False


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

    def toggle_conveyor(self) -> bool:
        """
        Marcha/parada manual de la cinta (M122). Solo fuera del ciclo automatico (IDLE/MANUAL):
        sirve para probar, calibrar o despejar la cinta. El firmware igual la frena al detectar.

        Retorno:
            bool: True si el comando fue encolado, False en caso de error o bloqueo de seguridad.
        """
        # Condición de Guarda
        if self.current_state not in [SystemState.IDLE, SystemState.MANUAL]:
            self.logger.warning(
                "Marcha manual de cinta denegada. "
                f"El estado '{self.current_state.value}' no permite control manual."
            )
            return False

        if self.conveyor_state == "MANUAL":
            return self._dispatch_command("M122 S0", "Detener cinta (marcha manual)")
        return self._dispatch_command("M122", "Marcha manual de cinta")

    def _set_conveyor_state(self, state: str) -> None:
        """
        Actualiza el estado de la cinta reportado por el firmware y notifica a la UI.
        """
        state = state.strip()
        if state != self.conveyor_state:
            self.logger.info(f"Cinta: {state}.")
        self.conveyor_state = state

        if self.conveyor_update_callback is not None:
            self.conveyor_update_callback(state)

    
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

                # Conversion a grados para la UI
                joints_rad = np.array([q1, q2, q3], dtype=np.float32)
                joints_degrees = np.rad2deg(joints_rad).tolist()

                # Notificamos a la UI inyectando las posiciones simuladas
                if self.ui_update_callback:
                    self.ui_update_callback(
                        self.robot.get_cartesian_state(),
                        joints_degrees,
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
            # El modelo cinematico quedo con la pose simulada: forzar que la proxima trama real
            # recalcule aunque traiga los mismos angulos que antes de simular. Se invalida ANTES
            # de bajar la bandera, para que ninguna trama intermedia reuse la pose simulada.
            self._last_telemetry_joints = None
            self._is_simulating = False

    def stop_simulation(self):
        """
        Detiene el hilo de simulación y limpia la UI.
        """
        self._last_telemetry_joints = None   # ver _simulation_worker (pose simulada en el modelo)
        self._is_simulating = False
        self.logger.info("Simulación detenida por el usuario.")
        # Enviamos un último callback con progreso 0 para limpiar la barra en la UI
        if self.ui_update_callback:
            self.ui_update_callback(
                self.robot.get_cartesian_state(),
                np.rad2deg(self.robot.get_angular_state()).tolist(),
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
        Bloquea el orquestador, purga operaciones y notifica la anomalia.
        """
        self._enter_lockdown_state(SystemState.EMERGENCY, reason, source="FIRMWARE")

    def _enter_lockdown_state(self, target_state: SystemState, reason: str, source: str) -> None:
        """
        Cascada comun de bloqueo de seguridad (FAULT/EMERGENCY): loguea, cambia de estado,
        aborta cualquier streaming en curso y purga el buffer de grabacion.

        Parametros:
            target_state: SystemState.FAULT o SystemState.EMERGENCY
            reason: motivo legible para el log
            source: "FIRMWARE" si el firmware realmente esta en ese estado (la recuperacion
                    correcta es M999, ver reset_fault), o "LINK_LOST" si es la PC la que perdio
                    el enlace y el firmware probablemente sigue operando con normalidad (la
                    recuperacion correcta es resincronizar via STATUS, no M999).
        """
        label = "EMERGENCIA" if target_state == SystemState.EMERGENCY else "FALLA"
        self.logger.critical(f"SISTEMA BLOQUEADO ({label}). Motivo: {reason}")

        self._fault_source = source

        # Bloqueo de maquina de Estados
        if self.current_state != target_state:
            self._change_internal_state(target_state)

        # Cascada de Abortos: TrajectoryStreamer
        self.logger.warning("Abortando inyeccion asincrona de trayectoria")
        self.streamer.reset()

        # Cascada de Aborto: Datos Volatiles
        if len(self.recording_buffer) > 0:
            self.logger.warning("Purgando bufer de grabacion. Datos descartados.")
            self.recording_buffer.clear()

    def reset_fault(self) -> bool:
        """
        Solicita la salida controlada de FAULT/EMERGENCY. El mecanismo depende de como se
        llego al bloqueo (ver self._fault_source):
          - "FIRMWARE": el firmware esta realmente en Fault/Emergency -> se envia M999 y se
            espera ACK_M999 (ver _handle_state_ack) antes de destrabar la UI. Nunca se cambia
            current_state de forma optimista aca: si el ACK se pierde, la UI debe quedar
            correctamente trabada en vez de destrabar una falla que puede seguir activa.
          - "LINK_LOST": la PC perdio el enlace, pero el firmware probablemente nunca dejo de
            operar con normalidad -> M999 no tiene sentido. Si ya se reconecto, se pide STATUS
            para resincronizar current_state con lo que el firmware realmente reporta.

        Retorno:
            bool: True si se despacho algun comando, False si fue denegado por guarda.
        """
        if self.current_state not in (SystemState.FAULT, SystemState.EMERGENCY):
            self.logger.warning("Reset denegado. El sistema no esta en FAULT/EMERGENCY.")
            return False

        if self._fault_source == "LINK_LOST":
            if not self.is_connected:
                self.logger.warning("Reconecte el enlace serial antes de intentar el reset.")
                return False
            self.logger.info("Enlace restablecido. Resincronizando estado real con el firmware...")
            return self._dispatch_command("STATUS", "Resincronizacion tras perdida de enlace")

        self.logger.warning("Solicitando reset de falla/emergencia (M999)...")
        return self._dispatch_command("M999", "Reset tras Fault/Emergency")

    def _on_unexpected_disconnect(self) -> None:
        """
        Invocado (desde el hilo de RX o TX de AsyncUARTCommunicator) cuando el enlace serial se
        cae de forma NO deliberada. La PC pierde toda visibilidad real del robot -> se bloquea
        la UI igual que un Fault, pero con origen "LINK_LOST" para que reset_fault() sepa que
        la recuperacion es resincronizar, no enviar M999.
        """
        self._enter_lockdown_state(
            SystemState.FAULT,
            "Conexion con el firmware perdida inesperadamente.",
            source="LINK_LOST"
        )
        self._set_conveyor_state("DESCONOCIDO")


    # ==== METODOS AUXILIARES ====
    def _on_streamer_progress(self, progress: int) -> None:
        self._current_progress = progress
        if self.ui_update_callback:
            self.ui_update_callback(
                self.robot.get_cartesian_state(),
                np.rad2deg(self.robot.get_angular_state()).tolist(),   # grados, como la telemetria
                self.current_state,
                self.actuator_active,
                progress
            )