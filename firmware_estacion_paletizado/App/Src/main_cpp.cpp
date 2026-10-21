#include "main_cpp.hpp"
#include "Config.hpp"
#include "Joystick.hpp"
#include "Servo.hpp"
#include "MotorDC.hpp"
#include "Actuador.hpp"
#include "TerminalSerial.hpp"
#include "Logger.hpp"
#include "StationController.hpp"
#include "SensorIR.hpp"
#include "Cinta.hpp"

#include "stm32f1xx_hal.h"
#include <cstdio>
#include <cstring>

extern "C" {
    extern ADC_HandleTypeDef hadc1;		// Lectura de Analogicos
    extern TIM_HandleTypeDef htim2;		// Timer para control de motorDC
    extern TIM_HandleTypeDef htim3;		// Timer para control de servos
    extern UART_HandleTypeDef huart1;	// Comunicacion UART
}


// ==== COMPONENTES DEL SISTEMA ====
// Buffer de ADC
volatile uint32_t buffer_global_adc[4];

// Joystick
Joystick joystick_izquierdo(&buffer_global_adc[0], &buffer_global_adc[1],
                            GPIOB, GPIO_PIN_5,
                            Config::JOYSTICK_IZQUIERDO_X_CENTER,
                            Config::JOYSTICK_IZQUIERDO_Y_CENTER,
                            Config::JOYSTICK_DEADZONE);

Joystick joystick_derecho(&buffer_global_adc[2], &buffer_global_adc[3],
                          GPIOB, GPIO_PIN_6,
                          Config::JOYSTICK_DERECHO_X_CENTER,
                          Config::JOYSTICK_DERECHO_Y_CENTER,
                          Config::JOYSTICK_DEADZONE);

// Servos
Servo servo_base(&TIM3->CCR1,
                 Config::SERVO_BASE_MIN_ANGLE,
                 Config::SERVO_BASE_MAX_ANGLE,
                 Config::SERVO_BASE_MIN_PULSE_US,
                 Config::SERVO_BASE_MAX_PULSE_US,
                 Config::SERVO_BASE_HOME_ANGLE,
                 Config::SERVO_BASE_MAX_SPEED,
                 Config::SERVO_BASE_ACCEL_DIVISOR);

Servo servo_hombro(&TIM3->CCR2,
				 Config::SERVO_HOMBRO_MIN_ANGLE,
		         Config::SERVO_HOMBRO_MAX_ANGLE,
		         Config::SERVO_HOMBRO_MIN_PULSE_US,
		         Config::SERVO_HOMBRO_MAX_PULSE_US,
		         Config::SERVO_HOMBRO_HOME_ANGLE,
		         Config::SERVO_HOMBRO_MAX_SPEED,
		         Config::SERVO_HOMBRO_ACCEL_DIVISOR);

Servo servo_codo(&TIM3->CCR3,
				 Config::SERVO_CODO_MIN_ANGLE,
		         Config::SERVO_CODO_MAX_ANGLE,
                 Config::SERVO_CODO_MIN_PULSE_US,
                 Config::SERVO_CODO_MAX_PULSE_US,
		         Config::SERVO_CODO_HOME_ANGLE,
		         Config::SERVO_CODO_MAX_SPEED,
		         Config::SERVO_CODO_ACCEL_DIVISOR);


// MotorDC
MotorDC motor_cinta(&TIM2->CCR3, 49, Config::CINTA_MIN_CCR);

//SensorIR//IMPORTANTE: suponemos que el IR da un nivel LOW cuando detecta por eso RESET
SensorIR sensor_fin_cinta(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET, Config::SENSOR_IR_DEBOUNCE_MS);

// Cinta transportadora (sub-maquina: motor + sensor de fin de cinta)
Cinta cinta_transportadora(motor_cinta, sensor_fin_cinta,
                           Config::CINTA_VELOCIDAD_NOMINAL,
                           Config::CINTA_RAMPA_ARRANQUE_MS);

// Actuador
Actuador efector_final(&TIM2->CCR1, &TIM2->CCR2, 49, 800);

// Terminal
TerminalSerial terminal(&huart1);

// Controlador
Station::Controller stationController(terminal,
									joystick_izquierdo, joystick_derecho,
									servo_base, servo_hombro, servo_codo,
									efector_final, cinta_transportadora);


// ==== INICIALIZACION DEL SISTEMA ====
void Cpp_Main_Init(void) {
	// Terminal
    terminal.init();

    // Logger
    Logger::init(&terminal, Config::DEFAULT_LOG_LEVEL);
    Logger::logINFO("Inicializando sistema...");

    // ADCs
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer_global_adc, 4);
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    // Servos
    servo_base.init();
    servo_hombro.init();
    servo_codo.init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim3);

    // Actuador
    efector_final.init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // MotorDC
    motor_cinta.init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Sensor IR de fin de cinta
    sensor_fin_cinta.init(HAL_GetTick());

    // Cinta transportadora (arranca deshabilitada; la arma la FSM con M120)
    cinta_transportadora.init(HAL_GetTick());

    // Controlador
    stationController.init();

    // Mensaje Inicial
    terminal.enviarNormal(
        "PROTO: M98 M99 M100 M101 M110 M111 M112 M113 M114 M116 M120 M121 M122 M999 M00 STATUS\r\n");

    // Causa del arranque (RCC_CSR). Si la PC estaba conectada, EVT:BOOT le avisa que el
    // firmware se reinicio (p. ej. por el watchdog) y que su estado ya no es valido.
    // El IWDG tambien activa PINRSTF, por eso se evalua primero.
    const uint32_t causaReset = RCC->CSR;
    const char* causa = "OTRO";
    if ((causaReset & RCC_CSR_IWDGRSTF) != 0U)      causa = "WATCHDOG";
    else if ((causaReset & RCC_CSR_SFTRSTF) != 0U)  causa = "SOFTWARE";
    else if ((causaReset & RCC_CSR_PORRSTF) != 0U)  causa = "ENCENDIDO";
    else if ((causaReset & RCC_CSR_PINRSTF) != 0U)  causa = "PIN_RESET";
    RCC->CSR |= RCC_CSR_RMVF;                // Limpia las banderas para el proximo arranque

    char msgBoot[32];
    std::snprintf(msgBoot, sizeof(msgBoot), "EVT:BOOT,%s\r\n", causa);
    terminal.enviarNormal(msgBoot);

    // Watchdog: se arranca al final para que la inicializacion no cuente en su ventana.
    // Secuencia del RM0008: arrancar (fuerza el LSI), configurar y esperar la sincronizacion.
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;   // No resetear al frenar en un breakpoint
    IWDG->KR  = 0xCCCCU;                     // Arranca (no se puede detener por software)
    IWDG->KR  = 0x5555U;                     // Habilita escritura de PR/RLR
    IWDG->PR  = Config::IWDG_PRESCALER_DIV32;
    IWDG->RLR = Config::IWDG_RECARGA;
    // PVU/RVU: unos pocos ciclos de LSI (~150 us). Acotado por si el LSI no arrancara.
    for (uint32_t espera = 0U; (IWDG->SR != 0U) && (espera < 100000U); ++espera) {}
    IWDG->KR  = 0xAAAAU;                     // Recarga con la ventana nueva
}


// ==== LOOP PRINCIPAL DEL SISTEMA ====
void Cpp_Main_Loop(void) {
    const uint32_t tiempo_actual = HAL_GetTick();

	// Filtro decimador (integrate-and-dump): acumula una muestra cruda del ADC en CADA vuelta
	// del loop, sin gate de tasa. Promedia el ruido de banda ancha entre los ~11.9kHz del
	// ADC/DMA y los 100Hz de muestreo real del joystick, evitando que ese ruido haga aliasing
	// al decimar (ver nota en JoystickInput.hpp).
	joystick_izquierdo.AcumularMuestra();
	joystick_derecho.AcumularMuestra();

	// Muestreo y filtrado del joystick a tasa fija (100Hz): promedia lo acumulado arriba y
	// alimenta el filtro EMA, que asi tiene una frecuencia de corte determinista.
    static uint32_t tiempoUltimaMuestraJoystick = 0U;
    if ((tiempo_actual - tiempoUltimaMuestraJoystick) >= Config::JOYSTICK_SAMPLE_PERIOD_MS) {
        tiempoUltimaMuestraJoystick += Config::JOYSTICK_SAMPLE_PERIOD_MS;

        joystick_izquierdo.Actualizar();
        joystick_derecho.Actualizar();
    }

    terminal.update();

    // La cinta (sensor de fin de cinta incluido) se actualiza dentro del controlador:
    // Cinta::update es el unico consumidor de SensorIR::update y emite EVT:SENSOR/EVT:CINTA.
    stationController.update(tiempo_actual);

    // Log para detectar cambios de estado (opcional)
    if (stationController.didStateChange()) {
        Logger::logDEBUG("Cambio de estado detectado: actual=%s | previo=%s",
                         stationController.getStateName(),
                         Station::Controller::stateToString(
                             stationController.getPreviousState()));

        stationController.clearStateChangeFlag();
    }

    // Refresco del watchdog: ultima sentencia de la vuelta (si el loop se cuelga, el IWDG resetea)
    IWDG->KR = 0xAAAAU;
}

// ==== CALLBACKS ====

// Actualizacion de Servos
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        servo_base.update();
        servo_hombro.update();
        servo_codo.update();
    }
}

// Interrupcion externa (evento de pulsador)
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        joystick_izquierdo.NotificarInterrupcion(GPIO_Pin);
    }

    if (GPIO_Pin == GPIO_PIN_6)
    {
        joystick_derecho.NotificarInterrupcion(GPIO_Pin);
    }

    if (sensor_fin_cinta.pinCoincide(GPIO_Pin))
    {
        sensor_fin_cinta.notificarInterrupcion(GPIO_Pin);
    }
}

// Recepcion de UART completa
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.interrupcionRecepcionByte();
    }
}

// Transmision de UART completa
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxCompleteISR();
    }
}

// Error en UART
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        terminal.onTxErrorISR();
        terminal.onRxErrorISR();
    }
}
