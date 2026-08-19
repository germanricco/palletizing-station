/**
 * @file JoystickInput.cpp
 * @brief Implementacion del acceso a hardware crudo del joystick.
 */

#include "JoystickInput.hpp"

JoystickInput::JoystickInput(volatile uint32_t* direccion_memoria_x, volatile uint32_t* direccion_memoria_y,
		GPIO_TypeDef* puerto_sw_asignado, uint16_t pin_sw_asignado)
    : ptr_raw_x(direccion_memoria_x), ptr_raw_y(direccion_memoria_y),
	  puerto_sw(puerto_sw_asignado), pin_sw(pin_sw_asignado),
	  tiempo_inicio_debounce(0), en_proceso_debounce(false) {
}

uint32_t JoystickInput::LeerRawX() const {
	return (ptr_raw_x != nullptr) ? *ptr_raw_x : 0U;
}

uint32_t JoystickInput::LeerRawY() const {
	return (ptr_raw_y != nullptr) ? *ptr_raw_y : 0U;
}

void JoystickInput::NotificarInterrupcion(uint16_t pin_disparado) {
	// FSM: Transicion iniciada exclusivamente por la ISR (Flujo unidireccional)
	if (pin_disparado == pin_sw && !en_proceso_debounce){
		tiempo_inicio_debounce = HAL_GetTick();
		en_proceso_debounce = true;
	}
}

bool JoystickInput::HayPulsacionValida() {
    if (en_proceso_debounce) {
    	// Evaluacion no bloqueante de la ventana temporal
        if ((HAL_GetTick() - tiempo_inicio_debounce) >= RETARDO_ANTIRREBOTE_MS) {

        	// FSM: Transicion finalizada exclusivamente por el bucle principal
            en_proceso_debounce = false;

            // Validacion del estado electrico post-ventana
            if (HAL_GPIO_ReadPin(puerto_sw, pin_sw) == GPIO_PIN_RESET) {
            	return true;
            }
        }
    }
    return false;
}
