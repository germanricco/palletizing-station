################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App/Src/Joystick.cpp \
../App/Src/Servo.cpp \
../App/Src/TerminalSerial.cpp \
../App/Src/main_cpp.cpp 

OBJS += \
./App/Src/Joystick.o \
./App/Src/Servo.o \
./App/Src/TerminalSerial.o \
./App/Src/main_cpp.o 

CPP_DEPS += \
./App/Src/Joystick.d \
./App/Src/Servo.d \
./App/Src/TerminalSerial.d \
./App/Src/main_cpp.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.cpp App/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../App/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/home/german/Documents/ingenieria/TF_Microcontroladores_Estacion-de-paletizado/firmware_estacion_paletizado/App/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/Joystick.cyclo ./App/Src/Joystick.d ./App/Src/Joystick.o ./App/Src/Joystick.su ./App/Src/Servo.cyclo ./App/Src/Servo.d ./App/Src/Servo.o ./App/Src/Servo.su ./App/Src/TerminalSerial.cyclo ./App/Src/TerminalSerial.d ./App/Src/TerminalSerial.o ./App/Src/TerminalSerial.su ./App/Src/main_cpp.cyclo ./App/Src/main_cpp.d ./App/Src/main_cpp.o ./App/Src/main_cpp.su

.PHONY: clean-App-2f-Src

