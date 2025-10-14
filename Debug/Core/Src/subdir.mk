################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/abc_to_dq.c \
../Core/Src/adc_readings.c \
../Core/Src/averageCalculations.c \
../Core/Src/counters.c \
../Core/Src/currentController.c \
../Core/Src/diagnostics.c \
../Core/Src/dq_to_pwm.c \
../Core/Src/input.c \
../Core/Src/main.c \
../Core/Src/pll.c \
../Core/Src/pll_lock.c \
../Core/Src/rmsCalculations.c \
../Core/Src/stateControl.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test.c \
../Core/Src/usart.c \
../Core/Src/variables.c \
../Core/Src/voltageController.c 

OBJS += \
./Core/Src/abc_to_dq.o \
./Core/Src/adc_readings.o \
./Core/Src/averageCalculations.o \
./Core/Src/counters.o \
./Core/Src/currentController.o \
./Core/Src/diagnostics.o \
./Core/Src/dq_to_pwm.o \
./Core/Src/input.o \
./Core/Src/main.o \
./Core/Src/pll.o \
./Core/Src/pll_lock.o \
./Core/Src/rmsCalculations.o \
./Core/Src/stateControl.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test.o \
./Core/Src/usart.o \
./Core/Src/variables.o \
./Core/Src/voltageController.o 

C_DEPS += \
./Core/Src/abc_to_dq.d \
./Core/Src/adc_readings.d \
./Core/Src/averageCalculations.d \
./Core/Src/counters.d \
./Core/Src/currentController.d \
./Core/Src/diagnostics.d \
./Core/Src/dq_to_pwm.d \
./Core/Src/input.d \
./Core/Src/main.d \
./Core/Src/pll.d \
./Core/Src/pll_lock.d \
./Core/Src/rmsCalculations.d \
./Core/Src/stateControl.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test.d \
./Core/Src/usart.d \
./Core/Src/variables.d \
./Core/Src/voltageController.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/abc_to_dq.cyclo ./Core/Src/abc_to_dq.d ./Core/Src/abc_to_dq.o ./Core/Src/abc_to_dq.su ./Core/Src/adc_readings.cyclo ./Core/Src/adc_readings.d ./Core/Src/adc_readings.o ./Core/Src/adc_readings.su ./Core/Src/averageCalculations.cyclo ./Core/Src/averageCalculations.d ./Core/Src/averageCalculations.o ./Core/Src/averageCalculations.su ./Core/Src/counters.cyclo ./Core/Src/counters.d ./Core/Src/counters.o ./Core/Src/counters.su ./Core/Src/currentController.cyclo ./Core/Src/currentController.d ./Core/Src/currentController.o ./Core/Src/currentController.su ./Core/Src/diagnostics.cyclo ./Core/Src/diagnostics.d ./Core/Src/diagnostics.o ./Core/Src/diagnostics.su ./Core/Src/dq_to_pwm.cyclo ./Core/Src/dq_to_pwm.d ./Core/Src/dq_to_pwm.o ./Core/Src/dq_to_pwm.su ./Core/Src/input.cyclo ./Core/Src/input.d ./Core/Src/input.o ./Core/Src/input.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pll.cyclo ./Core/Src/pll.d ./Core/Src/pll.o ./Core/Src/pll.su ./Core/Src/pll_lock.cyclo ./Core/Src/pll_lock.d ./Core/Src/pll_lock.o ./Core/Src/pll_lock.su ./Core/Src/rmsCalculations.cyclo ./Core/Src/rmsCalculations.d ./Core/Src/rmsCalculations.o ./Core/Src/rmsCalculations.su ./Core/Src/stateControl.cyclo ./Core/Src/stateControl.d ./Core/Src/stateControl.o ./Core/Src/stateControl.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/test.cyclo ./Core/Src/test.d ./Core/Src/test.o ./Core/Src/test.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/variables.cyclo ./Core/Src/variables.d ./Core/Src/variables.o ./Core/Src/variables.su ./Core/Src/voltageController.cyclo ./Core/Src/voltageController.d ./Core/Src/voltageController.o ./Core/Src/voltageController.su

.PHONY: clean-Core-2f-Src

