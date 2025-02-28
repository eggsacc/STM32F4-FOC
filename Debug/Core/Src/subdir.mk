################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AS5600.c \
../Core/Src/current_sense.c \
../Core/Src/fix16.c \
../Core/Src/foc_core.c \
../Core/Src/foc_hw.c \
../Core/Src/foc_utils.c \
../Core/Src/low_pass_filter.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/serial_commander.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/AS5600.o \
./Core/Src/current_sense.o \
./Core/Src/fix16.o \
./Core/Src/foc_core.o \
./Core/Src/foc_hw.o \
./Core/Src/foc_utils.o \
./Core/Src/low_pass_filter.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/serial_commander.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/AS5600.d \
./Core/Src/current_sense.d \
./Core/Src/fix16.d \
./Core/Src/foc_core.d \
./Core/Src/foc_hw.d \
./Core/Src/foc_utils.d \
./Core/Src/low_pass_filter.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/serial_commander.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/serial_commander.o: ../Core/Src/serial_commander.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u_printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AS5600.cyclo ./Core/Src/AS5600.d ./Core/Src/AS5600.o ./Core/Src/AS5600.su ./Core/Src/current_sense.cyclo ./Core/Src/current_sense.d ./Core/Src/current_sense.o ./Core/Src/current_sense.su ./Core/Src/fix16.cyclo ./Core/Src/fix16.d ./Core/Src/fix16.o ./Core/Src/fix16.su ./Core/Src/foc_core.cyclo ./Core/Src/foc_core.d ./Core/Src/foc_core.o ./Core/Src/foc_core.su ./Core/Src/foc_hw.cyclo ./Core/Src/foc_hw.d ./Core/Src/foc_hw.o ./Core/Src/foc_hw.su ./Core/Src/foc_utils.cyclo ./Core/Src/foc_utils.d ./Core/Src/foc_utils.o ./Core/Src/foc_utils.su ./Core/Src/low_pass_filter.cyclo ./Core/Src/low_pass_filter.d ./Core/Src/low_pass_filter.o ./Core/Src/low_pass_filter.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/serial_commander.cyclo ./Core/Src/serial_commander.d ./Core/Src/serial_commander.o ./Core/Src/serial_commander.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

