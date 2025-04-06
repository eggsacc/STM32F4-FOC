################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/OLED/Src/ssd1306.c \
../Drivers/OLED/Src/ssd1306_fonts.c 

OBJS += \
./Drivers/OLED/Src/ssd1306.o \
./Drivers/OLED/Src/ssd1306_fonts.o 

C_DEPS += \
./Drivers/OLED/Src/ssd1306.d \
./Drivers/OLED/Src/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/OLED/Src/%.o Drivers/OLED/Src/%.su Drivers/OLED/Src/%.cyclo: ../Drivers/OLED/Src/%.c Drivers/OLED/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I"C:/Users/yizha/STM32CubeIDE/workspace_1.17.0/STM32F411_FOC/Drivers/OLED/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-OLED-2f-Src

clean-Drivers-2f-OLED-2f-Src:
	-$(RM) ./Drivers/OLED/Src/ssd1306.cyclo ./Drivers/OLED/Src/ssd1306.d ./Drivers/OLED/Src/ssd1306.o ./Drivers/OLED/Src/ssd1306.su ./Drivers/OLED/Src/ssd1306_fonts.cyclo ./Drivers/OLED/Src/ssd1306_fonts.d ./Drivers/OLED/Src/ssd1306_fonts.o ./Drivers/OLED/Src/ssd1306_fonts.su

.PHONY: clean-Drivers-2f-OLED-2f-Src

