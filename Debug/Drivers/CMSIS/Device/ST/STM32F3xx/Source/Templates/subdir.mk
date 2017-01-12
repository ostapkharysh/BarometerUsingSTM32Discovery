################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.o 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/%.o: ../Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F303xC -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Inc" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/STM32F3xx_HAL_Driver/Inc" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/CMSIS/Include" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/CMSIS/Device/ST/STM32F3xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


