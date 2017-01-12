################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Src/sh_cmd.s 

C_SRCS += \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/semihosting.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/syscalls.c 

OBJS += \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/semihosting.o \
./Src/sh_cmd.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/syscalls.o 

C_DEPS += \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/semihosting.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F303xC -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Inc" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/STM32F3xx_HAL_Driver/Inc" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/CMSIS/Include" -I"D:/CSUCU/princ_of_org_comp_sys/BAR_CUBE_PROJ/Drivers/CMSIS/Device/ST/STM32F3xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


