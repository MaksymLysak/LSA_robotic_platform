################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/fun_for_USART.c \
../src/init_config.c \
../src/init_odom_var.c \
../src/main.c \
../src/stm32f10x_it.c \
../src/syscalls.c 

OBJS += \
./src/fun_for_USART.o \
./src/init_config.o \
./src/init_odom_var.o \
./src/main.o \
./src/stm32f10x_it.o \
./src/syscalls.o 

C_DEPS += \
./src/fun_for_USART.d \
./src/init_config.d \
./src/init_odom_var.d \
./src/main.d \
./src/stm32f10x_it.d \
./src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F1 -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -c -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/StdPeriph_Driver/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/core" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/device" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/FreeRTOS/include" -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-src

clean-src:
	-$(RM) ./src/fun_for_USART.d ./src/fun_for_USART.o ./src/init_config.d ./src/init_config.o ./src/init_odom_var.d ./src/init_odom_var.o ./src/main.d ./src/main.o ./src/stm32f10x_it.d ./src/stm32f10x_it.o ./src/syscalls.d ./src/syscalls.o

.PHONY: clean-src

