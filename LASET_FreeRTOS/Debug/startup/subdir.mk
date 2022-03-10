################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 

S_DEPS += \
./startup/startup_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/StdPeriph_Driver/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/core" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/device" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/FreeRTOS/include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-startup

clean-startup:
	-$(RM) ./startup/startup_stm32.d ./startup/startup_stm32.o

.PHONY: clean-startup

