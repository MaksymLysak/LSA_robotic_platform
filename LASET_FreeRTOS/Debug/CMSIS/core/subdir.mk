################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/core/core_cm3.c 

OBJS += \
./CMSIS/core/core_cm3.o 

C_DEPS += \
./CMSIS/core/core_cm3.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/core/%.o: ../CMSIS/core/%.c CMSIS/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F1 -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -c -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/StdPeriph_Driver/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/core" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/device" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/FreeRTOS/include" -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-CMSIS-2f-core

clean-CMSIS-2f-core:
	-$(RM) ./CMSIS/core/core_cm3.d ./CMSIS/core/core_cm3.o

.PHONY: clean-CMSIS-2f-core

