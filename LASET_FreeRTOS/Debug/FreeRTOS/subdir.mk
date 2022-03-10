################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/croutine.c \
../FreeRTOS/event_groups.c \
../FreeRTOS/heap_2.c \
../FreeRTOS/list.c \
../FreeRTOS/port.c \
../FreeRTOS/queue.c \
../FreeRTOS/stream_buffer.c \
../FreeRTOS/tasks.c \
../FreeRTOS/timers.c 

OBJS += \
./FreeRTOS/croutine.o \
./FreeRTOS/event_groups.o \
./FreeRTOS/heap_2.o \
./FreeRTOS/list.o \
./FreeRTOS/port.o \
./FreeRTOS/queue.o \
./FreeRTOS/stream_buffer.o \
./FreeRTOS/tasks.o \
./FreeRTOS/timers.o 

C_DEPS += \
./FreeRTOS/croutine.d \
./FreeRTOS/event_groups.d \
./FreeRTOS/heap_2.d \
./FreeRTOS/list.d \
./FreeRTOS/port.d \
./FreeRTOS/queue.d \
./FreeRTOS/stream_buffer.d \
./FreeRTOS/tasks.d \
./FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/%.o: ../FreeRTOS/%.c FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32F1 -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -c -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/StdPeriph_Driver/inc" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/core" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/CMSIS/device" -I"/home/maksym/STM32CubeIDE/workspace_1.8.0/LASET_FreeRTOS/FreeRTOS/include" -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-FreeRTOS

clean-FreeRTOS:
	-$(RM) ./FreeRTOS/croutine.d ./FreeRTOS/croutine.o ./FreeRTOS/event_groups.d ./FreeRTOS/event_groups.o ./FreeRTOS/heap_2.d ./FreeRTOS/heap_2.o ./FreeRTOS/list.d ./FreeRTOS/list.o ./FreeRTOS/port.d ./FreeRTOS/port.o ./FreeRTOS/queue.d ./FreeRTOS/queue.o ./FreeRTOS/stream_buffer.d ./FreeRTOS/stream_buffer.o ./FreeRTOS/tasks.d ./FreeRTOS/tasks.o ./FreeRTOS/timers.d ./FreeRTOS/timers.o

.PHONY: clean-FreeRTOS

