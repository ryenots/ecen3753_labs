################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: arm-none-eabi-*
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.c \
../Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.c \
../Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.o \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.o \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.d \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.d \
./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SystemView/SEGGER/%.o Middlewares/Third_Party/SystemView/SEGGER/%.su Middlewares/Third_Party/SystemView/SEGGER/%.cyclo: ../Middlewares/Third_Party/SystemView/SEGGER/%.c Middlewares/Third_Party/SystemView/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/SystemView/SEGGER/%.o: ../Middlewares/Third_Party/SystemView/SEGGER/%.S Middlewares/Third_Party/SystemView/SEGGER/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Middlewares-2f-Third_Party-2f-SystemView-2f-SEGGER

clean-Middlewares-2f-Third_Party-2f-SystemView-2f-SEGGER:
	-$(RM) ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.cyclo ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.d ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.o ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT.su ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.d ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.o ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.cyclo ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.d ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.o ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_RTT_printf.su ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.cyclo ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.d ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.o ./Middlewares/Third_Party/SystemView/SEGGER/SEGGER_SYSVIEW.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-SystemView-2f-SEGGER

