################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: arm-none-eabi-*
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c 

OBJS += \
./Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o 

C_DEPS += \
./Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/%.o Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/%.su Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/%.cyclo: ../Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/%.c Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-SystemView-2f-Sample-2f-FreeRTOS-2f-Config

clean-Middlewares-2f-Third_Party-2f-SystemView-2f-Sample-2f-FreeRTOS-2f-Config:
	-$(RM) ./Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.cyclo ./Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d ./Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o ./Middlewares/Third_Party/SystemView/Sample/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-SystemView-2f-Sample-2f-FreeRTOS-2f-Config

