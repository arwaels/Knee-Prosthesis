################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BNO055-for-STM32-master/usart/bsp_usart1.c 

OBJS += \
./Drivers/BNO055-for-STM32-master/usart/bsp_usart1.o 

C_DEPS += \
./Drivers/BNO055-for-STM32-master/usart/bsp_usart1.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BNO055-for-STM32-master/usart/%.o Drivers/BNO055-for-STM32-master/usart/%.su Drivers/BNO055-for-STM32-master/usart/%.cyclo: ../Drivers/BNO055-for-STM32-master/usart/%.c Drivers/BNO055-for-STM32-master/usart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Arwar/Documents/Embedded-C/My_Workspace/target/sensorthigh/Drivers/BNO055-for-STM32-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BNO055-2d-for-2d-STM32-2d-master-2f-usart

clean-Drivers-2f-BNO055-2d-for-2d-STM32-2d-master-2f-usart:
	-$(RM) ./Drivers/BNO055-for-STM32-master/usart/bsp_usart1.cyclo ./Drivers/BNO055-for-STM32-master/usart/bsp_usart1.d ./Drivers/BNO055-for-STM32-master/usart/bsp_usart1.o ./Drivers/BNO055-for-STM32-master/usart/bsp_usart1.su

.PHONY: clean-Drivers-2f-BNO055-2d-for-2d-STM32-2d-master-2f-usart

