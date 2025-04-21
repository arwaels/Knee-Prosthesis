################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bno055_stm32-master/bno055.c 

OBJS += \
./Drivers/bno055_stm32-master/bno055.o 

C_DEPS += \
./Drivers/bno055_stm32-master/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/bno055_stm32-master/%.o Drivers/bno055_stm32-master/%.su Drivers/bno055_stm32-master/%.cyclo: ../Drivers/bno055_stm32-master/%.c Drivers/bno055_stm32-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Arwar/Documents/Embedded-C/My_Workspace/target/sensorthigh/Drivers/bno055_stm32-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-bno055_stm32-2d-master

clean-Drivers-2f-bno055_stm32-2d-master:
	-$(RM) ./Drivers/bno055_stm32-master/bno055.cyclo ./Drivers/bno055_stm32-master/bno055.d ./Drivers/bno055_stm32-master/bno055.o ./Drivers/bno055_stm32-master/bno055.su

.PHONY: clean-Drivers-2f-bno055_stm32-2d-master

