################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.c 

OBJS += \
./Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.o 

C_DEPS += \
./Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BNO055-for-STM32-master/i2c/%.o Drivers/BNO055-for-STM32-master/i2c/%.su Drivers/BNO055-for-STM32-master/i2c/%.cyclo: ../Drivers/BNO055-for-STM32-master/i2c/%.c Drivers/BNO055-for-STM32-master/i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Arwar/Documents/Embedded-C/My_Workspace/target/sensorthigh/Drivers/BNO055-for-STM32-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BNO055-2d-for-2d-STM32-2d-master-2f-i2c

clean-Drivers-2f-BNO055-2d-for-2d-STM32-2d-master-2f-i2c:
	-$(RM) ./Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.cyclo ./Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.d ./Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.o ./Drivers/BNO055-for-STM32-master/i2c/bsp_i2c_ee.su

.PHONY: clean-Drivers-2f-BNO055-2d-for-2d-STM32-2d-master-2f-i2c

