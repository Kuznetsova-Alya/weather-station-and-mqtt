################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DHT11-DHT22-STM32-HAL-master/Src/DHT.c 

OBJS += \
./Core/Src/DHT11-DHT22-STM32-HAL-master/Src/DHT.o 

C_DEPS += \
./Core/Src/DHT11-DHT22-STM32-HAL-master/Src/DHT.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/DHT11-DHT22-STM32-HAL-master/Src/%.o Core/Src/DHT11-DHT22-STM32-HAL-master/Src/%.su: ../Core/Src/DHT11-DHT22-STM32-HAL-master/Src/%.c Core/Src/DHT11-DHT22-STM32-HAL-master/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-DHT11-2d-DHT22-2d-STM32-2d-HAL-2d-master-2f-Src

clean-Core-2f-Src-2f-DHT11-2d-DHT22-2d-STM32-2d-HAL-2d-master-2f-Src:
	-$(RM) ./Core/Src/DHT11-DHT22-STM32-HAL-master/Src/DHT.d ./Core/Src/DHT11-DHT22-STM32-HAL-master/Src/DHT.o ./Core/Src/DHT11-DHT22-STM32-HAL-master/Src/DHT.su

.PHONY: clean-Core-2f-Src-2f-DHT11-2d-DHT22-2d-STM32-2d-HAL-2d-master-2f-Src

