################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l3cx/porting/vl53lx_platform.c \
../Drivers/vl53l3cx/porting/vl53lx_platform_ipp.c \
../Drivers/vl53l3cx/porting/vl53lx_platform_log.c 

OBJS += \
./Drivers/vl53l3cx/porting/vl53lx_platform.o \
./Drivers/vl53l3cx/porting/vl53lx_platform_ipp.o \
./Drivers/vl53l3cx/porting/vl53lx_platform_log.o 

C_DEPS += \
./Drivers/vl53l3cx/porting/vl53lx_platform.d \
./Drivers/vl53l3cx/porting/vl53lx_platform_ipp.d \
./Drivers/vl53l3cx/porting/vl53lx_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l3cx/porting/%.o Drivers/vl53l3cx/porting/%.su Drivers/vl53l3cx/porting/%.cyclo: ../Drivers/vl53l3cx/porting/%.c Drivers/vl53l3cx/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l3cx-2f-porting

clean-Drivers-2f-vl53l3cx-2f-porting:
	-$(RM) ./Drivers/vl53l3cx/porting/vl53lx_platform.cyclo ./Drivers/vl53l3cx/porting/vl53lx_platform.d ./Drivers/vl53l3cx/porting/vl53lx_platform.o ./Drivers/vl53l3cx/porting/vl53lx_platform.su ./Drivers/vl53l3cx/porting/vl53lx_platform_ipp.cyclo ./Drivers/vl53l3cx/porting/vl53lx_platform_ipp.d ./Drivers/vl53l3cx/porting/vl53lx_platform_ipp.o ./Drivers/vl53l3cx/porting/vl53lx_platform_ipp.su ./Drivers/vl53l3cx/porting/vl53lx_platform_log.cyclo ./Drivers/vl53l3cx/porting/vl53lx_platform_log.d ./Drivers/vl53l3cx/porting/vl53lx_platform_log.o ./Drivers/vl53l3cx/porting/vl53lx_platform_log.su

.PHONY: clean-Drivers-2f-vl53l3cx-2f-porting

