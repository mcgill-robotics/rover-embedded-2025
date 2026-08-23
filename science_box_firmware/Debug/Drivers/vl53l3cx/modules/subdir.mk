################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l3cx/modules/vl53lx_api.c \
../Drivers/vl53l3cx/modules/vl53lx_api_calibration.c \
../Drivers/vl53l3cx/modules/vl53lx_api_core.c \
../Drivers/vl53l3cx/modules/vl53lx_api_debug.c \
../Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.c \
../Drivers/vl53l3cx/modules/vl53lx_core.c \
../Drivers/vl53l3cx/modules/vl53lx_core_support.c \
../Drivers/vl53l3cx/modules/vl53lx_dmax.c \
../Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.c \
../Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.c \
../Drivers/vl53l3cx/modules/vl53lx_hist_char.c \
../Drivers/vl53l3cx/modules/vl53lx_hist_core.c \
../Drivers/vl53l3cx/modules/vl53lx_hist_funcs.c \
../Drivers/vl53l3cx/modules/vl53lx_nvm.c \
../Drivers/vl53l3cx/modules/vl53lx_nvm_debug.c \
../Drivers/vl53l3cx/modules/vl53lx_register_funcs.c \
../Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.c \
../Drivers/vl53l3cx/modules/vl53lx_silicon_core.c \
../Drivers/vl53l3cx/modules/vl53lx_wait.c \
../Drivers/vl53l3cx/modules/vl53lx_xtalk.c 

OBJS += \
./Drivers/vl53l3cx/modules/vl53lx_api.o \
./Drivers/vl53l3cx/modules/vl53lx_api_calibration.o \
./Drivers/vl53l3cx/modules/vl53lx_api_core.o \
./Drivers/vl53l3cx/modules/vl53lx_api_debug.o \
./Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.o \
./Drivers/vl53l3cx/modules/vl53lx_core.o \
./Drivers/vl53l3cx/modules/vl53lx_core_support.o \
./Drivers/vl53l3cx/modules/vl53lx_dmax.o \
./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.o \
./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.o \
./Drivers/vl53l3cx/modules/vl53lx_hist_char.o \
./Drivers/vl53l3cx/modules/vl53lx_hist_core.o \
./Drivers/vl53l3cx/modules/vl53lx_hist_funcs.o \
./Drivers/vl53l3cx/modules/vl53lx_nvm.o \
./Drivers/vl53l3cx/modules/vl53lx_nvm_debug.o \
./Drivers/vl53l3cx/modules/vl53lx_register_funcs.o \
./Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.o \
./Drivers/vl53l3cx/modules/vl53lx_silicon_core.o \
./Drivers/vl53l3cx/modules/vl53lx_wait.o \
./Drivers/vl53l3cx/modules/vl53lx_xtalk.o 

C_DEPS += \
./Drivers/vl53l3cx/modules/vl53lx_api.d \
./Drivers/vl53l3cx/modules/vl53lx_api_calibration.d \
./Drivers/vl53l3cx/modules/vl53lx_api_core.d \
./Drivers/vl53l3cx/modules/vl53lx_api_debug.d \
./Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.d \
./Drivers/vl53l3cx/modules/vl53lx_core.d \
./Drivers/vl53l3cx/modules/vl53lx_core_support.d \
./Drivers/vl53l3cx/modules/vl53lx_dmax.d \
./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.d \
./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.d \
./Drivers/vl53l3cx/modules/vl53lx_hist_char.d \
./Drivers/vl53l3cx/modules/vl53lx_hist_core.d \
./Drivers/vl53l3cx/modules/vl53lx_hist_funcs.d \
./Drivers/vl53l3cx/modules/vl53lx_nvm.d \
./Drivers/vl53l3cx/modules/vl53lx_nvm_debug.d \
./Drivers/vl53l3cx/modules/vl53lx_register_funcs.d \
./Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.d \
./Drivers/vl53l3cx/modules/vl53lx_silicon_core.d \
./Drivers/vl53l3cx/modules/vl53lx_wait.d \
./Drivers/vl53l3cx/modules/vl53lx_xtalk.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l3cx/modules/%.o Drivers/vl53l3cx/modules/%.su Drivers/vl53l3cx/modules/%.cyclo: ../Drivers/vl53l3cx/modules/%.c Drivers/vl53l3cx/modules/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l3cx-2f-modules

clean-Drivers-2f-vl53l3cx-2f-modules:
	-$(RM) ./Drivers/vl53l3cx/modules/vl53lx_api.cyclo ./Drivers/vl53l3cx/modules/vl53lx_api.d ./Drivers/vl53l3cx/modules/vl53lx_api.o ./Drivers/vl53l3cx/modules/vl53lx_api.su ./Drivers/vl53l3cx/modules/vl53lx_api_calibration.cyclo ./Drivers/vl53l3cx/modules/vl53lx_api_calibration.d ./Drivers/vl53l3cx/modules/vl53lx_api_calibration.o ./Drivers/vl53l3cx/modules/vl53lx_api_calibration.su ./Drivers/vl53l3cx/modules/vl53lx_api_core.cyclo ./Drivers/vl53l3cx/modules/vl53lx_api_core.d ./Drivers/vl53l3cx/modules/vl53lx_api_core.o ./Drivers/vl53l3cx/modules/vl53lx_api_core.su ./Drivers/vl53l3cx/modules/vl53lx_api_debug.cyclo ./Drivers/vl53l3cx/modules/vl53lx_api_debug.d ./Drivers/vl53l3cx/modules/vl53lx_api_debug.o ./Drivers/vl53l3cx/modules/vl53lx_api_debug.su ./Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.cyclo ./Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.d ./Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.o ./Drivers/vl53l3cx/modules/vl53lx_api_preset_modes.su ./Drivers/vl53l3cx/modules/vl53lx_core.cyclo ./Drivers/vl53l3cx/modules/vl53lx_core.d ./Drivers/vl53l3cx/modules/vl53lx_core.o ./Drivers/vl53l3cx/modules/vl53lx_core.su ./Drivers/vl53l3cx/modules/vl53lx_core_support.cyclo ./Drivers/vl53l3cx/modules/vl53lx_core_support.d ./Drivers/vl53l3cx/modules/vl53lx_core_support.o ./Drivers/vl53l3cx/modules/vl53lx_core_support.su ./Drivers/vl53l3cx/modules/vl53lx_dmax.cyclo ./Drivers/vl53l3cx/modules/vl53lx_dmax.d ./Drivers/vl53l3cx/modules/vl53lx_dmax.o ./Drivers/vl53l3cx/modules/vl53lx_dmax.su ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.cyclo ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.d ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.o ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen3.su ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.cyclo ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.d ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.o ./Drivers/vl53l3cx/modules/vl53lx_hist_algos_gen4.su ./Drivers/vl53l3cx/modules/vl53lx_hist_char.cyclo ./Drivers/vl53l3cx/modules/vl53lx_hist_char.d ./Drivers/vl53l3cx/modules/vl53lx_hist_char.o ./Drivers/vl53l3cx/modules/vl53lx_hist_char.su ./Drivers/vl53l3cx/modules/vl53lx_hist_core.cyclo ./Drivers/vl53l3cx/modules/vl53lx_hist_core.d ./Drivers/vl53l3cx/modules/vl53lx_hist_core.o ./Drivers/vl53l3cx/modules/vl53lx_hist_core.su ./Drivers/vl53l3cx/modules/vl53lx_hist_funcs.cyclo ./Drivers/vl53l3cx/modules/vl53lx_hist_funcs.d ./Drivers/vl53l3cx/modules/vl53lx_hist_funcs.o ./Drivers/vl53l3cx/modules/vl53lx_hist_funcs.su ./Drivers/vl53l3cx/modules/vl53lx_nvm.cyclo ./Drivers/vl53l3cx/modules/vl53lx_nvm.d ./Drivers/vl53l3cx/modules/vl53lx_nvm.o ./Drivers/vl53l3cx/modules/vl53lx_nvm.su ./Drivers/vl53l3cx/modules/vl53lx_nvm_debug.cyclo ./Drivers/vl53l3cx/modules/vl53lx_nvm_debug.d ./Drivers/vl53l3cx/modules/vl53lx_nvm_debug.o ./Drivers/vl53l3cx/modules/vl53lx_nvm_debug.su ./Drivers/vl53l3cx/modules/vl53lx_register_funcs.cyclo ./Drivers/vl53l3cx/modules/vl53lx_register_funcs.d ./Drivers/vl53l3cx/modules/vl53lx_register_funcs.o ./Drivers/vl53l3cx/modules/vl53lx_register_funcs.su ./Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.cyclo ./Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.d ./Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.o ./Drivers/vl53l3cx/modules/vl53lx_sigma_estimate.su ./Drivers/vl53l3cx/modules/vl53lx_silicon_core.cyclo ./Drivers/vl53l3cx/modules/vl53lx_silicon_core.d ./Drivers/vl53l3cx/modules/vl53lx_silicon_core.o ./Drivers/vl53l3cx/modules/vl53lx_silicon_core.su ./Drivers/vl53l3cx/modules/vl53lx_wait.cyclo ./Drivers/vl53l3cx/modules/vl53lx_wait.d ./Drivers/vl53l3cx/modules/vl53lx_wait.o ./Drivers/vl53l3cx/modules/vl53lx_wait.su ./Drivers/vl53l3cx/modules/vl53lx_xtalk.cyclo ./Drivers/vl53l3cx/modules/vl53lx_xtalk.d ./Drivers/vl53l3cx/modules/vl53lx_xtalk.o ./Drivers/vl53l3cx/modules/vl53lx_xtalk.su

.PHONY: clean-Drivers-2f-vl53l3cx-2f-modules

