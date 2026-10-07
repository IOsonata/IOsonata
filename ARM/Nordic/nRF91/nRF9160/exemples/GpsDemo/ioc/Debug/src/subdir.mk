################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/Users/hoan/swdev/IOsonata/ARM/Nordic/nRF91/nRF9160/exemples/GpsDemo/src/main.cpp 

C_SRCS += \
/Users/hoan/swdev/IOsonata/ARM/Nordic/nRF91/src/nrf_modem_os_bare.c 

C_DEPS += \
./src/nrf_modem_os_bare.d 

OBJS += \
./src/main.o \
./src/nrf_modem_os_bare.o 

CPP_DEPS += \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/main.o: /Users/hoan/swdev/IOsonata/ARM/Nordic/nRF91/nRF9160/exemples/GpsDemo/src/main.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m33 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=auto -mcmse -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DNRF_TRUSTZONE_NONSECURE -DNRF9160_XXAA -I"../../src" -I"../../../../include" -I"../../../../lib/include" -I"../../../../../bsdlib/include" -I"../../../../../include" -I"../../../../../../include" -I"../../../../../../../include" -I"../../../../../../../CMSIS/Core/Include" -I"../../../../../../../../include" -I"../../../../../../../../../external/nRF5_SDK/modules/nrfx" -I"../../../../../../../../../external/nRF5_SDK/modules/nrfx/hal" -I"../../../../../../../../../external/sdk-nrfxlib/nrf_modem/include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/nrf_modem_os_bare.o: /Users/hoan/swdev/IOsonata/ARM/Nordic/nRF91/src/nrf_modem_os_bare.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=auto -mcmse -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DNRF_TRUSTZONE_NONSECURE -DNRF9160_XXAA -I"../../src" -I"../../../../include" -I"../../../../lib/include" -I"../../../../../bsdlib/include" -I"../../../../../include" -I"../../../../../../include" -I"../../../../../../../include" -I"../../../../../../../CMSIS/Core/Include" -I"../../../../../../../../include" -I"../../../../../../../../../external/nRF5_SDK/modules/nrfx" -I"../../../../../../../../../external/nRF5_SDK/modules/nrfx/hal" -I"../../../../../../../../../external/sdk-nrfxlib/nrf_modem/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


