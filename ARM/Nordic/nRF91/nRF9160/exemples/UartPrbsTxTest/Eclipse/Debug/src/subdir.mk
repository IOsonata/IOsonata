################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/Users/hoan/swdev/IOsonata/exemples/uart/uart_prbs_tx.cpp 

OBJS += \
./src/uart_prbs_tx.o 

CPP_DEPS += \
./src/uart_prbs_tx.d 


# Each subdirectory must supply rules for building sources it contributes
src/uart_prbs_tx.o: /Users/hoan/swdev/IOsonata/exemples/uart/uart_prbs_tx.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m33 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=auto -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DNRF9160_XXAA -I"../../src" -I"../../../../lib/include" -I"../../../../include" -I"../../../../../include" -I"../../../../../../include" -I"../../../../../../../include" -I"../../../../../../../../include" -I"../../../../../../../CMSIS/Core/Include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


