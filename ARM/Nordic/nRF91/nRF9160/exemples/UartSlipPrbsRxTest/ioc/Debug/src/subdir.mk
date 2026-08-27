################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/Users/hoan/swdev/IOsonata/exemples/uart/uart_slip_prbs_rx.cpp 

OBJS += \
./src/uart_slip_prbs_rx.o 

CPP_DEPS += \
./src/uart_slip_prbs_rx.d 


# Each subdirectory must supply rules for building sources it contributes
src/uart_slip_prbs_rx.o: /Users/hoan/swdev/IOsonata/exemples/uart/uart_slip_prbs_rx.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=auto -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DNRF9160_XXAA -I"../../src" -I"../../../../lib/include" -I"../../../../include" -I"../../../../../../include" -I"../../../../../../../include" -I"../../../../../../../../include" -I"../../../../../../../CMSIS/Core/Include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


