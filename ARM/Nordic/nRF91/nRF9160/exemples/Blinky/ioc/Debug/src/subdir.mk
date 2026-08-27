################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/hoan/swdev/IOsonata/exemples/misc/blinky.c 

C_DEPS += \
./src/blinky.d 

OBJS += \
./src/blinky.o 


# Each subdirectory must supply rules for building sources it contributes
src/blinky.o: /Users/hoan/swdev/IOsonata/exemples/misc/blinky.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=auto -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -D__PROGRAM_START -DNRF9160_XXAA -I"../../src" -I"../../../../include" -I"../../../../lib/include" -I"../../../../../include" -I"../../../../../../include" -I"../../../../../../../include" -I"../../../../../../../CMSIS/Core/Include" -I"../../../../../../../../include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


