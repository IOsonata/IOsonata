################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/hoan/swdev/external/nrf5_sdk_thread_zigbee/external/zboss/src/zb_error/zb_error_to_string.c 

C_DEPS += \
./src/zboss/src/zb_error/zb_error_to_string.d 

OBJS += \
./src/zboss/src/zb_error/zb_error_to_string.o 


# Each subdirectory must supply rules for building sources it contributes
src/zboss/src/zb_error/zb_error_to_string.o: /Users/hoan/swdev/external/nrf5_sdk_thread_zigbee/external/zboss/src/zb_error/zb_error_to_string.c src/zboss/src/zb_error/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DAPP_TIMER_V2_RTC1_ENABLED -DBOARD_PCA10056 -DCONFIG_GPIO_AS_PINRESET -DCONFIG_ZIGBEE_ERROR_TO_STRING_ENABLED -DENABLE_FEM -DFLOAT_ABI_HARD -DLIBZBOSS_CONFIG_FILE=\"libzboss_config.ed.h\" -DNRF52840_XXAA -DZBOSS_BUILD -DZB_ED_ROLE -DZB_NRF_INTERNAL -DZB_NVRAM_FLASH_AUTO_ADDRESS -DZB_ZCL_SCENES_OPTIONAL_COMMANDS_DISABLED -DZB_ZCL_USE_ZCL7_AS_DEFAULT_REVISION -I"../../src" -I"../../../../lib/include" -I"../../../../include" -I"../../../../../../include" -I"../../../../../../../include" -I"../../../../../../../../include" -I"../../../../../../../CMSIS/Core/Include" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/integration/nrfx" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/integration/nrfx/legacy" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/modules/nrfx" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/modules/nrfx/drivers/include" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/modules/nrfx/hal" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/modules/nrfx/mdk" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/external/zboss/include/addons" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/external/zboss/include/osif" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/external/zboss/include/ha" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/external/zboss/include" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/external/zboss/src/include" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/external/zboss/src/zb_error" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/zigbee/common" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/boards" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/balloc" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/bsp" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/button" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/experimental_section_vars" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/log" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/log/src" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/memobj" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/pwm" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/pwr_mgmt" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/strerror" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/timer" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/libraries/util" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/softdevice/common" -I"../../../../../../../../../external/nrf5_sdk_thread_zigbee/components/softdevice/s140/headers" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


