PROJECT := TinyUsbCdcLoopback
CONFIG ?= Release

IO_ROOT := $(abspath $(CURDIR)/../../../../../../..)
WORKSPACE_ROOT := $(abspath $(IO_ROOT)/..)
APP_DIR := $(IO_ROOT)/exemples/usb/tinyusb_cdc_loopback
COMMON_DIR := $(IO_ROOT)/exemples/usb/tinyusb_common
TINYUSB_DIR := $(WORKSPACE_ROOT)/external/tinyusb
NRFX_DIR := $(WORKSPACE_ROOT)/external/nrfx

TOOLCHAIN ?= arm-none-eabi-
CC := $(TOOLCHAIN)gcc
CXX := $(TOOLCHAIN)g++
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE := $(TOOLCHAIN)size

BUILD_DIR := $(CURDIR)/$(CONFIG)
ELF := $(BUILD_DIR)/$(PROJECT).elf
HEX := $(BUILD_DIR)/$(PROJECT).hex
BIN := $(BUILD_DIR)/$(PROJECT).bin
MAP := $(BUILD_DIR)/$(PROJECT).map

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
COMMON_FLAGS := $(CPUFLAGS) -ffunction-sections -fdata-sections -fsigned-char -fmessage-length=0 -DNRF52840_XXAA
ifeq ($(CONFIG),Debug)
OPTFLAGS := -O0 -g3
else
OPTFLAGS := -Os
endif

INCLUDES := \
	-I$(APP_DIR) \
	-I$(COMMON_DIR) \
	-I$(IO_ROOT)/ARM/Nordic/nRF52/nRF52840/lib/include \
	-I$(IO_ROOT)/ARM/Nordic/nRF52/include \
	-I$(IO_ROOT)/ARM/Nordic/include \
	-I$(IO_ROOT)/ARM/include \
	-I$(IO_ROOT)/include \
	-I$(IO_ROOT)/ARM/CMSIS/Core/Include \
	-I$(TINYUSB_DIR)/src \
	-I$(NRFX_DIR) \
	-I$(NRFX_DIR)/drivers/include \
	-I$(NRFX_DIR)/hal \
	-I$(NRFX_DIR)/bsp/stable/mdk

CFLAGS := $(COMMON_FLAGS) $(OPTFLAGS) -std=gnu17 -MMD -MP $(INCLUDES)
CXXFLAGS := $(COMMON_FLAGS) $(OPTFLAGS) -std=gnu++23 -fno-rtti -fno-exceptions -MMD -MP $(INCLUDES)
LDFLAGS := $(CPUFLAGS) --specs=nano.specs -T$(IO_ROOT)/ARM/Nordic/nRF52/nRF52840/ldscript/gcc_nrf52840_xxaa.ld -Wl,--gc-sections -Wl,-Map,$(MAP)
LDLIBS := -L$(IO_ROOT)/ARM/Nordic/nRF52/nRF52840/lib/ioc/$(CONFIG) -Wl,--start-group -lIOsonata_nRF52840 -lgcc -lc -lm -Wl,--end-group

OBJS := \
	$(BUILD_DIR)/main.o \
	$(BUILD_DIR)/tusb.o \
	$(BUILD_DIR)/tusb_fifo.o \
	$(BUILD_DIR)/usbd.o \
	$(BUILD_DIR)/usbd_control.o \
	$(BUILD_DIR)/cdc_device.o \
	$(BUILD_DIR)/dcd_nrf5x.o

.PHONY: all release debug clean check-deps

all: check-deps $(HEX) $(BIN)
	$(SIZE) $(ELF)

release:
	$(MAKE) CONFIG=Release all

debug:
	$(MAKE) CONFIG=Debug all

check-deps:
	@test -f "$(TINYUSB_DIR)/src/tusb.c" || (echo "TinyUSB not found at $(TINYUSB_DIR). Checkout TinyUSB 0.21.0 there."; exit 1)
	@test -f "$(NRFX_DIR)/bsp/stable/mdk/nrf.h" || (echo "nrfx not found at $(NRFX_DIR)."; exit 1)
	@test -f "$(IO_ROOT)/ARM/Nordic/nRF52/nRF52840/lib/ioc/$(CONFIG)/libIOsonata_nRF52840.a" || (echo "Build IOsonata_nRF52840 $(CONFIG) first."; exit 1)

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_DIR)/main.o: $(APP_DIR)/main.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/tusb.o: $(TINYUSB_DIR)/src/tusb.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/tusb_fifo.o: $(TINYUSB_DIR)/src/common/tusb_fifo.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/usbd.o: $(TINYUSB_DIR)/src/device/usbd.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/usbd_control.o: $(TINYUSB_DIR)/src/device/usbd_control.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/cdc_device.o: $(TINYUSB_DIR)/src/class/cdc/cdc_device.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/dcd_nrf5x.o: $(TINYUSB_DIR)/src/portable/nordic/nrf5x/dcd_nrf5x.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf Debug Release

-include $(OBJS:.o=.d)
