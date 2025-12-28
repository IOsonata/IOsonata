# Makefile for libIOsonata_nRF52832.a
# Builds IOsonata library for Nordic nRF52832 target
# Supports Debug and Release build variants

# ============================================
# Include auto-generated path configuration
# ============================================
# Determine IOSONATA_ROOT relative to this Makefile
MAKEFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
IOSONATA_ROOT := $(abspath $(MAKEFILE_DIR)../../../../..)

# Include the path configuration if it exists
-include $(IOSONATA_ROOT)/makefile_path.mk

# Fallback paths if makefile_path.mk doesn't exist
ARM_GCC ?= arm-none-eabi-gcc
ARM_GPP ?= arm-none-eabi-g++
ARM_AS ?= arm-none-eabi-as
ARM_AR ?= arm-none-eabi-ar
ARM_OBJCOPY ?= arm-none-eabi-objcopy
ARM_SIZE ?= arm-none-eabi-size

IOSONATA_INCLUDE ?= $(IOSONATA_ROOT)/include
IOSONATA_SRC ?= $(IOSONATA_ROOT)/src
ARM_ROOT ?= $(IOSONATA_ROOT)/ARM
ARM_CMSIS_INCLUDE ?= $(ARM_ROOT)/CMSIS/Core/Include
ARM_INCLUDE ?= $(ARM_ROOT)/include
ARM_SRC ?= $(ARM_ROOT)/src
ARM_NORDIC ?= $(ARM_ROOT)/Nordic
NRFX_ROOT ?= $(IOSONATA_ROOT)/../external/nrfx
NRF5_SDK ?=  $(IOSONATA_ROOT)/../external/nRF5_SDK

# Build common include paths if not defined
IOSONATA_INCLUDES ?= -I$(IOSONATA_INCLUDE) \
                     -I$(IOSONATA_INCLUDE)/bluetooth \
                     -I$(IOSONATA_INCLUDE)/audio \
                     -I$(IOSONATA_INCLUDE)/converters \
                     -I$(IOSONATA_INCLUDE)/coredev \
                     -I$(IOSONATA_INCLUDE)/display \
                     -I$(IOSONATA_INCLUDE)/imu \
                     -I$(IOSONATA_INCLUDE)/miscdev \
                     -I$(IOSONATA_INCLUDE)/pwrmgnt \
                     -I$(IOSONATA_INCLUDE)/sensors \
                     -I$(IOSONATA_INCLUDE)/storage \
                     -I$(IOSONATA_INCLUDE)/sys \
                     -I$(IOSONATA_INCLUDE)/usb

ARM_INCLUDES ?= -I$(ARM_INCLUDE) \
                -I$(ARM_CMSIS_INCLUDE)

# ============================================
# Library Configuration
# ============================================
LIBRARY_NAME = libIOsonata_nRF52832.a
MCU = nRF52832
TARGET_DEVICE = NRF52832_XXAA

# Build variant: Debug or Release (default: Release)
VARIANT ?= Release

# Build directory based on variant
BUILD_DIR = $(VARIANT)
OBJ_DIR = $(BUILD_DIR)/obj

# ============================================
# Toolchain Setup
# ============================================
CC = $(ARM_GCC)
CXX = $(ARM_GPP)
AS = $(ARM_AS)
AR = $(ARM_AR)
OBJCOPY = $(ARM_OBJCOPY)
SIZE = $(ARM_SIZE)

# ============================================
# Device-specific Settings
# ============================================
# CPU flags for Cortex-M4 with FPU
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Preprocessor defines
DEFINES = -D$(TARGET_DEVICE) \
          -DNRF52832 \
          -DNRF52 \
          -DARM_MATH_CM4 \
          -D__FPU_PRESENT=1 \
          -DCONFIG_NFCT_PINS_AS_GPIOS \
          -DCONFIG_GPIO_AS_PINRESET

# ============================================
# Include Paths
# ============================================
INCLUDES = $(IOSONATA_INCLUDES) \
           $(ARM_INCLUDES) \
           -I$(MAKEFILE_DIR)include \
           -I$(ARM_NORDIC)/nRF52/$(MCU)/lib/include \
           -I$(ARM_NORDIC)/nRF52/include \
           -I$(ARM_NORDIC)/include \
           -I$(NRFX_ROOT) \
           -I$(NRFX_ROOT)/bsp/stable/mdk \
           -I$(NRFX_ROOT)/hal \
           -I$(NRFX_ROOT)/drivers/include \
           -I$(NRFX_ROOT)/soc \
           -I$(NRF5_SDK)/components/libraries/scheduler \
           -I$(NRF5_SDK)/components/libraries/timer \
           -I$(NRF5_SDK)/components/libraries/util \
           -I$(NRF5_SDK)/components/softdevice/common \
           -I$(NRF5_SDK)/components/softdevice/s132/headers

# ============================================
# Compiler Flags
# ============================================
# Common flags for both C and C++
COMMON_FLAGS = $(CPU_FLAGS) \
               -Wall \
               -Werror \
               -ffunction-sections \
               -fdata-sections \
               -fno-strict-aliasing \
               -fno-builtin \
               --short-enums \
               $(DEFINES) \
               $(INCLUDES)

# C-specific flags
CFLAGS = $(COMMON_FLAGS) \
         -std=c17

# C++-specific flags
CXXFLAGS = $(COMMON_FLAGS) \
           -std=c++17 \
           -fno-exceptions \
           -fno-rtti \
           -fno-threadsafe-statics

# Assembly flags
ASFLAGS = $(CPU_FLAGS)

# Variant-specific flags
ifeq ($(VARIANT),Debug)
    CFLAGS += -O0 -g3 -DDEBUG
    CXXFLAGS += -O0 -g3 -DDEBUG
else ifeq ($(VARIANT),Release)
    CFLAGS += -O3 -DNDEBUG
    CXXFLAGS += -O3 -DNDEBUG
else
    $(error Unknown build variant: $(VARIANT). Use Debug or Release)
endif

# ============================================
# Source Files
# ============================================

# Device-specific sources (nRF52832)
DEVICE_SRCS = $(wildcard src/*.c)

# Common nRF52 sources
NRF52_SRC_DIR = $(ARM_NORDIC)/nRF52/src
NRF52_SRCS = $(wildcard $(NRF52_SRC_DIR)/*.c) \
             $(wildcard $(NRF52_SRC_DIR)/*.cpp)

# ARM common sources
ARM_COMMON_SRCS = $(wildcard $(ARM_SRC)/*.c) \
                  $(wildcard $(ARM_SRC)/*.cpp)

# IOsonata core sources
IOSONATA_SRCS = $(wildcard $(IOSONATA_SRC)/*.cpp) \
                $(wildcard $(IOSONATA_SRC)/*.c)

# IOsonata module sources
IOSONATA_MODULE_SRCS = $(wildcard $(IOSONATA_SRC)/pwrmgnt/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/bluetooth/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/miscdev/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/coredev/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/usb/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/display/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/converters/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/imu/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/sensors/*.cpp) \
                       $(wildcard $(IOSONATA_SRC)/storage/*.cpp)

# Combine all sources
ALL_SRCS = $(DEVICE_SRCS) \
           $(NRF52_SRCS) \
           $(ARM_COMMON_SRCS) \
           $(IOSONATA_SRCS) \
           $(IOSONATA_MODULE_SRCS)

# Generate object file paths
OBJS = $(addprefix $(OBJ_DIR)/, $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(notdir $(ALL_SRCS)))))

# ============================================
# VPATH for source file discovery
# ============================================
VPATH = src \
        $(NRF52_SRC_DIR) \
        $(ARM_SRC) \
        $(IOSONATA_SRC) \
        $(IOSONATA_SRC)/pwrmgnt \
        $(IOSONATA_SRC)/bluetooth \
        $(IOSONATA_SRC)/miscdev \
        $(IOSONATA_SRC)/coredev \
        $(IOSONATA_SRC)/usb \
        $(IOSONATA_SRC)/display \
        $(IOSONATA_SRC)/converters \
        $(IOSONATA_SRC)/imu \
        $(IOSONATA_SRC)/sensors \
        $(IOSONATA_SRC)/storage

# ============================================
# Build Rules
# ============================================

.PHONY: all debug release clean clean_debug clean_release info

# Default target
all: $(BUILD_DIR)/$(LIBRARY_NAME)

# Convenience targets
debug:
	@$(MAKE) VARIANT=Debug

release:
	@$(MAKE) VARIANT=Release

# Build the library
$(BUILD_DIR)/$(LIBRARY_NAME): $(OBJS)
	@echo "Creating library: $@"
	@mkdir -p $(BUILD_DIR)
	$(AR) rcs $@ $^
	@echo ""
	@echo "=============================================="
	@echo "Library built successfully: $@"
	@echo "Build variant: $(VARIANT)"
	@echo "=============================================="
	$(SIZE) $@

# Compile C++ sources
$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(OBJ_DIR)
	@echo "Compiling C++: $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile C sources
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(OBJ_DIR)
	@echo "Compiling C: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Compile Assembly sources
$(OBJ_DIR)/%.o: %.s
	@mkdir -p $(OBJ_DIR)
	@echo "Assembling: $<"
	$(AS) $(ASFLAGS) -c $< -o $@

# Clean build artifacts
clean: clean_debug clean_release

clean_debug:
	@echo "Cleaning Debug build..."
	@rm -rf Debug

clean_release:
	@echo "Cleaning Release build..."
	@rm -rf Release

# Display build configuration
info:
	@echo "=============================================="
	@echo "IOsonata nRF52832 Library Build Configuration"
	@echo "=============================================="
	@echo "Library Name:     $(LIBRARY_NAME)"
	@echo "Build Variant:    $(VARIANT)"
	@echo "Build Directory:  $(BUILD_DIR)"
	@echo "Target Device:    $(TARGET_DEVICE)"
	@echo "MCU:              $(MCU)"
	@echo ""
	@echo "Toolchain:"
	@echo "  CC:  $(CC)"
	@echo "  CXX: $(CXX)"
	@echo "  AR:  $(AR)"
	@echo ""
	@echo "Source Files:"
	@echo "  Device:   $(words $(DEVICE_SRCS)) files"
	@echo "  nRF52:    $(words $(NRF52_SRCS)) files"
	@echo "  ARM:      $(words $(ARM_COMMON_SRCS)) files"
	@echo "  IOsonata: $(words $(IOSONATA_SRCS)) files"
	@echo "  Modules:  $(words $(IOSONATA_MODULE_SRCS)) files"
	@echo "  Total:    $(words $(ALL_SRCS)) files"
	@echo "=============================================="

# ============================================
# Help
# ============================================
help:
	@echo "IOsonata nRF52832 Library Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  all            - Build library with current VARIANT (default: Release)"
	@echo "  debug          - Build Debug variant"
	@echo "  release        - Build Release variant"
	@echo "  clean          - Clean all build artifacts"
	@echo "  clean_debug    - Clean Debug build only"
	@echo "  clean_release  - Clean Release build only"
	@echo "  info           - Display build configuration"
	@echo "  help           - Show this help message"
	@echo ""
	@echo "Build variants:"
	@echo "  Debug          - Unoptimized with debug symbols (-O0 -g3)"
	@echo "  Release        - Optimized for size (-O3)"
	@echo ""
	@echo "Examples:"
	@echo "  make                    # Build Release variant"
	@echo "  make debug              # Build Debug variant"
	@echo "  make release            # Build Release variant"
	@echo "  make VARIANT=Debug      # Build Debug variant (alternative)"
	@echo "  make clean              # Clean all builds"
	@echo "  make info               # Show configuration"
