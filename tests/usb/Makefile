ROOT := ../..
BUILD_DIR := build
TARGET := $(BUILD_DIR)/usbd_core_test
INTRF_TARGET := $(BUILD_DIR)/usb_intrf_test
NRF52_TARGET := $(BUILD_DIR)/usbd_ctrlr_nrf52_test

CXX ?= g++
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include
CXXFLAGS ?= $(OPT) -g
CXXFLAGS += -std=gnu++17 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong \
	-D_GLIBCXX_ASSERTIONS

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
LDFLAGS += -Wl,-dead_strip
ASAN_RUNTIME_OPTIONS := detect_leaks=0:strict_string_checks=1
else
LDFLAGS += -Wl,--gc-sections
ASAN_RUNTIME_OPTIONS := detect_leaks=1:strict_string_checks=1
endif

SANITIZERS ?= -fsanitize=address,undefined -fno-sanitize-recover=all
TEST_ENV := ASAN_OPTIONS=$(ASAN_RUNTIME_OPTIONS) \
	UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1

SOURCES := \
	usbd_core_test.cpp \
	$(ROOT)/src/usb/usbd_core.cpp

INTRF_SOURCES := \
	usb_intrf_test.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/device_intrf.cpp \
	$(ROOT)/src/cfifo.c

NRF52_SOURCES := \
	usbd_ctrlr_nrf52_test.cpp \
	$(ROOT)/ARM/Nordic/nRF52/src/usbd_ctrlr_nrf52.cpp

HEADERS := \
	$(ROOT)/include/usb/usbd.h \
	$(ROOT)/include/usb/usb_def.h \
	$(ROOT)/include/usb/usbd_ctrlr.h \
	$(ROOT)/include/usb/usbd_core.h

.PHONY: all test test-gcc test-clang test-matrix clean

all: test

test: $(TARGET) $(INTRF_TARGET) $(NRF52_TARGET)
	$(TEST_ENV) ./$(TARGET)
	$(TEST_ENV) ./$(INTRF_TARGET)
	$(TEST_ENV) ./$(NRF52_TARGET)

test-gcc:
	$(MAKE) clean
	$(MAKE) test CXX=g++

test-clang:
	$(MAKE) clean
	$(MAKE) test CXX=clang++

test-matrix: test-gcc test-clang

$(TARGET): $(SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(INTRF_TARGET): $(INTRF_SOURCES) $(ROOT)/include/usb/usb_intrf.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(INTRF_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(NRF52_TARGET): $(NRF52_SOURCES) $(ROOT)/include/usb/usbd_ctrlr.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) -Inrf52_stub $(CXXFLAGS) $(SANITIZERS) \
		$(NRF52_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS) -pthread

clean:
	rm -rf $(BUILD_DIR)
