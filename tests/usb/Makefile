ROOT := ../..
BUILD_DIR := build

CC ?= cc
CXX ?= g++
OPT ?= -O1

# hostport supplies usb_ctrlr.h, the header every real port provides.
CPPFLAGS += -I$(ROOT)/include -Ihostport

CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -Wshadow -Wundef
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

CORE_TARGET := $(BUILD_DIR)/usb_core_test
CORE_SOURCES := \
	usb_core_test.cpp \
	$(ROOT)/src/usb/usb.cpp \
	$(ROOT)/src/usb/usbd_cdc_desc.cpp

LINK_TARGET := $(BUILD_DIR)/usb_stack_link_test
LINK_SOURCES := \
	usb_stack_link_test.cpp \
	$(ROOT)/src/usb/usb.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/usb/usbd_cdc.cpp \
	$(ROOT)/src/usb/usbd_cdc_desc.cpp \
	$(ROOT)/src/device_intrf.cpp \
	$(ROOT)/src/cfifo.c

BULK_TARGET := $(BUILD_DIR)/usbd_bulk_test
BULK_SOURCES := \
	usbd_bulk_test.cpp \
	$(ROOT)/src/usb/usbd_bulk.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/device_intrf.cpp \
	$(ROOT)/src/cfifo.c

INTRF_TARGET := $(BUILD_DIR)/usb_intrf_test
INTRF_SOURCES := \
	usb_intrf_test.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

RX_EVENT_TARGET := $(BUILD_DIR)/usb_rx_event_test
RX_EVENT_SOURCES := \
	usb_rx_event_test.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

DIRECT_TARGET := $(BUILD_DIR)/usb_direct_intrf_test
DIRECT_SOURCES := \
	usb_direct_intrf_test.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

ISO_TARGET := $(BUILD_DIR)/usb_iso_intrf_test
ISO_SOURCES := \
	usb_iso_intrf_test.cpp \
	$(ROOT)/src/usb/usb_iso.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

ISO_DUPLEX_TARGET := $(BUILD_DIR)/usb_iso_intrf_duplex_test
ISO_DUPLEX_SOURCES := \
	usb_iso_intrf_duplex_test.cpp \
	$(ROOT)/src/usb/usb_iso.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

INT_TARGET := $(BUILD_DIR)/usb_int_intrf_test
INT_SOURCES := \
	usb_int_intrf_test.cpp \
	$(ROOT)/src/usb/usb_int.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

# Physical hardware validation. The Python runner uses python-libusb1 so it
# still submits real asynchronous isochronous transfers through libusb.
ISO_HW_SCRIPT := $(ROOT)/Python/usb_iso_loopback.py
ISO_HW_ARGS ?= --manual-suspend-wake

# Throughput measurement, not a regression test. Built and run on demand.
# Sanitizers are left off so the numbers are not distorted.
PROBE_TARGET := $(BUILD_DIR)/tx_rate_probe
PROBE_SOURCES := \
	tx_rate_probe.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

SOF_TICKS ?= 500

HEADERS := \
	hostport/usb_ctrlr.h \
	$(ROOT)/include/usb/usb.h \
	$(ROOT)/include/usb/usb_def.h \
	$(ROOT)/include/usb/usb_intrf.h \
	$(ROOT)/include/usb/usb_int.h \
	$(ROOT)/include/usb/usb_iso.h \
	$(ROOT)/include/usb/usbd_bulk.h \
	$(ROOT)/include/usb/usbd_cdc.h \
	$(ROOT)/src/usb/usb_func.h

TARGETS := $(CORE_TARGET) $(INTRF_TARGET) $(RX_EVENT_TARGET) \
	$(DIRECT_TARGET) $(ISO_TARGET) $(ISO_DUPLEX_TARGET) $(INT_TARGET) \
	$(BULK_TARGET) $(LINK_TARGET)

.PHONY: all test test-c-header test-gcc test-clang test-matrix \
	test-nrf-suspend-policy test-nrf-iso-policy test-nrf-int-policy \
	iso-hw-build iso-hw probe clean

all: test

test: $(TARGETS)
	$(MAKE) test-c-header
	$(MAKE) test-nrf-suspend-policy
	$(MAKE) test-nrf-iso-policy
	$(MAKE) test-nrf-int-policy
	$(TEST_ENV) ./$(CORE_TARGET)
	$(TEST_ENV) ./$(INTRF_TARGET)
	$(TEST_ENV) ./$(RX_EVENT_TARGET)
	$(TEST_ENV) ./$(DIRECT_TARGET)
	$(TEST_ENV) ./$(ISO_TARGET)
	$(TEST_ENV) ./$(ISO_DUPLEX_TARGET)
	$(TEST_ENV) ./$(INT_TARGET)
	$(TEST_ENV) ./$(BULK_TARGET)
	$(TEST_ENV) ./$(LINK_TARGET)

test-c-header:
	@printf '%s\n' '#include "usb/usb_iso.h"' '#include "usb/usb_int.h"' \
		'int main(void) {' \
		' return UsbIsoIntrfTxReady((const UsbIsoIntrf_t *)0) ||' \
		'  UsbIntIntrfTxReady((const UsbIntIntrf_t *)0) ? 1 : 0;' '}' | \
		$(CC) $(CPPFLAGS) $(CFLAGS) -x c -fsyntax-only -

test-nrf-suspend-policy:
	python3 ./nrf_usb_suspend_policy_test.py

test-nrf-iso-policy:
	python3 ./nrf_usb_iso_policy_test.py

test-nrf-int-policy:
	python3 ./nrf_usb_int_policy_test.py

iso-hw-build: $(ISO_HW_SCRIPT)
	python3 -c "compile(open('$(ISO_HW_SCRIPT)', encoding='utf-8').read(), '$(ISO_HW_SCRIPT)', 'exec')"

iso-hw: iso-hw-build
	python3 $(ISO_HW_SCRIPT) $(ISO_HW_ARGS)

probe: $(PROBE_TARGET)
	./$(PROBE_TARGET) $(SOF_TICKS)

test-gcc:
	$(MAKE) clean
	$(MAKE) test CXX=g++

test-clang:
	$(MAKE) clean
	$(MAKE) test CXX=clang++

test-matrix: test-gcc test-clang

$(CORE_TARGET): $(CORE_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(CORE_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(INTRF_TARGET): $(INTRF_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(INTRF_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(RX_EVENT_TARGET): $(RX_EVENT_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(RX_EVENT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(DIRECT_TARGET): $(DIRECT_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(DIRECT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ISO_TARGET): $(ISO_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(ISO_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ISO_DUPLEX_TARGET): $(ISO_DUPLEX_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(ISO_DUPLEX_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(INT_TARGET): $(INT_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(INT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(BULK_TARGET): $(BULK_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(BULK_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(LINK_TARGET): $(LINK_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(LINK_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(PROBE_TARGET): $(PROBE_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -x c++ \
		$(PROBE_SOURCES) -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
