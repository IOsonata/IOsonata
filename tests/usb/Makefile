ROOT := ../..
BUILD_DIR := build

CXX ?= g++
CC ?= gcc
OPT ?= -O1

# hostport supplies usb_ctrlr.h, the header every real port provides.
CPPFLAGS += -I$(ROOT)/include -Ihostport

CXXFLAGS ?= $(OPT) -g
CXXFLAGS += -std=gnu++17 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong \
	-D_GLIBCXX_ASSERTIONS
CFLAGS ?= $(OPT) -g
CFLAGS += -std=gnu11 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong

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
	$(ROOT)/src/cfifo.c

INTRF_TARGET := $(BUILD_DIR)/usb_intrf_test
INTRF_SOURCES := \
	usb_intrf_test.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

HEADERS := \
	hostport/usb_ctrlr.h \
	$(ROOT)/include/usb/usb.h \
	$(ROOT)/include/usb/usb_def.h \
	$(ROOT)/include/usb/usb_intrf.h

TARGETS := $(CORE_TARGET) $(INTRF_TARGET) $(LINK_TARGET)

.PHONY: all test test-gcc test-clang test-matrix clean

all: test

test: $(TARGETS)
	$(TEST_ENV) ./$(CORE_TARGET)
	$(TEST_ENV) ./$(INTRF_TARGET)
	$(TEST_ENV) ./$(LINK_TARGET)

test-gcc:
	$(MAKE) clean
	$(MAKE) test CXX=g++ CC=gcc

test-clang:
	$(MAKE) clean
	$(MAKE) test CXX=clang++ CC=clang

test-matrix: test-gcc test-clang

$(CORE_TARGET): $(CORE_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(CORE_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(INTRF_TARGET): $(INTRF_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(INTRF_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(LINK_TARGET): $(LINK_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(LINK_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
