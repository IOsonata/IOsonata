ROOT := ../..
BUILD_DIR := build

CXX ?= g++
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include -Ihostport
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

RX_EVENT_TARGET := $(BUILD_DIR)/usb_rx_event_test
RX_EVENT_SOURCES := \
	usb_rx_event_test.cpp \
	$(ROOT)/src/usb/usb_intrf.cpp \
	$(ROOT)/src/cfifo.c

.PHONY: all test test-gcc test-clang test-matrix clean

all: test

test: $(RX_EVENT_TARGET)
	$(TEST_ENV) ./$(RX_EVENT_TARGET)

test-gcc:
	$(MAKE) clean
	$(MAKE) test CXX=g++

test-clang:
	$(MAKE) clean
	$(MAKE) test CXX=clang++

test-matrix: test-gcc test-clang

$(RX_EVENT_TARGET): $(RX_EVENT_SOURCES) hostport/usb_ctrlr.h \
	$(ROOT)/include/usb/usb_intrf.h $(ROOT)/include/cfifo.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(RX_EVENT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
