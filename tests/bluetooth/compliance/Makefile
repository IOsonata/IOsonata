ROOT := ../../..
BUILD_DIR := build
TARGET := $(BUILD_DIR)/bt_hci_compliance_test

CC ?= gcc
CXX ?= g++
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include -Iframework -Icontroller -Ipeer
CXXSTD := $(shell for s in gnu++23 gnu++2b gnu++20; do \
	if $(CXX) -std=$$s -x c++ -fsyntax-only /dev/null >/dev/null 2>&1; then \
		echo $$s; break; fi; done)
CXXFLAGS ?= $(OPT) -g
CXXFLAGS += -std=$(CXXSTD) -Wall -Wextra -fno-omit-frame-pointer \
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
	hci/bt_hci_compliance_test.cpp \
	framework/bt_compliance.cpp \
	framework/bt_virtual_clock.cpp \
	controller/bt_virtual_controller.cpp \
	peer/bt_virtual_peer.cpp \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

HEADERS := \
	$(wildcard framework/*.h) \
	$(wildcard controller/*.h) \
	$(wildcard peer/*.h) \
	$(wildcard $(ROOT)/include/bluetooth/*.h) \
	$(ROOT)/include/istddef.h \
	$(ROOT)/include/syslog.h

.PHONY: all test strict clean

all: test

test: $(TARGET)
	$(TEST_ENV) ./$(TARGET)

strict: $(TARGET)
	BT_COMPLIANCE_STRICT=1 $(TEST_ENV) ./$(TARGET)

# cfifo is C, not C++: it uses C11 atomics that a C++ compiler rejects. The
# controller layer needs it, so it is built as C and linked in.
CFIFO_OBJ := $(BUILD_DIR)/cfifo.o
CFLAGS ?= $(OPT) -g
CFLAGS += -std=gnu11 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong

$(CFIFO_OBJ): $(ROOT)/src/cfifo.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $< -o $@

$(TARGET): $(SOURCES) $(HEADERS) $(CFIFO_OBJ)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(SOURCES) $(CFIFO_OBJ) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
