ROOT := ../..
BUILD_DIR := build

CXX ?= g++
CC ?= gcc
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include

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

IPC_TARGET := $(BUILD_DIR)/ipc_intrf_test
TSAN_TARGET := $(BUILD_DIR)/ipc_intrf_test_tsan

IPC_SOURCES := \
	ipc_intrf_test.cpp \
	$(ROOT)/src/ipc/ipc_intrf.cpp \
	$(ROOT)/src/device_intrf.cpp \
	$(ROOT)/src/cfifo.c

HEADERS := \
	$(ROOT)/include/ipc/ipc_intrf.h \
	$(ROOT)/include/cfifo.h \
	$(ROOT)/include/device_intrf.h

BYTES ?= 4194304
TSAN_BYTES ?= 524288

.PHONY: all test test-tsan test-fw clean

all: test

# Address and undefined behavior sanitizers
test: $(IPC_TARGET) test-fw
	$(TEST_ENV) ./$(IPC_TARGET) $(BYTES)

# Race detection. Fewer bytes because the sanitizer is slow. The thread
# sanitizer cannot be combined with the address sanitizer.
test-tsan: $(TSAN_TARGET)
	TSAN_OPTIONS=halt_on_error=1 ./$(TSAN_TARGET) $(TSAN_BYTES)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(IPC_TARGET): $(IPC_SOURCES) $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $(ROOT)/src/cfifo.c -o $(BUILD_DIR)/cfifo.o
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		ipc_intrf_test.cpp $(ROOT)/src/ipc/ipc_intrf.cpp \
		$(ROOT)/src/device_intrf.cpp $(BUILD_DIR)/cfifo.o \
		$(LDFLAGS) -pthread -o $@

$(TSAN_TARGET): $(IPC_SOURCES) $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fsanitize=thread -c $(ROOT)/src/cfifo.c -o $(BUILD_DIR)/cfifo_tsan.o
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fsanitize=thread \
		ipc_intrf_test.cpp $(ROOT)/src/ipc/ipc_intrf.cpp \
		$(ROOT)/src/device_intrf.cpp $(BUILD_DIR)/cfifo_tsan.o \
		$(LDFLAGS) -pthread -o $@

# The IpcIntrf test firmware (exemples/ipc/ipc_intrf_test.cpp) run on the host
# in packet and byte mode, with one message in flight and with a full window.
FW_DIR := fw
FW_COUNT ?= 30000
FW_CASES := pkt_w1 pkt_w6 pkt_w12 byte_w1 byte_w6
FW_FLAGS_pkt_w1 := -DIPCTEST_PACKET=1 -DIPCTEST_WINDOW=1U
FW_FLAGS_pkt_w6 := -DIPCTEST_PACKET=1 -DIPCTEST_WINDOW=6U
FW_FLAGS_pkt_w12 := -DIPCTEST_PACKET=1 -DIPCTEST_WINDOW=12U
FW_FLAGS_byte_w1 := -DIPCTEST_PACKET=0 -DIPCTEST_WINDOW=1U
FW_FLAGS_byte_w6 := -DIPCTEST_PACKET=0 -DIPCTEST_WINDOW=6U
FW_SRC := $(FW_DIR)/fw_logic_test.cpp $(FW_DIR)/owner_tu.cpp $(FW_DIR)/peer_tu.cpp \
	$(ROOT)/exemples/ipc/ipc_intrf_test.cpp $(FW_DIR)/board.h $(FW_DIR)/nrf.h \
	$(FW_DIR)/idelay.h $(FW_DIR)/iopinctrl.h $(HEADERS) \
	$(ROOT)/tests/bluetooth/host/ipc_sync_port.h

test-fw: $(addprefix $(BUILD_DIR)/fw_,$(FW_CASES))
	@for c in $(FW_CASES); do \
		echo "ipc_intrf_test firmware, host, $$c"; \
		$(TEST_ENV) ./$(BUILD_DIR)/fw_$$c | tail -n 3 || exit 1; \
	done

$(BUILD_DIR)/fw_%: $(FW_SRC) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $(ROOT)/src/cfifo.c -o $(BUILD_DIR)/cfifo_fw.o
	$(CXX) -I$(FW_DIR) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		-DIPCTEST_COUNT=$(FW_COUNT)UL $(FW_FLAGS_$*) \
		$(FW_DIR)/fw_logic_test.cpp $(FW_DIR)/owner_tu.cpp $(FW_DIR)/peer_tu.cpp \
		$(ROOT)/src/ipc/ipc_intrf.cpp $(ROOT)/src/device_intrf.cpp \
		$(BUILD_DIR)/cfifo_fw.o $(LDFLAGS) -o $@

clean:
	rm -rf $(BUILD_DIR)
