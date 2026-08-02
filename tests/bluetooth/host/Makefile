ROOT := ../../..
BUILD_DIR := build
ADV_TARGET := $(BUILD_DIR)/bt_adv_uuid_test
HCI_TARGET := $(BUILD_DIR)/bt_hci_flow_test
HCI_CTLR_TARGET := $(BUILD_DIR)/bt_hci_ctlr_init_test
L2CAP_TARGET := $(BUILD_DIR)/bt_l2cap_signal_test
ATT_TARGET := $(BUILD_DIR)/bt_att_db_test
ATT_REQ_TARGET := $(BUILD_DIR)/bt_att_req_test
PEER_TARGET := $(BUILD_DIR)/bt_peer_pool_test
GAP_HCI_TARGET := $(BUILD_DIR)/bt_gap_hci_test
ADV_HCI_TARGET := $(BUILD_DIR)/bt_adv_hci_test
GATT_CCCD_TARGET := $(BUILD_DIR)/bt_gatt_cccd_test
GATT_SRVC_TARGET := $(BUILD_DIR)/bt_gatt_srvc_test
APP_ML_TARGET := $(BUILD_DIR)/bt_app_multilink_test
SYSLOG_TARGET := $(BUILD_DIR)/syslog_tx_test
GATT_TXC_TARGET := $(BUILD_DIR)/bt_gatt_txcomplete_test
CFIFO_OBJ := $(BUILD_DIR)/cfifo.o

CC ?= gcc
CXX ?= g++
CPPFLAGS += -I$(ROOT)/include
CFLAGS ?= -O1 -g
CFLAGS += -std=gnu11 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections
# C++23 is spelled gnu++23 by gcc and recent clang, gnu++2b by older Apple
# clang. Probe what this compiler accepts rather than hardcoding one form.
CXXSTD := $(shell for s in gnu++23 gnu++2b gnu++20; do \
	if $(CXX) -std=$$s -x c++ -fsyntax-only /dev/null >/dev/null 2>&1; then \
		echo $$s; break; fi; done)
CXXFLAGS ?= -O1 -g
CXXFLAGS += -std=$(CXXSTD) -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections
# Dead-strip unused sections so a target that pulls in a .cpp but exercises only
# part of it does not have to satisfy every symbol the unused code references
# (for example the CFifo calls in bt_hci_ctlr.cpp). The GNU and Apple linkers
# spell this differently; select by host OS.
ifeq ($(shell uname -s),Darwin)
LDFLAGS += -Wl,-dead_strip
else
LDFLAGS += -Wl,--gc-sections
endif
# no-sanitize-recover makes any UBSan finding abort the test run, so a suite
# cannot print PASS after undefined behavior scrolled by above it.
SANITIZERS ?= -fsanitize=address,undefined -fno-sanitize-recover=all

# Headers are prerequisites too. Without them a change to an inline function in
# a header leaves every binary up to date and the suite reruns the old code.
# Only the directories the suites include from, and not include/*.h as a whole:
# one file there has a space in its name, which make cannot express in a
# prerequisite list.
HEADERS := $(wildcard $(ROOT)/include/bluetooth/*.h) \
	$(ROOT)/include/istddef.h \
	$(ROOT)/include/device_intrf.h \
	$(ROOT)/include/cfifo.h \
	$(ROOT)/include/syslog.h

ADV_SOURCES := \
	bt_adv_uuid_test.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

HCI_SOURCES := \
	bt_hci_flow_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

HCI_CTLR_SOURCES := \
	bt_hci_ctlr_init_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

L2CAP_SOURCES := \
	bt_l2cap_signal_test.cpp \
	$(ROOT)/src/bluetooth/bt_l2cap.cpp

ATT_SOURCES := \
	bt_att_db_test.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp

ATT_REQ_SOURCES := \
	bt_att_req_test.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

PEER_SOURCES := \
	bt_peer_pool_test.cpp \
	$(ROOT)/src/bluetooth/bt_peer.cpp

GAP_HCI_SOURCES := \
	bt_gap_hci_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap_hci.cpp

ADV_HCI_SOURCES := \
	bt_adv_hci_test.cpp \
	$(ROOT)/src/bluetooth/bt_adv_hci.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

GATT_CCCD_SOURCES := \
	bt_gatt_cccd_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp

GATT_SRVC_SOURCES := \
	bt_gatt_srvc_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

APP_ML_SOURCES := \
	bt_app_multilink_test.cpp \
	$(ROOT)/src/bluetooth/bt_app.cpp

GATT_TXC_SOURCES := \
	bt_gatt_txcomplete_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_peer.cpp

SYSLOG_SOURCES := \
	syslog_tx_test.cpp \
	$(ROOT)/src/syslog.cpp \
	$(ROOT)/src/device_intrf.cpp

.PHONY: all test clean

all: test

test: $(ADV_TARGET) $(HCI_TARGET) $(HCI_CTLR_TARGET) $(L2CAP_TARGET) $(ATT_TARGET) $(ATT_REQ_TARGET) $(PEER_TARGET) $(GAP_HCI_TARGET) $(ADV_HCI_TARGET) $(GATT_CCCD_TARGET) $(GATT_SRVC_TARGET) $(APP_ML_TARGET) $(SYSLOG_TARGET) $(GATT_TXC_TARGET)
	./$(ADV_TARGET)
	./$(HCI_TARGET)
	./$(HCI_CTLR_TARGET)
	./$(L2CAP_TARGET)
	./$(ATT_TARGET)
	./$(ATT_REQ_TARGET)
	./$(PEER_TARGET)
	./$(GAP_HCI_TARGET)
	./$(ADV_HCI_TARGET)
	./$(GATT_CCCD_TARGET)
	./$(GATT_SRVC_TARGET)
	./$(APP_ML_TARGET)
	./$(SYSLOG_TARGET)
	./$(GATT_TXC_TARGET)

$(ADV_TARGET): $(ADV_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ADV_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(HCI_TARGET): $(HCI_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(HCI_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(CFIFO_OBJ): $(ROOT)/src/cfifo.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $< -o $@

$(HCI_CTLR_TARGET): $(HCI_CTLR_SOURCES) $(CFIFO_OBJ) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(HCI_CTLR_SOURCES) $(CFIFO_OBJ) -o $@ $(LDFLAGS) $(SANITIZERS)

$(L2CAP_TARGET): $(L2CAP_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(L2CAP_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ATT_TARGET): $(ATT_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ATT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ATT_REQ_TARGET): $(ATT_REQ_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ATT_REQ_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(PEER_TARGET): $(PEER_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(PEER_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(GAP_HCI_TARGET): $(GAP_HCI_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(GAP_HCI_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ADV_HCI_TARGET): $(ADV_HCI_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ADV_HCI_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(GATT_CCCD_TARGET): $(GATT_CCCD_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(GATT_CCCD_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(GATT_SRVC_TARGET): $(GATT_SRVC_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(GATT_SRVC_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(APP_ML_TARGET): $(APP_ML_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(APP_ML_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(SYSLOG_TARGET): $(SYSLOG_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(SYSLOG_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)

$(GATT_TXC_TARGET): $(GATT_TXC_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(GATT_TXC_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)
