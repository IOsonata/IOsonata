ROOT := ../../..
BUILD_DIR := build
ADV_TARGET := $(BUILD_DIR)/bt_adv_uuid_test
HCI_TARGET := $(BUILD_DIR)/bt_hci_flow_test
HCI_CTLR_TARGET := $(BUILD_DIR)/bt_hci_ctlr_init_test
L2CAP_TARGET := $(BUILD_DIR)/bt_l2cap_signal_test
ATT_TARGET := $(BUILD_DIR)/bt_att_db_test

CXX ?= g++
CPPFLAGS += -I$(ROOT)/include
CXXFLAGS ?= -O1 -g
CXXFLAGS += -std=gnu++23 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections
SANITIZERS ?= -fsanitize=address,undefined

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
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp \
	$(ROOT)/src/cfifo.c

L2CAP_SOURCES := \
	bt_l2cap_signal_test.cpp \
	$(ROOT)/src/bluetooth/bt_l2cap.cpp

ATT_SOURCES := \
	bt_att_db_test.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp

.PHONY: all test clean

all: test

test: $(ADV_TARGET) $(HCI_TARGET) $(HCI_CTLR_TARGET) $(L2CAP_TARGET) $(ATT_TARGET)
	./$(ADV_TARGET)
	./$(HCI_TARGET)
	./$(HCI_CTLR_TARGET)
	./$(L2CAP_TARGET)
	./$(ATT_TARGET)

$(ADV_TARGET): $(ADV_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ADV_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(HCI_TARGET): $(HCI_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(HCI_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(HCI_CTLR_TARGET): $(HCI_CTLR_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(HCI_CTLR_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(L2CAP_TARGET): $(L2CAP_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(L2CAP_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ATT_TARGET): $(ATT_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ATT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
