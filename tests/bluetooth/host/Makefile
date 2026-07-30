ROOT := ../../..
BUILD_DIR := build
ADV_TARGET := $(BUILD_DIR)/bt_adv_uuid_test
HCI_TARGET := $(BUILD_DIR)/bt_hci_flow_test

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

.PHONY: all test clean

all: test

test: $(ADV_TARGET) $(HCI_TARGET)
	./$(ADV_TARGET)
	./$(HCI_TARGET)

$(ADV_TARGET): $(ADV_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(ADV_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(HCI_TARGET): $(HCI_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(HCI_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
