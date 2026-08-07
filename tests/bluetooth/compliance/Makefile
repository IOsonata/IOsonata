ROOT := ../../..
BUILD_DIR := build
HCI_TARGET := $(BUILD_DIR)/bt_hci_compliance_test
CAP_TARGET := $(BUILD_DIR)/bt_hci_capability_test
ADV_TARGET := $(BUILD_DIR)/bt_adv_hci_test
GAP_TARGET := $(BUILD_DIR)/bt_gap_hci_test
PHY_TARGET := $(BUILD_DIR)/bt_hci_phy_test
SMP_TARGET := $(BUILD_DIR)/bt_smp_dual_host_test
ATT_TARGET := $(BUILD_DIR)/bt_att_dual_host_test
TARGETS := $(HCI_TARGET) $(CAP_TARGET) $(ADV_TARGET) $(GAP_TARGET) $(PHY_TARGET) $(SMP_TARGET) $(ATT_TARGET)

CC ?= gcc
CXX ?= g++
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include -Iframework -Icontroller -Ipeer -Ilink -Ioob
SMP_CPPFLAGS := $(CPPFLAGS) -include smp/bt_l2cap_test_override.h
SMP_CORE_DEFINES := -DBtSmpInit=BtSmpInitReal
SMP_TEST_RENAMES := \
	-DBtSmpLocalIrkGet=BtSmpTestLocalIrkGet \
	-DBtSmpLocalIdRecordSize=BtSmpTestLocalIdRecordSize \
	-DBtSmpLocalIdSerialize=BtSmpTestLocalIdSerialize \
	-DBtSmpLocalIdRestore=BtSmpTestLocalIdRestore
ATT_TEST_RENAMES := -Dmain=BtAttDualHostMain
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

HCI_SOURCES := \
	hci/bt_hci_compliance_test.cpp \
	../bt_hci_gatt_test_stubs.cpp \
	framework/bt_compliance.cpp \
	framework/bt_virtual_clock.cpp \
	controller/bt_virtual_controller.cpp \
	peer/bt_virtual_peer.cpp \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

CAP_SOURCES := \
	hci/bt_hci_capability_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

ADV_SOURCES := \
	../host/bt_adv_hci_test.cpp \
	../host/bt_gatt_init_test_support.cpp \
	$(ROOT)/src/bluetooth/bt_adv_hci.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

GAP_SOURCES := \
	../host/bt_gap_hci_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap_hci.cpp

PHY_SOURCES := \
	../host/bt_hci_phy_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

ATT_SOURCES := \
	att/bt_att_dual_host_test.cpp \
	att/bt_att_client_test_stubs.cpp \
	framework/bt_compliance.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_attreq.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

SMP_TEST_OBJ := $(BUILD_DIR)/smp/bt_smp_dual_host_test.o
SMP_FRAMEWORK_OBJ := $(BUILD_DIR)/smp/bt_compliance.o
SMP_CORE_OBJ := $(BUILD_DIR)/smp/bt_smp.o
SMP_AES_OBJ := $(BUILD_DIR)/smp/crypto_softaes.o
SMP_DEVICE_OBJ := $(BUILD_DIR)/smp/device.o
SMP_DEVICE_INTRF_OBJ := $(BUILD_DIR)/smp/device_intrf.o
SMP_STUB_OBJ := $(BUILD_DIR)/smp/bt_smp_test_stubs.o
SMP_OBJECTS := $(SMP_TEST_OBJ) $(SMP_FRAMEWORK_OBJ) $(SMP_CORE_OBJ) \
	$(SMP_AES_OBJ) $(SMP_DEVICE_OBJ) $(SMP_DEVICE_INTRF_OBJ) $(SMP_STUB_OBJ)

HEADERS := \
	$(wildcard framework/*.h) \
	$(wildcard controller/*.h) \
	$(wildcard peer/*.h) \
	$(wildcard link/*.h) \
	$(wildcard oob/*.h) \
	$(wildcard att/*.h) \
	$(wildcard smp/*.h) \
	$(wildcard $(ROOT)/include/bluetooth/*.h) \
	$(wildcard $(ROOT)/include/crypto/*.h) \
	$(ROOT)/include/device.h \
	$(ROOT)/include/device_intrf.h \
	$(ROOT)/include/istddef.h \
	$(ROOT)/include/syslog.h

.PHONY: all test strict clean

all: test

test: $(TARGETS)
	@set -e; \
	for target in $(TARGETS); do \
		echo "RUN $$target"; \
		$(TEST_ENV) ./$$target; \
	done

strict: $(TARGETS)
	BT_COMPLIANCE_STRICT=1 $(TEST_ENV) ./$(HCI_TARGET)
	$(TEST_ENV) ./$(CAP_TARGET)
	$(TEST_ENV) ./$(ADV_TARGET)
	$(TEST_ENV) ./$(GAP_TARGET)
	$(TEST_ENV) ./$(PHY_TARGET)
	$(TEST_ENV) ./$(SMP_TARGET)
	$(TEST_ENV) ./$(ATT_TARGET)

# cfifo is C, not C++: it uses C11 atomics that a C++ compiler rejects. The
# controller layer needs it, so it is built as C and linked in.
CFIFO_OBJ := $(BUILD_DIR)/cfifo.o
CFLAGS ?= $(OPT) -g
CFLAGS += -std=gnu11 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong

$(CFIFO_OBJ): $(ROOT)/src/cfifo.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $< -o $@

$(HCI_TARGET): $(HCI_SOURCES) $(HEADERS) $(CFIFO_OBJ)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(HCI_SOURCES) $(CFIFO_OBJ) -o $@ $(LDFLAGS) $(SANITIZERS)

$(CAP_TARGET): $(CAP_SOURCES) $(HEADERS) $(CFIFO_OBJ)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(CAP_SOURCES) $(CFIFO_OBJ) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ADV_TARGET): $(ADV_SOURCES) $(HEADERS) $(CFIFO_OBJ)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(ADV_SOURCES) $(CFIFO_OBJ) -o $@ $(LDFLAGS) $(SANITIZERS)

$(GAP_TARGET): $(GAP_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(GAP_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(PHY_TARGET): $(PHY_SOURCES) $(HEADERS) $(CFIFO_OBJ)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$(PHY_SOURCES) $(CFIFO_OBJ) -o $@ $(LDFLAGS) $(SANITIZERS)

$(ATT_TARGET): $(ATT_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(ATT_TEST_RENAMES) $(CXXFLAGS) $(SANITIZERS) \
		$(ATT_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(SMP_TEST_OBJ): smp/bt_smp_dual_host_test.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(SMP_CPPFLAGS) $(SMP_TEST_RENAMES) $(CXXFLAGS) $(SANITIZERS) \
		-c $< -o $@

$(SMP_FRAMEWORK_OBJ): framework/bt_compliance.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(SMP_CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -c $< -o $@

$(SMP_CORE_OBJ): $(ROOT)/src/bluetooth/bt_smp.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(SMP_CPPFLAGS) $(SMP_CORE_DEFINES) $(CXXFLAGS) $(SANITIZERS) -c $< -o $@

$(SMP_AES_OBJ): $(ROOT)/src/crypto/crypto_softaes.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -c $< -o $@

$(SMP_DEVICE_OBJ): $(ROOT)/src/device.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(SMP_CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -c $< -o $@

$(SMP_DEVICE_INTRF_OBJ): $(ROOT)/src/device_intrf.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(SMP_CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -c $< -o $@

$(SMP_STUB_OBJ): smp/bt_smp_test_stubs.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(SMP_CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -c $< -o $@

$(SMP_TARGET): $(SMP_OBJECTS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(SMP_OBJECTS) -o $@ $(LDFLAGS) $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
