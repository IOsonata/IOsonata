ROOT := ../../..
BUILD_DIR := build

CC ?= gcc
CXX ?= g++
PYTHON ?= python3
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include -I.
CFLAGS ?= $(OPT) -g
CFLAGS += -std=gnu11 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong

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

HEADERS := $(wildcard $(ROOT)/include/bluetooth/*.h) \
	$(wildcard $(ROOT)/include/bluetooth/services/*.h) \
	$(ROOT)/include/istddef.h \
	$(ROOT)/include/convutil.h \
	$(ROOT)/include/device_intrf.h \
	$(ROOT)/include/cfifo.h \
	$(ROOT)/include/coredev/interrupt.h \
	$(ROOT)/include/syslog.h \
	bt_test_harness.h

TEST_NAMES := \
	app_evt_handler_retry_test \
	cfifo_boundary_test \
	bt_adv_uuid_test \
	bt_intrf_test \
	bt_hci_flow_test \
	bt_hci_smp_tx_test \
	bt_hci_ctlr_init_test \
	bt_hci_phy_test \
	bt_hci_host_feature_test \
	bt_l2cap_signal_test \
	bt_att_db_test \
	bt_att_req_test \
	bt_att_client_transaction_test \
	bt_peer_pool_test \
	bt_gap_hci_test \
	bt_adv_hci_test \
	bt_gatt_cccd_test \
	bt_gatt_srvc_test \
	bt_gap_sec_levels_test \
	bt_adv_periodic_test \
	bt_gatt_hci_security_test \
	bt_app_multilink_test \
	bt_app_init_bm_test \
	bt_lesc_sd_test \
	syslog_tx_test \
	bt_gatt_txcomplete_test \
	bt_att_adversarial_test \
	bt_gatt_adversarial_test \
	bt_hci_adversarial_test \
	bt_l2cap_adversarial_test \
	bt_app_adversarial_test \
	bt_std_services_test \
	bt_smp_sign_counter_test \
	bt_smp_keydist_test \
	bt_smp_pairing_policy_test \
	bt_smp_pubkey_test \
	bt_sec_sd_test \
	bt_gatt_permission_test \
	bt_gap_init_port_test

TARGETS := $(addprefix $(BUILD_DIR)/,$(TEST_NAMES))
CFIFO_OBJ := $(BUILD_DIR)/cfifo.o
CRC_OBJ := $(BUILD_DIR)/crc.o
UECC_OBJ := $(BUILD_DIR)/uECC.o
BT_APP_BM_SOURCE := $(ROOT)/ARM/Nordic/nRF54/src/bt_app_bm.cpp
BT_APP_BM_INIT_INC := generated/bt_app_bm_init.inc
GATT_INIT_TEST_SUPPORT := bt_gatt_init_test_support.cpp
HCI_GATT_TEST_STUB := ../bt_hci_gatt_test_stubs.cpp

app_evt_handler_retry_test_SOURCES := \
	app_evt_handler_retry_test.cpp \
	$(ROOT)/src/app_evt_handler.cpp
app_evt_handler_retry_test_EXTRA := $(CFIFO_OBJ)

cfifo_boundary_test_SOURCES := cfifo_boundary_test.cpp
cfifo_boundary_test_EXTRA := $(CFIFO_OBJ)

bt_adv_uuid_test_SOURCES := \
	bt_adv_uuid_test.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_intrf_test_SOURCES := \
	bt_intrf_test.cpp \
	$(ROOT)/src/bluetooth/bt_intrf.cpp \
	$(ROOT)/src/device_intrf.cpp
bt_intrf_test_EXTRA := $(CFIFO_OBJ)

bt_hci_flow_test_SOURCES := \
	bt_hci_flow_test.cpp \
	$(HCI_GATT_TEST_STUB) \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

bt_hci_smp_tx_test_SOURCES := \
	bt_hci_smp_tx_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp

bt_hci_ctlr_init_test_SOURCES := \
	bt_hci_ctlr_init_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp
bt_hci_ctlr_init_test_EXTRA := $(CFIFO_OBJ)

bt_hci_phy_test_SOURCES := \
	bt_hci_phy_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp
bt_hci_phy_test_EXTRA := $(CFIFO_OBJ)

bt_hci_host_feature_test_SOURCES := \
	bt_hci_host_feature_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp
bt_hci_host_feature_test_EXTRA := $(CFIFO_OBJ)

bt_l2cap_signal_test_SOURCES := \
	bt_l2cap_signal_test.cpp \
	$(ROOT)/src/bluetooth/bt_l2cap.cpp

bt_att_db_test_SOURCES := \
	bt_att_db_test.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp

bt_att_req_test_SOURCES := \
	bt_att_req_test.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_att_client_transaction_test_SOURCES := \
	bt_att_client_transaction_test.cpp \
	$(ROOT)/src/bluetooth/bt_attreq.cpp \
	$(ROOT)/src/bluetooth/bt_attrsp.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_peer_pool_test_SOURCES := \
	bt_peer_pool_test.cpp \
	$(ROOT)/src/bluetooth/bt_peer.cpp

bt_gap_hci_test_SOURCES := \
	bt_gap_hci_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap_hci.cpp

bt_adv_hci_test_SOURCES := \
	bt_adv_hci_test.cpp \
	$(GATT_INIT_TEST_SUPPORT) \
	$(ROOT)/src/bluetooth/bt_adv_hci.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp
bt_adv_hci_test_EXTRA := $(CFIFO_OBJ)

bt_gatt_cccd_test_SOURCES := \
	bt_gatt_cccd_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp

bt_gatt_txcomplete_test_SOURCES := \
	bt_gatt_txcomplete_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_peer.cpp

bt_gap_init_port_test_SOURCES := \
	bt_gap_init_port_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_gatt_permission_test_SOURCES := \
	bt_gatt_permission_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_gatt_srvc_test_SOURCES := \
	bt_gatt_srvc_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_adv_periodic_test_SOURCES := \
	bt_adv_periodic_test.cpp \
	$(GATT_INIT_TEST_SUPPORT) \
	$(ROOT)/src/bluetooth/bt_adv_hci.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp
bt_adv_periodic_test_EXTRA := $(CFIFO_OBJ)

bt_gap_sec_levels_test_SOURCES := \
	bt_gap_sec_levels_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_gatt_hci_security_test_SOURCES := \
	bt_gatt_hci_security_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt_hci.cpp

bt_app_multilink_test_SOURCES := \
	bt_app_multilink_test.cpp \
	$(ROOT)/src/bluetooth/bt_app.cpp

bt_app_init_bm_test_SOURCES := \
	bt_app_init_bm_test.cpp \
	$(GATT_INIT_TEST_SUPPORT) \
	$(ROOT)/src/bluetooth/services/bt_dis.cpp
bt_app_init_bm_test_DEPS := $(BT_APP_BM_INIT_INC)

bt_sec_sd_test_CPPFLAGS := -Inordic_stub -I$(ROOT)/ARM/Nordic/include
bt_sec_sd_test_SOURCES := \
	bt_sec_sd_test.cpp \
	bt_sec_sd_test_stubs.cpp \
	$(ROOT)/ARM/Nordic/nRF52/src/bt_sec_sd.cpp
bt_sec_sd_test_DEPS := \
	$(wildcard nordic_stub/*.h) \
	bt_sec_sd_test_stubs.h \
	$(ROOT)/ARM/Nordic/include/bt_lesc.h

bt_lesc_sd_test_CPPFLAGS := -Inordic_stub -I$(ROOT)/ARM/Nordic/include
bt_lesc_sd_test_SOURCES := \
	bt_lesc_sd_test.cpp \
	$(ROOT)/ARM/Nordic/src/bt_lesc_sd.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp
bt_lesc_sd_test_DEPS := \
	$(wildcard nordic_stub/*.h) \
	$(ROOT)/ARM/Nordic/include/bt_lesc.h

syslog_tx_test_SOURCES := \
	syslog_tx_test.cpp \
	$(ROOT)/src/syslog.cpp \
	$(ROOT)/src/device_intrf.cpp

bt_att_adversarial_test_SOURCES := \
	bt_att_adversarial_test.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_gatt_adversarial_test_SOURCES := \
	bt_gatt_adversarial_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_hci_adversarial_test_SOURCES := \
	bt_hci_adversarial_test.cpp \
	$(HCI_GATT_TEST_STUB) \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

bt_l2cap_adversarial_test_SOURCES := \
	bt_l2cap_adversarial_test.cpp \
	$(ROOT)/src/bluetooth/bt_l2cap.cpp

bt_app_adversarial_test_SOURCES := \
	bt_app_adversarial_test.cpp \
	$(ROOT)/src/bluetooth/bt_app.cpp

bt_std_services_test_SOURCES := \
	bt_std_services_test.cpp \
	$(GATT_INIT_TEST_SUPPORT) \
	$(ROOT)/src/bluetooth/services/bt_dis.cpp \
	$(ROOT)/src/bluetooth/services/bt_bas.cpp

bt_smp_keydist_test_SOURCES := \
	bt_smp_keydist_test.cpp \
	bt_smp_link_test_stubs.cpp \
	$(ROOT)/src/device.cpp
bt_smp_keydist_test_EXTRA := $(CRC_OBJ)
bt_smp_keydist_test_DEPS := \
	bt_smp_link_test_stubs.h \
	$(ROOT)/src/bluetooth/bt_smp.cpp

bt_smp_pairing_policy_test_SOURCES := \
	bt_smp_pairing_policy_test.cpp \
	bt_smp_link_test_stubs.cpp \
	$(ROOT)/src/device.cpp
bt_smp_pairing_policy_test_EXTRA := $(CRC_OBJ)
bt_smp_pairing_policy_test_DEPS := \
	bt_smp_link_test_stubs.h \
	$(ROOT)/src/bluetooth/bt_smp.cpp

bt_smp_pubkey_test_SOURCES := \
	bt_smp_pubkey_test.cpp \
	bt_smp_link_test_stubs.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/crypto/crypto_uecc.cpp \
	$(ROOT)/src/crypto/crypto_softaes.cpp \
	$(ROOT)/src/crypto/crypto_softrng.cpp \
	$(ROOT)/src/crypto/crypto_p256.cpp
bt_smp_pubkey_test_EXTRA := $(CRC_OBJ) $(UECC_OBJ)
bt_smp_pubkey_test_CPPFLAGS := -I$(ROOT)/micro-ecc
bt_smp_pubkey_test_DEPS := \
	bt_smp_link_test_stubs.h \
	$(ROOT)/src/bluetooth/bt_smp.cpp

bt_smp_sign_counter_test_SOURCES := \
	bt_smp_sign_counter_test.cpp \
	$(ROOT)/src/bluetooth/bt_smp_bond.cpp

.PHONY: all test test-gcc test-clang test-matrix clean

all: test

test: $(TARGETS)
	@set -e; \
	for target in $(TARGETS); do \
		echo "RUN $$target"; \
		$(TEST_ENV) ./$$target; \
	done

test-gcc:
	$(MAKE) clean
	$(MAKE) test CC=gcc CXX=g++

test-clang:
	$(MAKE) clean
	$(MAKE) test CC=clang CXX=clang++

test-matrix: test-gcc test-clang

$(BT_APP_BM_INIT_INC): extract_bt_app_init.py $(BT_APP_BM_SOURCE)
	$(PYTHON) extract_bt_app_init.py $(BT_APP_BM_SOURCE) $@

$(CFIFO_OBJ): $(ROOT)/src/cfifo.c $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $< -o $@

$(CRC_OBJ): $(ROOT)/src/crc.c $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $< -o $@

# micro-ecc is vendored third party C. Its own rule so it is compiled as C
# rather than through a _SOURCES list, and without the project warning set it
# was not written against.
$(UECC_OBJ): $(ROOT)/micro-ecc/uECC.c
	@mkdir -p $(BUILD_DIR)
	$(CC) -I$(ROOT)/micro-ecc $(OPT) -g -std=gnu11 -fno-omit-frame-pointer \
		$(SANITIZERS) -c $< -o $@

define BUILD_TEST
$(BUILD_DIR)/$(1): $$($(1)_SOURCES) $$($(1)_EXTRA) $$($(1)_DEPS) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $$($(1)_CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) \
		$$($(1)_SOURCES) $$($(1)_EXTRA) -o $$@ \
		$(LDFLAGS) $(SANITIZERS)
endef

$(foreach test,$(TEST_NAMES),$(eval $(call BUILD_TEST,$(test))))

clean:
	rm -rf $(BUILD_DIR) generated
