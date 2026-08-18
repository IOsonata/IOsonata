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
	bt_hci_ctlr_sdc_test \
	bt_l2cap_signal_test \
	bt_att_db_test \
	bt_att_req_test \
	bt_att_client_transaction_test \
	bt_peer_pool_test \
	bt_gap_hci_test \
	bt_gap_srvc_test \
	bt_gap_gatt_init_test \
	bt_ead_test \
	bt_adv_hci_test \
	bt_padv_hci_test \
	bt_psync_test \
	bt_gatt_cccd_test \
	bt_gatt_srvc_test \
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
	bt_smp_keydist_test

TARGETS := $(addprefix $(BUILD_DIR)/,$(TEST_NAMES))
CFIFO_OBJ := $(BUILD_DIR)/cfifo.o
CRC_OBJ := $(BUILD_DIR)/crc.o
BT_APP_BM_SOURCE := $(ROOT)/ARM/Nordic/nRF54/src/bt_app_bm.cpp
BT_APP_BM_INIT_INC := generated/bt_app_bm_init.inc
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
	$(ROOT)/src/bluetooth/bt_ead.cpp \
	$(ROOT)/src/crypto/crypto_softaes.cpp \
	$(ROOT)/src/crypto/crypto_softrng.cpp \
	$(ROOT)/src/crypto/crypto_p256.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp \
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
	$(ROOT)/src/bluetooth/bt_psync.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

bt_hci_smp_tx_test_SOURCES := \
	bt_hci_smp_tx_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_host.cpp \
	$(ROOT)/src/bluetooth/bt_psync.cpp

bt_hci_ctlr_init_test_SOURCES := \
	bt_hci_ctlr_init_test.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp
bt_hci_ctlr_init_test_EXTRA := $(CFIFO_OBJ)

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
	$(ROOT)/src/bluetooth/bt_adv_hci.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_ead.cpp \
	$(ROOT)/src/crypto/crypto_p256.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_padv_hci_test_SOURCES := \
	bt_padv_hci_test.cpp \
	$(ROOT)/src/bluetooth/bt_padv_hci.cpp

bt_ead_test_SOURCES := \
	bt_ead_test.cpp \
	$(ROOT)/src/bluetooth/bt_ead.cpp \
	$(ROOT)/src/crypto/crypto_softaes.cpp \
	$(ROOT)/src/crypto/crypto_softrng.cpp \
	$(ROOT)/src/crypto/crypto_p256.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp

bt_gap_srvc_test_SOURCES := \
	bt_gap_srvc_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap.cpp

# The only place bt_gap.cpp is linked against the real GATT server. Everything
# else that touches bt_gap.cpp stubs BtGattSrvcAdd or BtGattCharSetValue, and
# a stub that is more permissive than the real function hides an init failure.
bt_gap_gatt_init_test_SOURCES := \
	bt_gap_gatt_init_test.cpp \
	$(ROOT)/src/bluetooth/bt_gap.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_att.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

bt_psync_test_SOURCES := \
	bt_psync_test.cpp \
	$(ROOT)/src/bluetooth/bt_psync.cpp \
	$(ROOT)/src/bluetooth/bt_psync_hci.cpp

bt_gatt_cccd_test_SOURCES := \
	bt_gatt_cccd_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp

bt_gatt_txcomplete_test_SOURCES := \
	bt_gatt_txcomplete_test.cpp \
	$(ROOT)/src/bluetooth/bt_gatt.cpp \
	$(ROOT)/src/bluetooth/bt_peer.cpp

bt_gatt_srvc_test_SOURCES := \
	bt_gatt_srvc_test.cpp \
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
	$(ROOT)/src/bluetooth/services/bt_dis.cpp
bt_app_init_bm_test_DEPS := $(BT_APP_BM_INIT_INC)

# The SoftDevice Controller setup, against the stand-in vendor headers in
# nordic_stub/sdc. They sit under nordic_stub beside the SoftDevice ones
# because both stand in for a Nordic header on a host; the subdirectory keeps
# them apart because a build links the SoftDevice or the controller library,
# never both. Nothing else in the tree compiles that file on a host, so which
# sdc_support_* calls it makes and which resources it reserves had no check at
# all, which is where four rounds of review found the next defect.
bt_hci_ctlr_sdc_test_CPPFLAGS := -Inordic_stub/sdc -I$(ROOT)/ARM/Nordic/include
bt_hci_ctlr_sdc_test_SOURCES := \
	bt_hci_ctlr_sdc_test.cpp \
	nordic_stub/sdc/sdc_hci_cmd_stub.cpp \
	$(ROOT)/ARM/Nordic/src/bt_hci_ctlr_sdc.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp \
	$(BUILD_DIR)/cfifo.o
bt_hci_ctlr_sdc_test_DEPS := \
	$(wildcard nordic_stub/sdc/*.h) \
	$(ROOT)/ARM/Nordic/include/nrf_mpsl.h \
	$(ROOT)/ARM/Nordic/include/crypto_rng_nrf.h

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
	$(ROOT)/src/bluetooth/bt_psync.cpp \
	$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

bt_l2cap_adversarial_test_SOURCES := \
	bt_l2cap_adversarial_test.cpp \
	$(ROOT)/src/bluetooth/bt_l2cap.cpp

bt_app_adversarial_test_SOURCES := \
	bt_app_adversarial_test.cpp \
	$(ROOT)/src/bluetooth/bt_app.cpp

bt_std_services_test_SOURCES := \
	bt_std_services_test.cpp \
	$(ROOT)/src/bluetooth/services/bt_dis.cpp \
	$(ROOT)/src/bluetooth/services/bt_bas.cpp

bt_smp_sign_counter_test_SOURCES := \
	bt_smp_sign_counter_test.cpp \
	$(ROOT)/src/bluetooth/bt_smp_bond.cpp

# bt_smp.cpp is included by the test, not listed here: the cases reach the
# per-link state it keeps static. Its weak transport and platform hooks are
# replaced from the stub object, which has to be a separate translation unit
# for the strong definitions to win.
bt_smp_keydist_test_SOURCES := \
	bt_smp_keydist_test.cpp \
	bt_smp_link_test_stubs.cpp \
	$(ROOT)/src/crypto/crypto_p256.cpp \
	$(ROOT)/src/crypto/crypto_softrng.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp
bt_smp_keydist_test_EXTRA := $(CRC_OBJ) $(CFIFO_OBJ)
bt_smp_keydist_test_DEPS := \
	bt_smp_link_test_stubs.h \
	$(ROOT)/src/bluetooth/bt_smp.cpp

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
