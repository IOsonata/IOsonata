ROOT := ../..
BUILD_DIR := build
IMG_DIR := $(BUILD_DIR)/img

CC ?= cc
CXX ?= g++
PYTHON ?= python3
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include -Ihostport -I$(ROOT)/micro-ecc \
	-DuECC_SUPPORTS_secp256r1=1 -DuECC_SUPPORT_COMPRESSED_POINT=0 \
	-DuECC_OPTIMIZATION_LEVEL=2

CFLAGS ?= $(OPT) -g
CFLAGS += -std=gnu17 -Wall -Wextra -fno-omit-frame-pointer
CXXFLAGS ?= $(OPT) -g
CXXFLAGS += -std=gnu++23 -Wall -Wextra -fno-omit-frame-pointer \
	-fno-exceptions -fno-rtti

SANITIZERS ?= -fsanitize=address,undefined -fno-sanitize-recover=all
TEST_ENV := ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1

# The layout a target gets from dfu_layout_*.ld, at the address dfu_host.cpp
# maps its memory. Must match dfu_host.h.
LAYOUT := -no-pie \
	-Wl,--defsym,__dfu_slot0_start=0x10000000 \
	-Wl,--defsym,__dfu_slot0_end=0x10010000 \
	-Wl,--defsym,__dfu_rec_start=0x10010000 \
	-Wl,--defsym,__dfu_rec_end=0x10011000 \
	-Wl,--defsym,__dfu_slot1_start=0x10011000 \
	-Wl,--defsym,__dfu_slot1_end=0x10022000 \
	-Wl,--defsym,__dfu_flag=0x1003FFF0

CRYPTO_SRC := \
	$(ROOT)/src/crypto/crypto_softsha256.cpp \
	$(ROOT)/src/crypto/crypto_uecc.cpp

DFU_SRC := \
	$(ROOT)/src/dfu/dfu_image.cpp \
	$(ROOT)/src/dfu/dfu_layout.cpp \
	$(ROOT)/src/dfu/dfu_writer.cpp \
	$(ROOT)/src/dfu/dfu_store_nvm.cpp

HOST_SRC := \
	dfu_host.cpp \
	$(ROOT)/src/storage/nvm.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp

# C sources, built as C.
UECC_OBJ := $(BUILD_DIR)/uECC.o $(BUILD_DIR)/cbor.o $(BUILD_DIR)/crc.o \
	$(BUILD_DIR)/base64.o

TESTS := $(BUILD_DIR)/dfu_image_test $(BUILD_DIR)/dfu_boot_test \
	$(BUILD_DIR)/dfu_mgr_test $(BUILD_DIR)/dfu_wire_test \
	$(BUILD_DIR)/dfu_smp_test $(BUILD_DIR)/bt_dfu_smp_test \
	$(BUILD_DIR)/dfu_serial_test $(BUILD_DIR)/dfu_http_test

.PHONY: all test clean images udp serial http wire

all: $(TESTS) $(BUILD_DIR)/dfu_smp_udp_host $(BUILD_DIR)/dfu_serial_host \
	$(BUILD_DIR)/dfu_wire_host \
	$(BUILD_DIR)/dfu_http_host

$(BUILD_DIR):
	mkdir -p $@

images: $(IMG_DIR)/v1.bin

$(IMG_DIR)/v1.bin: mkimages.py | $(BUILD_DIR)
	$(PYTHON) mkimages.py $(IMG_DIR) 0x10000000

$(BUILD_DIR)/uECC.o: $(ROOT)/micro-ecc/uECC.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) -std=gnu17 -O2 -g $(SANITIZERS) -c $< -o $@

$(BUILD_DIR)/base64.o: $(ROOT)/src/base64.c $(ROOT)/include/base64.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -include stdint.h -include stddef.h -c $< -o $@

$(BUILD_DIR)/crc.o: $(ROOT)/src/crc.c $(ROOT)/include/crc.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) -c $< -o $@

$(BUILD_DIR)/cbor.o: $(ROOT)/src/cbor.c $(ROOT)/include/cbor.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -Werror -Wshadow $(SANITIZERS) -c $< -o $@

$(BUILD_DIR)/dfu_image_test: dfu_image_test.cpp $(DFU_SRC) $(CRYPTO_SRC) \
		$(HOST_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/dfu_boot_test: dfu_boot_test.cpp $(ROOT)/src/dfu/dfu_boot.cpp \
		$(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/dfu_mgr_test: dfu_mgr_test.cpp $(ROOT)/src/dfu/dfu_mgr.cpp \
		$(ROOT)/src/dfu/dfu_boot.cpp \
		$(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-missing-field-initializers \
		$(SANITIZERS) -x c++ $(filter %.cpp,$^) -x none $(UECC_OBJ) \
		$(LAYOUT) -o $@

$(BUILD_DIR)/dfu_wire_test: dfu_wire_test.cpp $(ROOT)/src/dfu/dfu_wire.cpp \
		$(ROOT)/src/slip_intrf.cpp \
		$(ROOT)/src/dfu/dfu_mgr.cpp $(ROOT)/src/dfu/dfu_boot.cpp \
		$(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-missing-field-initializers \
		$(SANITIZERS) -x c++ $(filter %.cpp,$^) -x none $(UECC_OBJ) \
		$(LAYOUT) -o $@

$(BUILD_DIR)/dfu_smp_test: dfu_smp_test.cpp $(ROOT)/src/dfu/dfu_smp.cpp $(ROOT)/src/dfu/dfu_mgr.cpp \
		$(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) \
		$(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/dfu_smp_udp_host: dfu_smp_udp_host.cpp $(ROOT)/src/dfu/dfu_smp.cpp $(ROOT)/src/dfu/dfu_mgr.cpp \
		$(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) \
		$(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/bt_dfu_smp_test: bt_dfu_smp_test.cpp \
		$(ROOT)/src/bluetooth/bt_dfu_smp.cpp $(ROOT)/src/dfu/dfu_smp.cpp $(ROOT)/src/dfu/dfu_mgr.cpp \
		$(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) \
		$(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-missing-field-initializers \
		$(SANITIZERS) -x c++ $(filter %.cpp,$^) -x none $(UECC_OBJ) \
		$(LAYOUT) -o $@

$(BUILD_DIR)/dfu_serial_test: dfu_serial_test.cpp $(ROOT)/src/dfu/dfu_serial.cpp \
		$(ROOT)/src/dfu/dfu_smp.cpp $(ROOT)/src/dfu/dfu_mgr.cpp $(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) \
		$(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/dfu_http_test: dfu_http_test.cpp $(ROOT)/src/dfu/dfu_http.cpp $(ROOT)/src/dfu/dfu_mgr.cpp \
		$(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) \
		$(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/dfu_serial_host: dfu_serial_host.cpp $(ROOT)/src/dfu/dfu_serial.cpp \
		$(ROOT)/src/dfu/dfu_smp.cpp $(ROOT)/src/dfu/dfu_mgr.cpp $(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) \
		$(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

test: $(TESTS) $(BUILD_DIR)/dfu_prod_boot images
	$(TEST_ENV) $(BUILD_DIR)/dfu_image_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/dfu_boot_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/dfu_mgr_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/dfu_wire_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/dfu_smp_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/bt_dfu_smp_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/dfu_serial_test $(IMG_DIR)
	$(TEST_ENV) $(BUILD_DIR)/dfu_http_test $(IMG_DIR)
	$(PYTHON) prod_hex_test.py $(IMG_DIR) $(BUILD_DIR)/dfu_prod_boot

# End to end with a real SMP client library (smpclient) over UDP.
$(BUILD_DIR)/dfu_prod_boot: dfu_prod_boot.cpp $(ROOT)/src/dfu/dfu_boot.cpp \
		$(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

$(BUILD_DIR)/dfu_http_host: dfu_http_host.cpp $(ROOT)/src/dfu/dfu_http.cpp $(ROOT)/src/dfu/dfu_mgr.cpp \
		$(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) $(HOST_SRC) $(CRYPTO_SRC) \
		$(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ $(filter %.cpp,$^) \
		-x none $(UECC_OBJ) $(LAYOUT) -o $@

# Download from Python's http.server over TCP, with dropped connections.
http: $(BUILD_DIR)/dfu_http_host images
	$(PYTHON) http_test.py $(BUILD_DIR)/dfu_http_host $(IMG_DIR)

$(BUILD_DIR)/dfu_wire_host: dfu_wire_host.cpp $(ROOT)/src/dfu/dfu_wire.cpp \
		$(ROOT)/src/slip_intrf.cpp \
		$(ROOT)/src/dfu/dfu_mgr.cpp $(ROOT)/src/dfu/dfu_boot.cpp $(DFU_SRC) \
		$(HOST_SRC) $(CRYPTO_SRC) $(UECC_OBJ) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-missing-field-initializers \
		$(SANITIZERS) -x c++ $(filter %.cpp,$^) -x none $(UECC_OBJ) \
		$(LAYOUT) -o $@

# Stage 0 recovery driven by Python/dfu_wire.py, the host tool, on a pty.
wire: $(BUILD_DIR)/dfu_wire_host images
	$(PYTHON) wire_test.py $(BUILD_DIR)/dfu_wire_host $(IMG_DIR) nor
	$(PYTHON) wire_test.py $(BUILD_DIR)/dfu_wire_host $(IMG_DIR) rram
	$(PYTHON) wire_test.py $(BUILD_DIR)/dfu_wire_host $(IMG_DIR) ecc16
	$(PYTHON) wire_test.py $(BUILD_DIR)/dfu_wire_host $(IMG_DIR) big256
	$(PYTHON) wire_test.py $(BUILD_DIR)/dfu_wire_host $(IMG_DIR) sect

# Stage 0 recovery driven by smpclient over its serial transport, on a pty.
serial: $(BUILD_DIR)/dfu_serial_host images
	$(PYTHON) smp_serial_test.py $(BUILD_DIR)/dfu_serial_host $(IMG_DIR) nor
	$(PYTHON) smp_serial_test.py $(BUILD_DIR)/dfu_serial_host $(IMG_DIR) rram
	$(PYTHON) smp_serial_test.py $(BUILD_DIR)/dfu_serial_host $(IMG_DIR) ecc16
	$(PYTHON) smp_serial_test.py $(BUILD_DIR)/dfu_serial_host $(IMG_DIR) big256

udp: $(BUILD_DIR)/dfu_smp_udp_host images
	$(PYTHON) smp_udp_test.py $(BUILD_DIR)/dfu_smp_udp_host $(IMG_DIR) nor
	$(PYTHON) smp_udp_test.py $(BUILD_DIR)/dfu_smp_udp_host $(IMG_DIR) rram

clean:
	rm -rf $(BUILD_DIR)
