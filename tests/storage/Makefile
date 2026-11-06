ROOT := ../..
BUILD_DIR := build
CXX ?= c++
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
HOST_INC := $(ROOT)/OSX/include
else
HOST_INC := $(ROOT)/Linux/include
endif

NVM_DISK_TEST := $(BUILD_DIR)/nvm_diskio_test
NVM_DISK_SOURCES := \
	$(ROOT)/exemples/storage/nvm_diskio_test.cpp \
	$(ROOT)/src/storage/diskio_nvm.cpp \
	$(ROOT)/src/storage/diskio_impl.cpp \
	$(ROOT)/src/storage/nvm.cpp \
	$(ROOT)/src/device.cpp \
	$(ROOT)/src/device_intrf.cpp

FAT12_TEST := $(BUILD_DIR)/fat12_provision_test
FAT12_SOURCES := \
	fat12_provision_test.cpp \
	$(ROOT)/src/storage/fat12_provision.cpp \
	$(ROOT)/src/storage/diskio_impl.cpp

.PHONY: test-nvm-disk test-fat12 clean

test-nvm-disk: $(NVM_DISK_TEST)
	./$(NVM_DISK_TEST)

$(NVM_DISK_TEST): $(NVM_DISK_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=gnu++2b -O1 -I$(ROOT)/include \
		-I$(ROOT)/include/storage -I$(HOST_INC) \
		-I$(ROOT)/tests/dfu/hostport \
		$(NVM_DISK_SOURCES) -o $@

test-fat12: $(FAT12_TEST)
	./$(FAT12_TEST)

$(FAT12_TEST): $(FAT12_SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=gnu++2b -O1 -I$(ROOT)/include \
		-I$(ROOT)/include/storage -I$(HOST_INC) \
		$(FAT12_SOURCES) -o $@

clean:
	rm -rf $(BUILD_DIR)
