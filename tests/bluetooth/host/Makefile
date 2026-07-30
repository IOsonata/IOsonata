ROOT := ../../..
BUILD_DIR := build
TARGET := $(BUILD_DIR)/bt_adv_uuid_test

CXX ?= g++
CPPFLAGS += -I$(ROOT)/include
CXXFLAGS ?= -O1 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -Wpedantic -fno-omit-frame-pointer
SANITIZERS ?= -fsanitize=address,undefined

SOURCES := \
	bt_adv_uuid_test.cpp \
	$(ROOT)/src/bluetooth/bt_adv.cpp \
	$(ROOT)/src/bluetooth/bt_uuid.cpp

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SOURCES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) $(SOURCES) -o $@ $(SANITIZERS)

clean:
	rm -rf $(BUILD_DIR)
