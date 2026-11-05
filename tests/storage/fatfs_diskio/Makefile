ROOT := ../../..
BUILD := build

CC ?= cc
CXX ?= c++
CPPFLAGS += -I$(ROOT)/include -I$(ROOT)/fatfs
CFLAGS += -std=c11 -O2 -Wall -Wextra
CXXFLAGS += -std=gnu++17 -O2 -Wall -Wextra

OBJECTS := $(BUILD)/test.o $(BUILD)/diskio.o $(BUILD)/ff_diskio.o \
	$(BUILD)/ff.o $(BUILD)/ffsystem.o $(BUILD)/ffunicode.o

.PHONY: test clean

test: $(BUILD)/fatfs_diskio_test
	./$(BUILD)/fatfs_diskio_test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test.o: fatfs_diskio_test.cpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD)/diskio.o: $(ROOT)/src/storage/diskio_impl.cpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD)/ff_diskio.o: $(ROOT)/src/storage/ff_diskio.cpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD)/ff.o: $(ROOT)/fatfs/ff.c | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/ffsystem.o: $(ROOT)/fatfs/ffsystem.c | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/ffunicode.o: $(ROOT)/fatfs/ffunicode.c | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/fatfs_diskio_test: $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD)
