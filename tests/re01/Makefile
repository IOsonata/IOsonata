ROOT := ../..
BUILD := build
CPPFLAGS := -DRE01_1500KB -DRE01_1500KB_CFB -Ihost -I$(ROOT)/include -I$(ROOT)/ARM/Renesas/RE01/include -I$(ROOT)/ARM/Renesas/RE01/RE01_1500KB/lib/include
COMMON := -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
CXXFLAGS := $(COMMON) -std=gnu++17 -fno-exceptions -fno-rtti
CFLAGS := $(COMMON) -std=gnu11 -Wno-int-to-pointer-cast
PORT := $(ROOT)/ARM/Renesas/RE01/src
CXXSRC := $(PORT)/interrupt_re01.cpp $(PORT)/timer_re01.cpp $(PORT)/timer_re01_agt.cpp $(PORT)/timer_re01_tmr.cpp $(PORT)/uart_re01.cpp re01_port_test.cpp
CSRC := $(PORT)/system_re01.c $(PORT)/iopincfg_re01.c $(ROOT)/src/cfifo.c
OBJS := $(addprefix $(BUILD)/,$(notdir $(CXXSRC:.cpp=.o) $(CSRC:.c=.o)))
vpath %.cpp $(sort $(dir $(CXXSRC)))
vpath %.c $(sort $(dir $(CSRC)))
.PHONY: test clean
test: $(BUILD)/re01_port_test
	./$(BUILD)/re01_port_test
$(BUILD)/re01_port_test: $(OBJS)
	$(CXX) $^ -o $@
$(BUILD)/%.o: %.cpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
$(BUILD):
	mkdir -p $@
clean:
	rm -rf $(BUILD)
