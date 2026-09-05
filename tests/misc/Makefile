ROOT := ../..
BUILD_DIR := build

CXX ?= g++
CC ?= gcc
OPT ?= -O1

CPPFLAGS += -I$(ROOT)/include

CXXFLAGS ?= $(OPT) -g
CXXFLAGS += -std=gnu++17 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong \
	-D_GLIBCXX_ASSERTIONS
CFLAGS ?= $(OPT) -g
CFLAGS += -std=gnu11 -Wall -Wextra -fno-omit-frame-pointer \
	-ffunction-sections -fdata-sections -fstack-protector-strong

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

MEMCPY_TARGET := $(BUILD_DIR)/memcpy_fast_test
MEMCPY_SOURCES := \
	memcpy_fast_test.cpp \
	$(ROOT)/src/memcpy_fast.cpp

CFIFO_TARGET := $(BUILD_DIR)/cfifo_test
CFIFO_SOURCES := \
	cfifo_test.cpp \
	$(ROOT)/src/cfifo.c

# Concurrency. Thread sanitizer cannot be combined with the address
# sanitizer, so the race build is its own target.
THREAD_TARGET := $(BUILD_DIR)/cfifo_thread_test
TSAN_TARGET := $(BUILD_DIR)/cfifo_thread_test_tsan
THREAD_SOURCES := \
	cfifo_thread_test.cpp \
	$(ROOT)/src/cfifo.c
THREAD_OLD_TARGET := $(BUILD_DIR)/cfifo_thread_test_2021
TSAN_OLD_TARGET := $(BUILD_DIR)/cfifo_thread_test_2021_tsan
OPS ?= 2000000
TSAN_OPS ?= 200000

# Same harness against the current CFifo and against the 2021 version, which
# is extracted from git rather than kept as a copy in the tree.
LEGACY_REV ?= 4c69ab6c
LEGACY_DIR := $(BUILD_DIR)/legacy

CMP_CUR_TARGET := $(BUILD_DIR)/cfifo_compare_current
CMP_OLD_TARGET := $(BUILD_DIR)/cfifo_compare_2021

# Timing, not a regression test. Built at -O2 with sanitizers off so the
# numbers reflect a release build. Repeat count is the first argument.
BENCH_TARGET := $(BUILD_DIR)/bench
BENCH_SOURCES := \
	bench.cpp \
	$(ROOT)/src/memcpy_fast.cpp \
	$(ROOT)/src/cfifo.c
BENCH_OPT ?= -O2
REPEATS ?= 7

HEADERS := \
	$(ROOT)/include/cfifo.h \
	$(ROOT)/include/memcpy_fast.h

TARGETS := $(MEMCPY_TARGET) $(CFIFO_TARGET) $(THREAD_TARGET)

.PHONY: all test test-gcc test-clang test-matrix bench compare thread \
	thread-compare test-tsan bench-memcpy bench-cfifo clean

all: test

test: $(TARGETS)
	$(TEST_ENV) ./$(MEMCPY_TARGET)
	$(TEST_ENV) ./$(CFIFO_TARGET)
	./$(THREAD_TARGET) $(OPS)

# Race detection. Fewer operations because the sanitizer is slow.
test-tsan: $(TSAN_TARGET)
	TSAN_OPTIONS=halt_on_error=1 ./$(TSAN_TARGET) $(TSAN_OPS)

thread: $(THREAD_TARGET)
	./$(THREAD_TARGET) $(OPS)

# The same concurrency cases against both versions, plain then under the
# race detector.
thread-compare: $(THREAD_TARGET) $(THREAD_OLD_TARGET) $(TSAN_TARGET) \
		$(TSAN_OLD_TARGET)
	./$(THREAD_TARGET) $(OPS)
	@echo
	-./$(THREAD_OLD_TARGET) $(OPS)
	@echo
	TSAN_OPTIONS=halt_on_error=0 ./$(TSAN_TARGET) $(TSAN_OPS) 2>&1 | \
		grep -cE "WARNING: ThreadSanitizer" | \
		xargs -I{} echo "current: {} race reports"
	@echo
	-TSAN_OPTIONS=halt_on_error=0 ./$(TSAN_OLD_TARGET) $(TSAN_OPS) 2>&1 | \
		grep -cE "WARNING: ThreadSanitizer" | \
		xargs -I{} echo "2021: {} race reports"

bench: $(BENCH_TARGET)
	./$(BENCH_TARGET) all $(REPEATS)

bench-memcpy: $(BENCH_TARGET)
	./$(BENCH_TARGET) memcpy $(REPEATS)

bench-cfifo: $(BENCH_TARGET)
	./$(BENCH_TARGET) cfifo $(REPEATS)

# Correctness and timing for both versions, one after the other.
compare: $(CMP_CUR_TARGET) $(CMP_OLD_TARGET)
	./$(CMP_CUR_TARGET) $(REPEATS)
	@echo
	./$(CMP_OLD_TARGET) $(REPEATS)

test-gcc:
	$(MAKE) clean
	$(MAKE) test CXX=g++ CC=gcc

test-clang:
	$(MAKE) clean
	$(MAKE) test CXX=clang++ CC=clang

test-matrix: test-gcc test-clang

$(MEMCPY_TARGET): $(MEMCPY_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(MEMCPY_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

$(CFIFO_TARGET): $(CFIFO_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(SANITIZERS) -x c++ \
		$(CFIFO_SOURCES) -o $@ $(LDFLAGS) $(SANITIZERS)

# The 2021 sources come straight out of git, so nothing legacy is tracked.
$(LEGACY_DIR)/cfifo.c $(LEGACY_DIR)/cfifo.h $(LEGACY_DIR)/stdatomic.h: 
	@mkdir -p $(LEGACY_DIR)
	cd $(ROOT) && git show $(LEGACY_REV):src/cfifo.c > \
		tests/misc/$(LEGACY_DIR)/cfifo.c
	cd $(ROOT) && git show $(LEGACY_REV):include/cfifo.h > \
		tests/misc/$(LEGACY_DIR)/cfifo.h
	@printf '%s\n' \
		'// Build shim. The 2021 cfifo.c includes stdatomic.h, which a C++' \
		'// compiler rejects. It only uses atomic_int and atomic_store on an' \
		'// int, so those are what this supplies, with the same sequentially' \
		'// consistent store the original got.' \
		'#ifndef __LEGACY_STDATOMIC_SHIM_H__' \
		'#define __LEGACY_STDATOMIC_SHIM_H__' \
		'typedef int atomic_int;' \
		'static inline void atomic_store(atomic_int *pVar, int NewVal)' \
		'{ __atomic_store_n(pVar, NewVal, __ATOMIC_SEQ_CST); }' \
		'#endif' > $(LEGACY_DIR)/stdatomic.h

$(CMP_CUR_TARGET): cfifo_compare.cpp $(ROOT)/src/cfifo.c $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra -x c++ \
		cfifo_compare.cpp $(ROOT)/src/cfifo.c -o $@ $(LDFLAGS)

$(CMP_OLD_TARGET): cfifo_compare.cpp $(LEGACY_DIR)/cfifo.c $(LEGACY_DIR)/cfifo.h \
		$(LEGACY_DIR)/stdatomic.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) -I$(LEGACY_DIR) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra \
		-DCFIFO_LEGACY -x c++ \
		cfifo_compare.cpp $(LEGACY_DIR)/cfifo.c -o $@ $(LDFLAGS)

# Built without the address sanitizer: it does not add anything here and
# the deliberate race case runs far slower under it.
$(THREAD_TARGET): $(THREAD_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra -x c++ \
		$(THREAD_SOURCES) -o $@ $(LDFLAGS) -pthread

$(TSAN_TARGET): $(THREAD_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra \
		-fsanitize=thread -x c++ \
		$(THREAD_SOURCES) -o $@ -fsanitize=thread -pthread

$(THREAD_OLD_TARGET): cfifo_thread_test.cpp $(LEGACY_DIR)/cfifo.c \
		$(LEGACY_DIR)/cfifo.h $(LEGACY_DIR)/stdatomic.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) -I$(LEGACY_DIR) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra \
		-DCFIFO_LEGACY -x c++ \
		cfifo_thread_test.cpp $(LEGACY_DIR)/cfifo.c -o $@ $(LDFLAGS) -pthread

$(TSAN_OLD_TARGET): cfifo_thread_test.cpp $(LEGACY_DIR)/cfifo.c \
		$(LEGACY_DIR)/cfifo.h $(LEGACY_DIR)/stdatomic.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) -I$(LEGACY_DIR) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra \
		-DCFIFO_LEGACY -fsanitize=thread -x c++ \
		cfifo_thread_test.cpp $(LEGACY_DIR)/cfifo.c -o $@ \
		-fsanitize=thread -pthread

$(BENCH_TARGET): $(BENCH_SOURCES) $(HEADERS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(BENCH_OPT) -g -std=gnu++17 -Wall -Wextra -x c++ \
		$(BENCH_SOURCES) -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
