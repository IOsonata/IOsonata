CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-omit-frame-pointer
.PHONY: test clean
test: build/buzzer_test
	ASAN_OPTIONS=detect_leaks=0 ./build/buzzer_test
build/buzzer_test: buzzer_test.cpp ../../src/audio/buzzer.cpp ../../src/coredev/timer.cpp ../../include/audio/buzzer.h shim/idelay.h
	mkdir -p build
	$(CXX) $(CXXFLAGS) -Ishim -I../../include buzzer_test.cpp ../../src/audio/buzzer.cpp ../../src/coredev/timer.cpp -o $@
clean:
	rm -rf build
