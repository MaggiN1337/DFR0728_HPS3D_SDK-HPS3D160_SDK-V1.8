# Test Makefile for HPS3D LIDAR Service Unit Tests
# 
# This Makefile builds and runs comprehensive unit tests for:
# - MQTT functionality
# - LIDAR interface with mocks
# - Memory management
# - Thread safety
#
# Usage:
#   make all          - Build all tests
#   make test         - Build and run all tests
#   make clean        - Clean build artifacts
#   make mqtt         - Build and run MQTT tests only
#   make lidar        - Build and run LIDAR tests only
#   make memory       - Build and run memory tests only
#   make threads      - Build and run thread tests only
#   make coverage     - Run tests with coverage analysis

# Compiler and flags
CC=gcc
CFLAGS=-Wall -Wextra -std=c99 -pthread -D_GNU_SOURCE -g -O0
LDFLAGS=-pthread -lm

# Optional flags for different configurations
ifeq ($(DEBUG),1)
    CFLAGS += -DDEBUG -g3
endif

ifeq ($(ASAN),1)
    CFLAGS += -fsanitize=address -fno-omit-frame-pointer
    LDFLAGS += -fsanitize=address
endif

ifeq ($(TSAN),1)
    CFLAGS += -fsanitize=thread -fno-omit-frame-pointer
    LDFLAGS += -fsanitize=thread
endif

ifeq ($(COVERAGE),1)
    CFLAGS += --coverage -fprofile-arcs -ftest-coverage
    LDFLAGS += --coverage -lgcov
endif

# MQTT test specific flags
MQTT_LDFLAGS=$(LDFLAGS) -lmosquitto

# Test source files
MQTT_TEST_SRC=test_mqtt.c
LIDAR_TEST_SRC=test_lidar_mock.c
MEMORY_TEST_SRC=test_memory.c
THREADS_TEST_SRC=test_threads.c

# Test executables
MQTT_TEST=test_mqtt
LIDAR_TEST=test_lidar_mock
MEMORY_TEST=test_memory
THREADS_TEST=test_threads

# All tests
ALL_TESTS=$(MQTT_TEST) $(LIDAR_TEST) $(MEMORY_TEST) $(THREADS_TEST)

# Default target
.PHONY: all test clean mqtt lidar memory threads coverage help check-deps

all: $(ALL_TESTS)

# Build individual tests
$(MQTT_TEST): $(MQTT_TEST_SRC)
	@echo "Building MQTT tests..."
	$(CC) $(CFLAGS) -o $@ $< $(MQTT_LDFLAGS)

$(LIDAR_TEST): $(LIDAR_TEST_SRC)
	@echo "Building LIDAR mock interface tests..."
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(MEMORY_TEST): $(MEMORY_TEST_SRC)
	@echo "Building memory management tests..."
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(THREADS_TEST): $(THREADS_TEST_SRC)
	@echo "Building thread safety tests..."
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# Check dependencies
check-deps:
	@echo "Checking test dependencies..."
	@command -v gcc >/dev/null 2>&1 || { echo "ERROR: gcc not found"; exit 1; }
	@echo "  ✓ gcc found"
	@pkg-config --exists libmosquitto 2>/dev/null || echo "  ⚠ libmosquitto not found - MQTT tests may fail"
	@command -v valgrind >/dev/null 2>&1 && echo "  ✓ valgrind available" || echo "  ⚠ valgrind not found - memory leak detection unavailable"
	@command -v gcov >/dev/null 2>&1 && echo "  ✓ gcov available" || echo "  ⚠ gcov not found - coverage analysis unavailable"

# Run all tests
test: all check-deps
	@echo "=========================================="
	@echo "Running HPS3D LIDAR Service Test Suite"
	@echo "=========================================="
	@echo ""
	@total=0; passed=0; \
	for test in $(ALL_TESTS); do \
		echo "Running $$test..."; \
		if ./$$test; then \
			passed=$$((passed + 1)); \
		fi; \
		total=$$((total + 1)); \
		echo ""; \
	done; \
	echo "=========================================="; \
	echo "Test Summary: $$passed/$$total tests passed"; \
	if [ $$passed -eq $$total ]; then \
		echo "🎉 All tests PASSED!"; \
		exit 0; \
	else \
		echo "❌ Some tests FAILED!"; \
		exit 1; \
	fi

# Run individual test suites
mqtt: $(MQTT_TEST) check-deps
	@echo "Running MQTT functionality tests..."
	@./$(MQTT_TEST)

lidar: $(LIDAR_TEST) check-deps
	@echo "Running LIDAR interface tests..."
	@./$(LIDAR_TEST)

memory: $(MEMORY_TEST) check-deps
	@echo "Running memory management tests..."
	@./$(MEMORY_TEST)

threads: $(THREADS_TEST) check-deps
	@echo "Running thread safety tests..."
	@./$(THREADS_TEST)

# Memory leak detection with Valgrind
valgrind: all
	@echo "Running tests with Valgrind memory leak detection..."
	@for test in $(ALL_TESTS); do \
		echo "Checking $$test for memory leaks..."; \
		valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
		         --error-exitcode=1 ./$$test > valgrind_$$test.log 2>&1; \
		if [ $$? -eq 0 ]; then \
			echo "  ✓ $$test: No memory leaks detected"; \
		else \
			echo "  ❌ $$test: Memory issues detected (see valgrind_$$test.log)"; \
		fi; \
	done

# Thread safety analysis with ThreadSanitizer
tsan: clean
	@echo "Building tests with ThreadSanitizer..."
	@$(MAKE) all TSAN=1
	@echo "Running thread safety analysis..."
	@for test in $(THREADS_TEST) $(MEMORY_TEST); do \
		echo "Analyzing $$test for thread safety issues..."; \
		./$$test > tsan_$$test.log 2>&1; \
		if [ $$? -eq 0 ]; then \
			echo "  ✓ $$test: No thread safety issues detected"; \
		else \
			echo "  ⚠ $$test: Potential thread safety issues (see tsan_$$test.log)"; \
		fi; \
	done

# Address sanitizer for memory corruption detection
asan: clean
	@echo "Building tests with AddressSanitizer..."
	@$(MAKE) all ASAN=1
	@echo "Running memory corruption detection..."
	@for test in $(ALL_TESTS); do \
		echo "Checking $$test for memory corruption..."; \
		./$$test > asan_$$test.log 2>&1; \
		if [ $$? -eq 0 ]; then \
			echo "  ✓ $$test: No memory corruption detected"; \
		else \
			echo "  ❌ $$test: Memory corruption detected (see asan_$$test.log)"; \
		fi; \
	done

# Coverage analysis
coverage: clean
	@echo "Building tests with coverage instrumentation..."
	@$(MAKE) all COVERAGE=1
	@echo "Running tests for coverage analysis..."
	@for test in $(ALL_TESTS); do \
		echo "Running $$test with coverage..."; \
		./$$test > /dev/null 2>&1; \
	done
	@echo "Generating coverage reports..."
	@for src in $(MQTT_TEST_SRC) $(LIDAR_TEST_SRC) $(MEMORY_TEST_SRC) $(THREADS_TEST_SRC); do \
		gcov $$src > /dev/null 2>&1; \
	done
	@echo "Coverage files generated (.gcov files)"
	@echo "Summary:"
	@for gcov_file in *.gcov; do \
		if [ -f "$$gcov_file" ]; then \
			lines_total=$$(grep -c '^        [0-9#]' "$$gcov_file" 2>/dev/null || echo 0); \
			lines_covered=$$(grep -c '^        [1-9]' "$$gcov_file" 2>/dev/null || echo 0); \
			if [ $$lines_total -gt 0 ]; then \
				coverage_pct=$$(echo "scale=1; $$lines_covered * 100 / $$lines_total" | bc -l 2>/dev/null || echo "0"); \
				echo "  $$gcov_file: $$coverage_pct% ($$lines_covered/$$lines_total lines)"; \
			fi; \
		fi; \
	done

# Continuous integration test suite
ci: clean check-deps
	@echo "Running CI test suite..."
	@$(MAKE) test
	@echo "Running memory leak detection..."
	@$(MAKE) valgrind || echo "Valgrind analysis completed with warnings"
	@echo "Running thread safety analysis..."
	@$(MAKE) tsan || echo "ThreadSanitizer analysis completed with warnings"
	@echo "Running address sanitizer..."
	@$(MAKE) asan || echo "AddressSanitizer analysis completed with warnings"
	@echo "CI test suite completed"

# Performance benchmark
benchmark: all
	@echo "Running performance benchmarks..."
	@echo "Memory allocation benchmark:"
	@time ./$(MEMORY_TEST) > /dev/null
	@echo "Thread synchronization benchmark:"
	@time ./$(THREADS_TEST) > /dev/null
	@echo "MQTT communication benchmark:"
	@time ./$(MQTT_TEST) > /dev/null || echo "MQTT benchmark skipped (broker not available)"

# Help target
help:
	@echo "HPS3D LIDAR Service Test Suite"
	@echo "==============================="
	@echo ""
	@echo "Available targets:"
	@echo "  all        - Build all tests"
	@echo "  test       - Build and run all tests"
	@echo "  mqtt       - Run MQTT functionality tests"
	@echo "  lidar      - Run LIDAR interface tests"
	@echo "  memory     - Run memory management tests"
	@echo "  threads    - Run thread safety tests"
	@echo "  valgrind   - Run tests with memory leak detection"
	@echo "  tsan       - Run tests with thread safety analysis"
	@echo "  asan       - Run tests with address sanitizer"
	@echo "  coverage   - Run tests with coverage analysis"
	@echo "  ci         - Run complete CI test suite"
	@echo "  benchmark  - Run performance benchmarks"
	@echo "  clean      - Clean build artifacts"
	@echo "  check-deps - Check test dependencies"
	@echo "  help       - Show this help"
	@echo ""
	@echo "Environment variables:"
	@echo "  DEBUG=1    - Enable debug build"
	@echo "  ASAN=1     - Enable AddressSanitizer"
	@echo "  TSAN=1     - Enable ThreadSanitizer"
	@echo "  COVERAGE=1 - Enable coverage instrumentation"
	@echo ""
	@echo "Examples:"
	@echo "  make test                    # Run all tests"
	@echo "  make DEBUG=1 test           # Run with debug info"
	@echo "  make ASAN=1 memory          # Run memory tests with AddressSanitizer"
	@echo "  make coverage               # Generate coverage report"

# Clean up
clean:
	@echo "Cleaning test artifacts..."
	rm -f $(ALL_TESTS)
	rm -f *.o *.gcno *.gcda *.gcov
	rm -f valgrind_*.log tsan_*.log asan_*.log
	rm -f core core.*
	@echo "Clean complete"

# Install test dependencies (Ubuntu/Debian)
install-deps:
	@echo "Installing test dependencies..."
	sudo apt-get update
	sudo apt-get install -y \
		gcc \
		make \
		libmosquitto-dev \
		mosquitto \
		mosquitto-clients \
		valgrind \
		gcovr \
		bc
	@echo "Dependencies installed"

# Create test report
report: coverage
	@echo "Generating test report..."
	@echo "# HPS3D LIDAR Service Test Report" > TEST_REPORT.md
	@echo "" >> TEST_REPORT.md
	@echo "Generated on: $$(date)" >> TEST_REPORT.md
	@echo "" >> TEST_REPORT.md
	@echo "## Test Results" >> TEST_REPORT.md
	@echo "" >> TEST_REPORT.md
	@$(MAKE) test 2>&1 | grep -E "(PASS|FAIL|Test Summary)" >> TEST_REPORT.md || true
	@echo "" >> TEST_REPORT.md
	@echo "## Coverage Analysis" >> TEST_REPORT.md
	@echo "" >> TEST_REPORT.md
	@echo "\`\`\`" >> TEST_REPORT.md
	@$(MAKE) coverage 2>&1 | grep -E "\.gcov|%" >> TEST_REPORT.md || true
	@echo "\`\`\`" >> TEST_REPORT.md
	@echo "Test report generated: TEST_REPORT.md"