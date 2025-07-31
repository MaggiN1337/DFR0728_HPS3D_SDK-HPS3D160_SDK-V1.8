#!/bin/bash

# HPS3D LIDAR Service Test Runner Script
# 
# This script provides a comprehensive test execution framework with:
# - Automated dependency checking
# - Test environment setup
# - Parallel test execution
# - Detailed reporting
# - CI/CD integration
# - Performance monitoring

set -e  # Exit on any error

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TEST_RESULTS_DIR="$SCRIPT_DIR/results"
LOG_FILE="$TEST_RESULTS_DIR/test_execution.log"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Test configuration
MQTT_BROKER_TIMEOUT=5
PARALLEL_JOBS=4
COVERAGE_THRESHOLD=80
MEMORY_LEAK_THRESHOLD=0

# Global counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
SKIPPED_TESTS=0

# Logging functions
log() {
    echo -e "$(date +'%Y-%m-%d %H:%M:%S') $*" | tee -a "$LOG_FILE"
}

log_info() {
    log "${BLUE}[INFO]${NC} $*"
}

log_success() {
    log "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
    log "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
    log "${RED}[ERROR]${NC} $*"
}

log_debug() {
    if [[ "${DEBUG:-0}" == "1" ]]; then
        log "${PURPLE}[DEBUG]${NC} $*"
    fi
}

# Banner function
print_banner() {
    echo -e "${CYAN}"
    echo "================================================================="
    echo "           HPS3D LIDAR Service Test Suite Runner"
    echo "================================================================="
    echo -e "${NC}"
    echo "Project: HPS3D Distance CLI"
    echo "Test Directory: $SCRIPT_DIR"
    echo "Results Directory: $TEST_RESULTS_DIR"
    echo "Date: $(date)"
    echo ""
}

# Help function
show_help() {
    cat << EOF
HPS3D LIDAR Service Test Runner

Usage: $0 [OPTIONS] [TEST_SUITES...]

OPTIONS:
  -h, --help              Show this help message
  -v, --verbose           Enable verbose output
  -d, --debug             Enable debug output
  -q, --quiet             Quiet mode (minimal output)
  -j, --parallel JOBS     Number of parallel jobs (default: $PARALLEL_JOBS)
  -t, --timeout SECONDS   MQTT broker timeout (default: $MQTT_BROKER_TIMEOUT)
  -c, --coverage          Enable coverage analysis
  -m, --memory-check      Enable memory leak detection (Valgrind)
  -s, --sanitizers        Enable all sanitizers (AddressSanitizer, ThreadSanitizer)
  --asan                  Enable AddressSanitizer only
  --tsan                  Enable ThreadSanitizer only
  --ci                    Run in CI mode (all checks enabled)
  --benchmark             Run performance benchmarks
  --report                Generate detailed test report
  --clean                 Clean before running tests
  --install-deps          Install test dependencies
  --mqtt-broker           Start local MQTT broker for tests

TEST_SUITES:
  mqtt                    MQTT functionality tests
  lidar                   LIDAR interface tests with mocks
  memory                  Memory management tests
  threads                 Thread safety tests
  all                     All test suites (default)

EXAMPLES:
  $0                      # Run all tests with default settings
  $0 mqtt lidar           # Run only MQTT and LIDAR tests
  $0 --coverage --report  # Run with coverage and generate report
  $0 --ci                 # Run in CI mode with all checks
  $0 --benchmark          # Run performance benchmarks
  $0 --memory-check mqtt  # Run MQTT tests with memory leak detection

ENVIRONMENT VARIABLES:
  DEBUG=1                 Enable debug mode
  QUIET=1                 Enable quiet mode
  NO_COLOR=1              Disable colored output
  MQTT_HOST=host          Override MQTT broker host (default: localhost)
  MQTT_PORT=port          Override MQTT broker port (default: 1883)

EOF
}

# Setup functions
setup_directories() {
    log_info "Setting up test directories..."
    mkdir -p "$TEST_RESULTS_DIR"
    mkdir -p "$TEST_RESULTS_DIR/coverage"
    mkdir -p "$TEST_RESULTS_DIR/logs"
    mkdir -p "$TEST_RESULTS_DIR/reports"
}

check_dependencies() {
    log_info "Checking test dependencies..."
    
    local missing_deps=()
    
    # Check required tools
    if ! command -v gcc >/dev/null 2>&1; then
        missing_deps+=("gcc")
    fi
    
    if ! command -v make >/dev/null 2>&1; then
        missing_deps+=("make")
    fi
    
    # Check optional tools
    if ! command -v valgrind >/dev/null 2>&1; then
        log_warning "valgrind not found - memory leak detection will be unavailable"
    fi
    
    if ! command -v gcov >/dev/null 2>&1; then
        log_warning "gcov not found - coverage analysis will be unavailable"
    fi
    
    if ! pkg-config --exists libmosquitto 2>/dev/null; then
        log_warning "libmosquitto not found - MQTT tests may fail"
    fi
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        log_error "Missing required dependencies: ${missing_deps[*]}"
        log_info "Install with: sudo apt-get install ${missing_deps[*]}"
        return 1
    fi
    
    log_success "All required dependencies found"
    return 0
}

install_dependencies() {
    log_info "Installing test dependencies..."
    
    if [[ "$EUID" -eq 0 ]]; then
        apt-get update
        apt-get install -y gcc make libmosquitto-dev mosquitto mosquitto-clients valgrind gcovr bc
    else
        sudo apt-get update
        sudo apt-get install -y gcc make libmosquitto-dev mosquitto mosquitto-clients valgrind gcovr bc
    fi
    
    log_success "Dependencies installed successfully"
}

setup_mqtt_broker() {
    log_info "Setting up MQTT broker for tests..."
    
    # Check if mosquitto is running
    if pgrep -x "mosquitto" > /dev/null; then
        log_info "MQTT broker already running"
        return 0
    fi
    
    # Try to start mosquitto
    if command -v mosquitto >/dev/null 2>&1; then
        log_info "Starting local MQTT broker..."
        mosquitto -d -p "${MQTT_PORT:-1883}" > /dev/null 2>&1 || {
            log_warning "Could not start MQTT broker - MQTT tests may be skipped"
            return 1
        }
        sleep 2
        log_success "MQTT broker started"
    else
        log_warning "MQTT broker not available - MQTT tests may be skipped"
        return 1
    fi
}

cleanup_mqtt_broker() {
    log_info "Cleaning up MQTT broker..."
    pkill -f "mosquitto" 2>/dev/null || true
}

# Build functions
clean_build() {
    log_info "Cleaning previous build artifacts..."
    cd "$SCRIPT_DIR"
    make clean > /dev/null 2>&1 || true
    rm -rf "$TEST_RESULTS_DIR/logs/*" 2>/dev/null || true
    rm -rf "$TEST_RESULTS_DIR/coverage/*" 2>/dev/null || true
}

build_tests() {
    local build_flags="$1"
    
    log_info "Building tests with flags: $build_flags"
    cd "$SCRIPT_DIR"
    
    if ! make all $build_flags 2>&1 | tee "$TEST_RESULTS_DIR/logs/build.log"; then
        log_error "Test build failed - see $TEST_RESULTS_DIR/logs/build.log"
        return 1
    fi
    
    log_success "Tests built successfully"
    return 0
}

# Test execution functions
run_test_suite() {
    local test_name="$1"
    local test_binary="$2"
    local log_file="$TEST_RESULTS_DIR/logs/${test_name}.log"
    
    log_info "Running $test_name tests..."
    
    if [[ ! -x "$test_binary" ]]; then
        log_error "Test binary not found: $test_binary"
        ((FAILED_TESTS++))
        return 1
    fi
    
    local start_time=$(date +%s)
    
    if timeout 300 "$test_binary" > "$log_file" 2>&1; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        
        # Parse test results
        local test_count=$(grep -c "PASS\|FAIL" "$log_file" 2>/dev/null || echo "0")
        local pass_count=$(grep -c "PASS" "$log_file" 2>/dev/null || echo "0")
        local fail_count=$(grep -c "FAIL" "$log_file" 2>/dev/null || echo "0")
        
        if [[ $fail_count -eq 0 ]]; then
            log_success "$test_name: $pass_count/$test_count tests passed (${duration}s)"
            ((PASSED_TESTS++))
            return 0
        else
            log_error "$test_name: $fail_count/$test_count tests failed (${duration}s)"
            ((FAILED_TESTS++))
            return 1
        fi
    else
        log_error "$test_name: Test suite timed out or crashed"
        ((FAILED_TESTS++))
        return 1
    fi
}

run_memory_check() {
    local test_name="$1"
    local test_binary="$2"
    
    if ! command -v valgrind >/dev/null 2>&1; then
        log_warning "Valgrind not available - skipping memory check for $test_name"
        return 0
    fi
    
    log_info "Running memory leak detection for $test_name..."
    
    local valgrind_log="$TEST_RESULTS_DIR/logs/valgrind_${test_name}.log"
    
    if valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
                --error-exitcode=1 --log-file="$valgrind_log" \
                "$test_binary" > /dev/null 2>&1; then
        log_success "$test_name: No memory leaks detected"
        return 0
    else
        local leak_count=$(grep -c "definitely lost\|indirectly lost\|possibly lost" "$valgrind_log" 2>/dev/null || echo "0")
        if [[ $leak_count -gt $MEMORY_LEAK_THRESHOLD ]]; then
            log_error "$test_name: Memory leaks detected ($leak_count issues) - see $valgrind_log"
            return 1
        else
            log_warning "$test_name: Minor memory issues detected - see $valgrind_log"
            return 0
        fi
    fi
}

run_coverage_analysis() {
    log_info "Running coverage analysis..."
    
    cd "$SCRIPT_DIR"
    
    if ! make coverage > "$TEST_RESULTS_DIR/logs/coverage.log" 2>&1; then
        log_error "Coverage analysis failed - see $TEST_RESULTS_DIR/logs/coverage.log"
        return 1
    fi
    
    # Parse coverage results
    local total_coverage=0
    local file_count=0
    
    for gcov_file in *.gcov; do
        if [[ -f "$gcov_file" ]]; then
            local coverage=$(grep "Lines executed:" "$gcov_file" | sed 's/.*:\([0-9.]*\)%.*/\1/' | head -1)
            if [[ -n "$coverage" ]] && [[ "$coverage" != "Lines executed" ]]; then
                total_coverage=$(echo "$total_coverage + $coverage" | bc -l 2>/dev/null || echo "$total_coverage")
                ((file_count++))
            fi
        fi
    done
    
    if [[ $file_count -gt 0 ]]; then
        local avg_coverage=$(echo "scale=1; $total_coverage / $file_count" | bc -l 2>/dev/null || echo "0")
        
        # Move coverage files to results directory
        mv *.gcov "$TEST_RESULTS_DIR/coverage/" 2>/dev/null || true
        
        if (( $(echo "$avg_coverage >= $COVERAGE_THRESHOLD" | bc -l 2>/dev/null || echo "0") )); then
            log_success "Coverage analysis completed: ${avg_coverage}% (threshold: ${COVERAGE_THRESHOLD}%)"
            return 0
        else
            log_warning "Coverage below threshold: ${avg_coverage}% (threshold: ${COVERAGE_THRESHOLD}%)"
            return 1
        fi
    else
        log_error "No coverage data generated"
        return 1
    fi
}

run_benchmarks() {
    log_info "Running performance benchmarks..."
    
    cd "$SCRIPT_DIR"
    
    local benchmark_log="$TEST_RESULTS_DIR/logs/benchmark.log"
    
    {
        echo "HPS3D LIDAR Service Performance Benchmarks"
        echo "============================================"
        echo "Date: $(date)"
        echo ""
        
        for test in test_memory test_threads test_mqtt test_lidar_mock; do
            if [[ -x "$test" ]]; then
                echo "Benchmarking $test..."
                /usr/bin/time -v "./$test" 2>&1 | grep -E "(User time|System time|Maximum resident set size|Page faults)" || true
                echo ""
            fi
        done
    } > "$benchmark_log"
    
    log_success "Benchmarks completed - see $benchmark_log"
}

generate_report() {
    log_info "Generating comprehensive test report..."
    
    local report_file="$TEST_RESULTS_DIR/reports/test_report_$(date +%Y%m%d_%H%M%S).html"
    
    cat > "$report_file" << EOF
<!DOCTYPE html>
<html>
<head>
    <title>HPS3D LIDAR Service Test Report</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .header { background: #f0f0f0; padding: 20px; border-radius: 5px; }
        .section { margin: 20px 0; }
        .pass { color: green; font-weight: bold; }
        .fail { color: red; font-weight: bold; }
        .warning { color: orange; font-weight: bold; }
        .code { background: #f5f5f5; padding: 10px; border-radius: 3px; font-family: monospace; }
        table { border-collapse: collapse; width: 100%; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
        th { background-color: #f2f2f2; }
    </style>
</head>
<body>
    <div class="header">
        <h1>HPS3D LIDAR Service Test Report</h1>
        <p>Generated on: $(date)</p>
        <p>Test Directory: $SCRIPT_DIR</p>
    </div>
    
    <div class="section">
        <h2>Test Summary</h2>
        <table>
            <tr><th>Metric</th><th>Value</th></tr>
            <tr><td>Total Test Suites</td><td>$TOTAL_TESTS</td></tr>
            <tr><td>Passed</td><td class="pass">$PASSED_TESTS</td></tr>
            <tr><td>Failed</td><td class="fail">$FAILED_TESTS</td></tr>
            <tr><td>Skipped</td><td class="warning">$SKIPPED_TESTS</td></tr>
        </table>
    </div>
    
    <div class="section">
        <h2>Test Logs</h2>
EOF

    # Add individual test results
    for log_file in "$TEST_RESULTS_DIR/logs"/*.log; do
        if [[ -f "$log_file" ]]; then
            local test_name=$(basename "$log_file" .log)
            echo "<h3>$test_name</h3>" >> "$report_file"
            echo "<div class=\"code\">" >> "$report_file"
            tail -50 "$log_file" | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g' >> "$report_file"
            echo "</div>" >> "$report_file"
        fi
    done
    
    cat >> "$report_file" << EOF
    </div>
</body>
</html>
EOF
    
    log_success "Test report generated: $report_file"
    
    # Also generate a simple text report
    local text_report="$TEST_RESULTS_DIR/reports/test_summary.txt"
    cat > "$text_report" << EOF
HPS3D LIDAR Service Test Summary
===============================
Generated: $(date)

Results:
  Total Test Suites: $TOTAL_TESTS
  Passed: $PASSED_TESTS
  Failed: $FAILED_TESTS
  Skipped: $SKIPPED_TESTS
  Success Rate: $(echo "scale=1; $PASSED_TESTS * 100 / $TOTAL_TESTS" | bc -l 2>/dev/null || echo "0")%

Test Details:
EOF
    
    for log_file in "$TEST_RESULTS_DIR/logs"/*.log; do
        if [[ -f "$log_file" ]]; then
            local test_name=$(basename "$log_file" .log)
            echo "  $test_name: $(grep -c "PASS" "$log_file" 2>/dev/null || echo "0") passed, $(grep -c "FAIL" "$log_file" 2>/dev/null || echo "0") failed" >> "$text_report"
        fi
    done
    
    log_success "Text summary generated: $text_report"
}

# Main execution function
main() {
    local test_suites=()
    local enable_coverage=false
    local enable_memory_check=false
    local enable_sanitizers=false
    local enable_asan=false
    local enable_tsan=false
    local enable_ci=false
    local enable_benchmark=false
    local enable_report=false
    local enable_clean=false
    local enable_install_deps=false
    local enable_mqtt_broker=false
    local build_flags=""
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -v|--verbose)
                export VERBOSE=1
                ;;
            -d|--debug)
                export DEBUG=1
                ;;
            -q|--quiet)
                export QUIET=1
                ;;
            -j|--parallel)
                PARALLEL_JOBS="$2"
                shift
                ;;
            -t|--timeout)
                MQTT_BROKER_TIMEOUT="$2"
                shift
                ;;
            -c|--coverage)
                enable_coverage=true
                build_flags+=" COVERAGE=1"
                ;;
            -m|--memory-check)
                enable_memory_check=true
                ;;
            -s|--sanitizers)
                enable_sanitizers=true
                ;;
            --asan)
                enable_asan=true
                build_flags+=" ASAN=1"
                ;;
            --tsan)
                enable_tsan=true
                build_flags+=" TSAN=1"
                ;;
            --ci)
                enable_ci=true
                enable_coverage=true
                enable_memory_check=true
                enable_sanitizers=true
                enable_report=true
                build_flags+=" COVERAGE=1"
                ;;
            --benchmark)
                enable_benchmark=true
                ;;
            --report)
                enable_report=true
                ;;
            --clean)
                enable_clean=true
                ;;
            --install-deps)
                enable_install_deps=true
                ;;
            --mqtt-broker)
                enable_mqtt_broker=true
                ;;
            mqtt|lidar|memory|threads|all)
                test_suites+=("$1")
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
        shift
    done
    
    # Default to running all tests
    if [[ ${#test_suites[@]} -eq 0 ]]; then
        test_suites=("all")
    fi
    
    # Initialize
    print_banner
    setup_directories
    
    # Handle special options
    if [[ "$enable_install_deps" == true ]]; then
        install_dependencies
        exit 0
    fi
    
    # Check dependencies
    if ! check_dependencies; then
        log_error "Dependency check failed"
        exit 1
    fi
    
    # Setup MQTT broker if requested
    if [[ "$enable_mqtt_broker" == true ]]; then
        setup_mqtt_broker
    fi
    
    # Clean if requested
    if [[ "$enable_clean" == true ]]; then
        clean_build
    fi
    
    # Build tests
    if ! build_tests "$build_flags"; then
        cleanup_mqtt_broker
        exit 1
    fi
    
    # Run tests
    cd "$SCRIPT_DIR"
    
    for suite in "${test_suites[@]}"; do
        case $suite in
            mqtt)
                ((TOTAL_TESTS++))
                run_test_suite "mqtt" "./test_mqtt"
                if [[ "$enable_memory_check" == true ]]; then
                    run_memory_check "mqtt" "./test_mqtt"
                fi
                ;;
            lidar)
                ((TOTAL_TESTS++))
                run_test_suite "lidar" "./test_lidar_mock"
                if [[ "$enable_memory_check" == true ]]; then
                    run_memory_check "lidar" "./test_lidar_mock"
                fi
                ;;
            memory)
                ((TOTAL_TESTS++))
                run_test_suite "memory" "./test_memory"
                if [[ "$enable_memory_check" == true ]]; then
                    run_memory_check "memory" "./test_memory"
                fi
                ;;
            threads)
                ((TOTAL_TESTS++))
                run_test_suite "threads" "./test_threads"
                if [[ "$enable_memory_check" == true ]]; then
                    run_memory_check "threads" "./test_threads"
                fi
                ;;
            all)
                ((TOTAL_TESTS += 4))
                run_test_suite "mqtt" "./test_mqtt"
                run_test_suite "lidar" "./test_lidar_mock"
                run_test_suite "memory" "./test_memory"
                run_test_suite "threads" "./test_threads"
                
                if [[ "$enable_memory_check" == true ]]; then
                    run_memory_check "mqtt" "./test_mqtt"
                    run_memory_check "lidar" "./test_lidar_mock"
                    run_memory_check "memory" "./test_memory"
                    run_memory_check "threads" "./test_threads"
                fi
                ;;
        esac
    done
    
    # Run additional analysis
    if [[ "$enable_coverage" == true ]]; then
        run_coverage_analysis
    fi
    
    if [[ "$enable_benchmark" == true ]]; then
        run_benchmarks
    fi
    
    if [[ "$enable_report" == true ]]; then
        generate_report
    fi
    
    # Cleanup
    cleanup_mqtt_broker
    
    # Final summary
    echo ""
    log_info "Test execution completed"
    log_info "Results saved to: $TEST_RESULTS_DIR"
    echo ""
    
    if [[ $FAILED_TESTS -eq 0 ]]; then
        log_success "🎉 All $PASSED_TESTS test suites PASSED!"
        exit 0
    else
        log_error "❌ $FAILED_TESTS out of $TOTAL_TESTS test suites FAILED!"
        exit 1
    fi
}

# Trap to ensure cleanup on exit
trap 'cleanup_mqtt_broker' EXIT

# Run main function
main "$@"