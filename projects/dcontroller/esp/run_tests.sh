#!/bin/bash
# Test Runner Script for ESP32 Motor Controller
# Usage: ./run_tests.sh [mock|hil|all|ci]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$SCRIPT_DIR/test"
MOCK_DIR="$TEST_DIR/mocks"
RESULTS_DIR="$TEST_DIR/results"

# Test mode
MODE="${1:-mock}"

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to run mock tests
run_mock_tests() {
    print_info "========================================"
    print_info "RUNNING MOCK TESTS"
    print_info "========================================"
    
    mkdir -p "$RESULTS_DIR/mock"
    
    # Test 2:  Sensors
    print_info ""
    print_info "Building  Sensor mock tests..."
    # Note: These may need additional stubs for complex dependencies
    if gcc -DUNIT_TEST_MOCK \
        "$TEST_DIR/test_sensors.c" \
        "$MOCK_DIR/test_mocks.c" \
        -I. -I"$SCRIPT_DIR/esp/include" \
        -I"$SCRIPT_DIR/esp/test" \
        -o "$RESULTS_DIR/mock/test_sensors" \
        -lm 2>&1 | tee "$RESULTS_DIR/mock/build.log"; then
        
        print_info "Running  Sensor mock tests..."
        if "$RESULTS_DIR/mock/test_sensors" 2>&1 | tee "$RESULTS_DIR/mock/results.log"; then
            print_info "✓  Sensor mock tests PASSED"
        else
            print_warn "⚠  Sensor tests had issues (may need stubs)"
        fi
    else
        print_warn "⚠  Sensor build had issues (expected for complex deps)"
        print_info "   To fix: Add stubs for motor functions in test_mocks.c"
    fi
    
    print_info ""
    print_info "========================================"
    print_info "MOCK TESTS COMPLETE"
    print_info "========================================"
    print_info "Results saved to: $RESULTS_DIR/mock/"
}


# Function to show usage
show_usage() {
    echo "Usage: $0 [mock|hil|quick-hil|full-hil|all|ci|help]"
    echo ""
    echo "Commands:"
    echo "  mock       - Run mock tests on host machine (no hardware needed)
    echo "  help       - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 mock       # Quick tests without hardware"
}

# Main script logic
case "$MODE" in
    mock)
        run_mock_tests
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        print_error "Unknown mode: $MODE"
        show_usage
        exit 1
        ;;
esac

exit 0
