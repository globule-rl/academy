# Two-Wire Sensor & Full HIL Test Implementation Summary

## ✅ What Was Implemented

### 1. Two-Wire Sensor System (pin.h, motor.h, motor.cc)

**Configuration (easily adjustable after testing):**
```c
// Pin definitions
#define M1_RED_PIN    GPIO_NUM_22
#define M1_YELLOW_PIN GPIO_NUM_23
#define M2_RED_PIN    GPIO_NUM_0
#define M2_YELLOW_PIN GPIO_NUM_2

// Logic (change after multimeter testing)
#define RED_WIRE_LOGIC        0     // 0=active LOW, 1=active HIGH
#define YELLOW_WIRE_LOGIC     0

// Function (change based on what wires actually do)
#define RED_WIRE_FUNCTION     WIRE_THERMAL_CUTOFF
#define YELLOW_WIRE_FUNCTION  WIRE_THERMAL_WARNING

// Wire function options
enum WireFunction {
    WIRE_UNKNOWN = 0,
    WIRE_THERMAL_CUTOFF,    // Emergency stop
    WIRE_THERMAL_WARNING,   // Warning only
    WIRE_TACHOMETER,        // Speed sensing
    WIRE_ENDSTOP_TOP,       // Top limit
    WIRE_ENDSTOP_BOTTOM     // Bottom limit
};
```

**Features:**
- ✓ Separate ISRs for red and yellow wires
- ✓ Independent alarm tracking per motor
- ✓ Temperature estimation from wire state
- ✓ Safety check integration
- ✓ Thread-safe updates

### 2. Mock Test Infrastructure

**Files:**
- `test/mocks/test_mocks.h` - Mock definitions
- `test/mocks/test_mocks.c` - Mock implementations

**Mocks:**
- FreeRTOS (tasks, queues, semaphores, delays)
- GPIO (get/set level, configure)
- ADC (read raw, set raw for testing)
- LEDC (PWM duty control)
- ESP Logging

**Run Mock Tests:**
```bash
./run_tests.sh mock
```

### 3. Hardware-in-Loop (HIL) Test Suite

**Two Modes:**

#### A. Quick HIL (No Motors Move - Safe)
Tests 15 hardware components without motor movement:

```bash
./run_tests.sh quick-hil
```

**Tests:**
1. GPIO initialization (20 pins)
2. PWM output verification
3. Hall sensor reading
4. Current sensor (ACS712)
5. Limit switches (4 total)
6. Two-wire sensors (red/yellow)
7. Button inputs (4 buttons)
8. Height calculation
9. Memory positions (NVS)
10. Safety system status
11. Emergency stop
12. Overcurrent detection
13. Temperature monitoring
14. Motor sync check
15. Configuration validation

#### B. Full HIL (Motors Move - Complete)
All Quick HIL tests + motor movement:

```bash
./run_tests.sh full-hil
```

**Additional Tests:**
- ✓ Motor UP (2 seconds, verify hall delta)
- ✓ Motor DOWN (2 seconds, verify hall delta)
- ✓ Motor sync (both motors within 10%)
- ✓ Emergency stop (immediate stop)
- ✓ Safety scenarios (limit switch stop)

**Run with Interactive Menu:**
```bash
./run_tests.sh hil
```

### 4. CI/CD Configuration

**File:** `.github/workflows/tests.yml`

**Automatic on push/PR:**
1. Mock tests (calculator, wire sensors)
2. Compilation test (PlatformIO)
3. Static analysis (clang-tidy)
4. Artifact upload

**Trigger:**
```bash
git push origin main  # CI runs automatically!
```

### 5. Test Runner Script

**File:** `run_tests.sh`

**Commands:**
```bash
./run_tests.sh mock        # Host-only tests
./run_tests.sh quick-hil   # Safe hardware tests
./run_tests.sh full-hil    # Complete hardware tests
./run_tests.sh hil         # Interactive menu
./run_tests.sh all         # Mock + Full HIL
./run_tests.sh ci          # Non-interactive mode
```

## 📋 After Power Supply Arrives

### Step 1: Test Wires with Multimeter

Measure at motor end:
- Red wire voltage (normal vs alarm)
- Yellow wire voltage (normal vs alarm)
- Note if active LOW (0V) or HIGH (3.3V)

### Step 2: Update Configuration

Edit `include/motor/pin.h`:
```c
// Based on your measurements:
#define RED_WIRE_LOGIC        0     // or 1
#define YELLOW_WIRE_LOGIC     0     // or 1

// Based on wire behavior:
#define RED_WIRE_FUNCTION     WIRE_THERMAL_CUTOFF  // adjust as needed
#define YELLOW_WIRE_FUNCTION  WIRE_THERMAL_WARNING // adjust as needed
```

### Step 3: Run Tests

```bash
# Safe initial test (no motors)
./run_tests.sh quick-hil

# Check serial monitor for results:
# pio device monitor -b 115200

# Once all sensors pass, run full test:
./run_tests.sh full-hil
```

### Step 4: Verify All Tests Pass

Expected output:
```
========================================
HARDWARE-IN-LOOP TEST RESULTS
========================================
 1. GPIO Init           : ✓ PASS (120 ms)
 2. PWM Output          : ✓ PASS (50 ms)
 3. Hall Sensors        : ✓ PASS (510 ms)
 ...
15. Safety Limit        : ✓ PASS (30 ms)
========================================
Results: 15 passed, 0 failed
========================================
```

## 🔧 File Summary

| File | Purpose |
|------|---------|
| `include/motor/pin.h` | Two-wire pin config + WireFunction enum |
| `include/motor/motor.h` | Motor state with wire alarm fields |
| `src/motor.cc` | Sensor ISRs, safety check, temperature calc |
| `test/mocks/test_mocks.h/.c` | Mock implementations for host testing |
| `test/test_wire_sensors.c` | Mock + HIL tests for two-wire sensors |
| `test/test_hil_full.c` | **Complete** 15-test HIL suite |
| `test/README.md` | Comprehensive test documentation |
| `run_tests.sh` | Test runner with all modes |
| `.github/workflows/tests.yml` | CI/CD pipeline |

## 🎯 Test Coverage

**Mock Tests (No Hardware):**
- ✓ Calculator functions
- ✓ Two-wire sensor logic
- ✓ State machine behavior
- ✓ Math calculations

**Quick HIL (Safe Hardware):**
- ✓ All GPIO pins (20 pins)
- ✓ All sensors (hall, current, limits, two-wire)
- ✓ User interface (buttons)
- ✓ Non-volatile storage
- ✓ Safety systems (logic only)

**Full HIL (Complete Hardware):**
- ✓ Everything in Quick HIL
- ✓ Motor movement (UP/DOWN)
- ✓ Motor synchronization
- ✓ Real-world safety scenarios
- ✓ Emergency stop
- ✓ Limit switch stops

## 🚀 Usage Examples

**Development workflow:**
```bash
# Make code changes
# Run mock tests to verify logic
./run_tests.sh mock

# Commit if tests pass
git add . && git commit -m "Fix: motor sync logic"

# CI automatically runs mock tests

# When ready for hardware testing:
./run_tests.sh quick-hil

# Final validation before deployment:
./run_tests.sh full-hil
```

**Troubleshooting:**
```bash
# Mock test fails - check build log
cat esp/test/results/mock/calc_build.log

# HIL test fails - check serial output
pio device monitor -b 115200

# Check which test failed
cat esp/test/results/hil/full_results.log
```

## 📝 Notes

1. **Two-wire sensors** are highly configurable - just change 4 lines in `pin.h` after testing

2. **HIL tests** have two modes:
   - Quick (safe, no motors move) - run anytime
   - Full (complete, motors move) - run when ready

3. **Mock tests** run on your computer - good for development

4. **CI/CD** runs automatically - no manual steps needed

5. **All test results** saved to `esp/test/results/` for debugging

## ✨ Complete Implementation Status

- ✅ Two-wire sensor code (configurable)
- ✅ Mock test infrastructure
- ✅ Full HIL test suite (15 tests)
- ✅ Test runner script
- ✅ CI/CD pipeline
- ✅ Complete documentation
- ✅ GPIO mapping reference
- ✅ Troubleshooting guide

**Ready for power supply and testing!**
