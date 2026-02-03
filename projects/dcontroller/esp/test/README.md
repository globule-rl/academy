# ESP32 Motor Controller - Test System

This directory contains comprehensive test suites for the ESP32 Dual Motor Desk Controller.

## 📁 Test Structure

```
/test
├── mocks/                    # Mock implementations for host testing
│   ├── test_mocks.h         # Mock definitions
│   └── test_mocks.c         # Mock implementations
├── test_calc.c              # Calculator unit tests
├── test_wire_sensors.c      # Two-wire sensor tests (mock + HIL)
├── test_hil_full.c          # FULL hardware-in-loop test suite (15 tests)
├── results/                 # Test results storage
│   ├── mock/               # Mock test outputs
│   └── hil/                # Hardware-in-loop outputs
│       ├── quick_results.log    # Quick HIL (no motors)
│       └── full_results.log     # Full HIL (with motors)
└── README.md               # This file
```

## 🧪 Test Types

### 1. Mock Tests (Host Machine)
Run on your computer without any ESP32 hardware.

**Purpose:**
- Test logic and algorithms
- Fast feedback during development
- CI/CD integration
- No hardware required

**Running Mock Tests:**
```bash
./run_tests.sh mock
```

**What's Tested:**
- Calculator functions (add, sub, mul, div)
- Two-wire sensor logic
- State machine behavior
- Math calculations

### 2. Hardware-in-Loop (HIL) Tests
Run on actual ESP32 hardware with all peripherals connected.

#### A. Quick HIL (Safe - No Motors Move)
Tests all sensors and inputs without moving motors.

```bash
./run_tests.sh quick-hil
```

**15 Tests Included:**
1. ✓ GPIO initialization (all pins)
2. ✓ PWM output verification
3. ✓ Hall sensor reading
4. ✓ Current sensor (ACS712) readings
5. ✓ Limit switches (4 total)
6. ✓ Two-wire sensors (red/yellow)
7. ✓ Button inputs (4 buttons)
8. ✓ Height calculation
9. ✓ NVS/memory positions
10. ✓ Motor sync check
11. ✓ Safety system status
12. ✓ Emergency stop functional
13. ✓ Overcurrent detection
14. ✓ Temperature monitoring
15. ✓ Configuration validation

#### B. Full HIL (Complete - Motors Move)
Comprehensive test including motor movement.

```bash
./run_tests.sh full-hil
# or
./run_tests.sh hil  # Interactive menu
```

**Additional Tests:**
- ✓ Motor movement UP (2 seconds)
- ✓ Motor movement DOWN (2 seconds)
- ✓ Motor synchronization (dual-motor tracking)
- ✓ Emergency stop functionality
- ✓ Limit switch stop verification
- ✓ Real-world safety scenarios

## 🚀 Quick Start

```bash
# Development (no hardware)
./run_tests.sh mock

# Hardware validation (safe)
./run_tests.sh quick-hil

# Complete validation (motors move)
./run_tests.sh full-hil

# Everything
./run_tests.sh all

# Interactive menu
./run_tests.sh hil
```

## 🔌 Hardware Connection for HIL

```
ESP32 GPIO Mapping:

Motor 1:
  PWM: GPIO25, DIR: GPIO26, Hall: GPIO34
  Current: GPIO32 (ADC), Red: GPIO22, Yellow: GPIO23
  Top Limit: GPIO18, Bottom Limit: GPIO19

Motor 2:
  PWM: GPIO27, DIR: GPIO14, Hall: GPIO35
  Current: GPIO33 (ADC), Red: GPIO0, Yellow: GPIO2
  Top Limit: GPIO20, Bottom Limit: GPIO21

Interface:
  UP: GPIO4, DOWN: GPIO16, M1: GPIO17, M2: GPIO5
  I2C SDA: GPIO21, I2C SCL: GPIO22
```

## 🔧 Two-Wire Sensor Configuration

After testing with multimeter, edit `include/motor/pin.h`:

```c
// Logic: 0=active LOW, 1=active HIGH
#define RED_WIRE_LOGIC        0
#define YELLOW_WIRE_LOGIC     0

// Function: WIRE_THERMAL_CUTOFF, WIRE_THERMAL_WARNING, etc.
#define RED_WIRE_FUNCTION     WIRE_THERMAL_CUTOFF
#define YELLOW_WIRE_FUNCTION  WIRE_THERMAL_WARNING
```

## 📊 Test Results

Saved to `esp/test/results/`:

**Mock Tests:**
- `mock/calc_results.log`
- `mock/wire_results.log`

**HIL Tests:**
- `hil/quick_results.log`
- `hil/full_results.log`

**CI/CD:**
- `build.log` - Compilation
- `check.log` - Static analysis

## 🧪 Full HIL Test Detail

### Test Sequence:

1. **GPIO Init** - Verify 20 pins readable
2. **PWM Output** - Check duty cycles
3. **Hall Sensors** - Verify incrementing
4. **Current Sensors** - Read ACS712 (<500mA stopped)
5. **Limit Switches** - Read 4 switches
6. **Two-Wire** - Check red/yellow
7. **Buttons** - Verify 4 buttons
8. **Height Calc** - Valid range check
9. **Memory** - NVS read
10. **Motor UP** - Move 2s, verify hall delta
11. **Motor DOWN** - Move 2s, verify hall delta
12. **Motor Sync** - Both within 10%
13. **Emergency Stop** - Immediate stop
14. **Safety OC** - Overcurrent detection
15. **Safety Limit** - Limit switch stops

### Expected Output:
```
========================================
HARDWARE-IN-LOOP TEST RESULTS
========================================
 1. GPIO Init           : ✓ PASS (120 ms)
    GPIO: 20/20 pins readable
 2. PWM Output          : ✓ PASS (50 ms)
    PWM Duty: M1=0 M2=0 (expected 0,0)
 ...
15. Safety Limit        : ✓ PASS (30 ms)
Results: 15 passed, 0 failed
========================================
```

## 🐛 Debugging

**Mock Tests Fail:**
```bash
cat esp/test/results/mock/calc_results.log
# Missing stubs? Add to test_mocks.c
```

**HIL Tests Fail:**
```bash
pio device monitor -b 115200
# GPIO fail → Check pin.h
# Hall fail → Check GPIO34/35 wiring
# Current fail → Check ACS712
# Motors fail → Check power supply
# Sync fail → Check both motors connected
```

## 📋 Checklists

**Before Full HIL:**
- [ ] Power supply ON
- [ ] Motors wired correctly
- [ ] Sensors connected
- [ ] Desk clear
- [ ] Emergency stop ready

**Before Commit:**
- [ ] `./run_tests.sh mock` passes
- [ ] `pio run` compiles
- [ ] `pio check` clean

**Before Production:**
- [ ] `./run_tests.sh quick-hil` passes
- [ ] `./run_tests.sh full-hil` passes
- [ ] Safety verified
- [ ] Sync confirmed
- [ ] Height calibrated

## 📚 Resources

- [Unity Framework](https://github.com/ThrowTheSwitch/Unity)
- [PlatformIO Testing](https://docs.platformio.org/en/latest/plus/unit-testing.html)
- [ESP-IDF Testing](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/unit-tests.html)

## ✨ Summary

**Three Test Levels:**
1. **Mock** - Host only, fast, no hardware
2. **Quick HIL** - All sensors, no motor movement
3. **Full HIL** - Complete, motors move, full validation
