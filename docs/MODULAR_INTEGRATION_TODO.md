# Modular Architecture Integration - TODO

## Overview

The project has **modular controller files** in `controllers/` and root directory that implement a sophisticated architecture with:

- Motor controller (dual motor coordination)
- Hardware controller (ADC, PWM, GPIO abstraction)
- Safety controller (fault detection, limits)
- Communication manager (UART, SPI, OLED)
- System diagnostics (performance monitoring)

**Current Status:** These files are NOT yet integrated into the build due to struct/type mismatches with the existing FOC code.

---

## Files Awaiting Integration

### Root Directory

- `main_controller.c` / `main_controller.h` (15.8 KB / 9.4 KB)
- `main_controller_v3.c` (23.7 KB) - **Latest version with sub-controllers**
- `communication_manager.c` / `communication_manager.h` (20 KB / 8 KB)
- `system_diagnostics.c` / `system_diagnostics.h` (21 KB / 12 KB)

### Controllers Directory

- `controllers/motor_controller.c` / `motor_controller.h`
- `controllers/hardware_controller.c` / `hardware_controller.h`
- `controllers/safety_controller.c` / `safety_controller.h`
- `controllers/communication_controller.c`

---

## Issues to Fix

### 1. **Struct Member Mismatches**

#### `CurLoop_Obj` (Foc/inc/CurrentLoop.h)

**Problem:** `motor_controller.c` line 234 tries to access `.IqTarget` which doesn't exist

```c
// ❌ DOESN'T WORK:
motor_I[motor_id].CurLoop.IqTarget = torque_current;

// ✅ ACTUAL FIELDS AVAILABLE:
typedef struct _CURLOOP_OBJ_ {
    float IdFilterTmp, IqFilterTmp;
    float IdFilter, IqFilter;
    int16_t IdFedBak, IqFedBak;  // ← Use these for feedback
    int16_t PiIdSum, PiIqSum;
    int16_t PiIdOut, PiIqOut;
    int32_t UdOut_Q15, UqOut_Q15;
    // ... no IqTarget field
} CurLoop_Obj;
```

**Fix:** Need to set torque via existing FOC functions or add `.IqTarget` field to struct

---

#### `MotorFlag_Obj` (Foc/inc/ErrDeal.h)

**Problem:** Multiple references to `.Flag.Reg` which doesn't exist

```c
// ❌ DOESN'T WORK:
motor_I[motor_id].Flag.Reg = 0;

// ✅ ACTUAL STRUCTURE:
typedef union _MOTOR_FLAG_ALL_ {
    uint16_t Arr[2];  // ← Use this instead
    struct {
        uint16_t SwOverCurrent : 1;  // ← Note: "Sw" prefix!
        uint16_t HwOverCurrent : 1;
        uint16_t OverVolatage : 1;   // ← Note: typo in original!
        uint16_t LackVolatage : 1;   // ← Note: typo in original!
        // ... many more flags
    } Bits;
} MotorFlag_Obj;
```

**Fixes Applied:**

- ✅ Changed `.Flag.Reg` → `.Flag.Arr[0]`
- ✅ Changed `.OverCurrent` → `.SwOverCurrent`
- ✅ Changed `.OverVoltage` → `.OverVolatage` (matches typo in original)
- ✅ Changed `.LackVoltage` → `.LackVolatage` (matches typo in original)

---

#### `MotorFeedback_t` (controllers/motor_controller.h)

**Problem:** `motor_controller.c` line 580 tries to access `.motor_id` which doesn't exist

```c
// ❌ DOESN'T WORK:
motor->feedback.motor_id = motor_id;
```

**Fix:** Either remove this line or add `motor_id` field to `MotorFeedback_t` struct

---

### 2. **Missing Function Implementation**

#### `HardwareController_GetTimestampMs()`

**Problem:** Called in multiple files but not implemented

**Files affected:**

- `main_controller_v3.c` (lines 88, 122, 123)
- `controllers/motor_controller.c` (line 388)
- `system_diagnostics.c` (multiple lines)

**Fix Options:**

1. Implement using SysTick timer counter
2. Use existing timing mechanism from FOC code
3. Add simple millisecond counter incremented in ISR

---

### 3. **Type Definition Mismatches**

#### `system_diagnostics.c` Issues

Multiple undefined types that don't match `system_diagnostics.h`:

- `DiagnosticsAlertLevel_t` (header has `DiagSeverity_t`)
- `DiagnosticsAlertCallback_t` (header has `FaultDetectionCallback_t`)
- `DiagnosticsConfig_t` (not in header)
- `DiagnosticsStatus_t` (not in header)

**Fix:** Rewrite `system_diagnostics.c` to use types from `.h` file

---

## Integration Strategy

### Phase 1: Fix Struct Mismatches ✅ (Partially Done)

- [x] Fix flag name typos (`OverCurrent` → `SwOverCurrent`, etc.)
- [x] Fix `.Flag.Reg` → `.Flag.Arr[0]`
- [ ] Fix `CurLoop.IqTarget` access
- [ ] Fix `MotorFeedback_t.motor_id` access

### Phase 2: Implement Missing Functions

- [ ] Implement `HardwareController_GetTimestampMs()`
- [ ] Test timing accuracy

### Phase 3: Fix Type Definitions

- [ ] Align `system_diagnostics.c` with `.h` file types
- [ ] OR: Update `.h` to match `.c` expectations

### Phase 4: Test Build

- [ ] Uncomment modular files in Makefile
- [ ] Fix any remaining compilation errors
- [ ] Verify firmware size is acceptable

### Phase 5: Runtime Integration

- [ ] Update `App/src/main.c` to call `SystemController_Initialize()`
- [ ] Replace `MotorMain_Circle()` with `SystemController_Execute()`
- [ ] Test on hardware

---

## Benefits Once Integrated

✅ **Modular Design:** Easy to add/modify motor controllers  
✅ **Safety Layer:** Comprehensive fault detection and limits  
✅ **Diagnostics:** Performance monitoring and health checks  
✅ **Communication:** Abstracted UART/SPI/OLED interfaces  
✅ **Scalability:** Clean separation of concerns  
✅ **Maintainability:** Well-documented API boundaries

---

## Current Workaround

For now, the project builds successfully with the **original simple main loop**:

```c
int main() {
    System_Init();
    while(1) {
        MotorMain_Circle();
    }
}
```

This works fine but doesn't use the modular architecture. Once integrated, it will become:

```c
int main() {
    SystemController_Initialize();
    while(1) {
        SystemController_Execute();
    }
}
```

---

## Next Steps

1. **Immediate:** Fix `CurLoop.IqTarget` - most critical blocker
2. **Then:** Implement `HardwareController_GetTimestampMs()`
3. **Finally:** Align `system_diagnostics.c` types with header

**Estimated effort:** 2-4 hours to complete all fixes and test
