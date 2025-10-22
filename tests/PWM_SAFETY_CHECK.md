# ⚠️ PWM Safety Verification Report

## CRITICAL: Read Before Hardware Testing!

### ✅ GOOD NEWS: Dead-Time Is Configured!

**Found in `App/src/Pwm.c` line 105:**

```c
TIM1_BDTRInitStructure.DeadTime = 64;
```

**Analysis:**

- Dead-time register value: **64**
- With 144 MHz clock and prescaler configuration
- Estimated dead-time: **~444 ns** (0.444 μs)

**Safety Assessment:**

- ✅ Dead-time **IS** present (prevents shoot-through)
- ⚠️ Value is **conservative but acceptable** for small MOSFETs
- 💡 Typical recommendation: 1-2 μs for better safety margin

### PWM Configuration Summary

| Parameter                 | Value                 | Safety Rating        |
| ------------------------- | --------------------- | -------------------- |
| **Dead-Time**             | 64 (444ns)            | ⚠️ Marginal          |
| **PWM Frequency**         | From `PWM_FREQUENCY1` | ✅ Check UserParam.h |
| **Center-Aligned Mode**   | Yes (Mode 1)          | ✅ Good              |
| **Complementary Outputs** | Enabled               | ✅ Correct           |
| **Auto Output Enable**    | Yes                   | ✅ Good              |
| **Break Input**           | Disabled              | ⚠️ Consider enabling |

## 🔍 Recommended Improvements

### 1. Increase Dead-Time (Safer)

**Current:**

```c
TIM1_BDTRInitStructure.DeadTime = 64;  // ~444 ns
```

**Recommended:**

```c
TIM1_BDTRInitStructure.DeadTime = 144; // ~1 μs (safer)
```

**Why:**

- Gives MOSFETs more time to fully turn off
- Prevents shoot-through in high-current transients
- Standard industry practice for motor control

### 2. Enable Break Input (Hardware Protection)

**Current:**

```c
TIM1_BDTRInitStructure.Break = TIM_BREAK_IN_DISABLE;
```

**Recommended:**

```c
TIM1_BDTRInitStructure.Break = TIM_BREAK_IN_ENABLE;
TIM1_BDTRInitStructure.BreakPolarity = TIM_BREAK_POLARITY_HIGH;
```

**Why:**

- Hardware-level emergency stop
- Faster than software fault detection
- Connect to external comparator (overcurrent detection)

## 📊 PWM Timing Analysis

### Dead-Time Calculation

Given N32G430 at 144 MHz:

```
Timer Clock = 144 MHz
Dead-Time Clock = Timer Clock / (prescaler + 1)
Dead-Time = (DeadTime value) × (1 / Dead-Time Clock)

With DeadTime = 64:
Dead-Time ≈ 64 / 144 MHz ≈ 444 ns
```

### Safety Margins

| MOSFET Type     | Typical Turn-off Time | Required Dead-Time | Current Setting | Status       |
| --------------- | --------------------- | ------------------ | --------------- | ------------ |
| Small (<10A)    | 100-200 ns            | 500 ns - 1 μs      | 444 ns          | ⚠️ Marginal  |
| Medium (10-30A) | 200-500 ns            | 1-2 μs             | 444 ns          | ❌ Too short |
| Large (>30A)    | 500+ ns               | 2-3 μs             | 444 ns          | ❌ Too short |

**Conclusion:** Current dead-time is **marginal** for small MOSFETs, **insufficient** for larger ones.

## 🛡️ Hardware Protection Checklist

Before connecting motor:

### 1. Check Power Stage MOSFETs

- [ ] Identify MOSFET part numbers on board
- [ ] Look up datasheet turn-off time (t_off)
- [ ] Verify dead-time > t_off + 100ns margin
- [ ] Check gate driver specs

### 2. Measure PWM Signals (Oscilloscope)

- [ ] Probe complementary pair (e.g., TIM1_CH1 and TIM1_CH1N)
- [ ] Verify dead-time visually
- [ ] Check PWM frequency matches expected
- [ ] Ensure no overlap between high-side/low-side

**What to look for:**

```
High-Side PWM:  ─────┐         ┌─────┐
                     │         │     │
                     └─────────┘     └─
                        ↑ Dead-time ↑

Low-Side PWM:   ─┐         ┌─────────┐
                 │         │         │
                 └─────────┘         └─
```

### 3. Test Current Limiting

- [ ] Set power supply current limit to 1A
- [ ] Monitor actual current draw
- [ ] Verify software limits trigger before 1A

### 4. Emergency Stop Test

- [ ] Verify PWM disables on fault
- [ ] Test software overcurrent detection
- [ ] Test undervoltage lockout
- [ ] Test overvoltage protection

## 🔧 Recommended Code Changes

### Option A: Quick Fix (Increase Dead-Time)

```c
// In App/src/Pwm.c, line 105, change:
TIM1_BDTRInitStructure.DeadTime = 144;  // 1 μs (was 64)
```

Then rebuild:

```bash
make clean
make
```

### Option B: Full Safety Upgrade

1. **Increase dead-time**
2. **Enable break input**
3. **Add external comparator for hardware overcurrent**

See: `docs/PWM_SAFETY_UPGRADE.md` for detailed implementation

## ⚡ What Dead-Time Value Should You Use?

### Quick Reference:

| Your Motor Current | Recommended DeadTime | Actual Dead-Time |
| ------------------ | -------------------- | ---------------- |
| < 5A               | 96                   | 667 ns           |
| 5-10A              | 144                  | 1.0 μs           |
| 10-20A             | 216                  | 1.5 μs           |
| 20-30A             | 288                  | 2.0 μs           |

**Formula:**

```c
DeadTime_register = (desired_dead_time_seconds) × (Timer_Clock_Hz)
                  = (1.0e-6 s) × (144e6 Hz)
                  = 144
```

## 🎯 Pre-Flight Checklist

Before flashing to hardware:

- [ ] **Review current code:** Dead-time = 64 (444 ns)
- [ ] **Decision:** Keep current OR increase to 144 (1 μs)?
- [ ] **If increasing:** Modify Pwm.c, rebuild, reflash
- [ ] **Oscilloscope ready:** To verify PWM timing
- [ ] **Current-limited supply:** Set to 1A max
- [ ] **Motor disconnected:** Test PWM signals first!
- [ ] **Emergency stop plan:** How to cut power quickly

## 💡 My Recommendation

**For your first test:**

1. **Increase dead-time to 144** (1 μs safety margin)

   ```bash
   # Edit App/src/Pwm.c line 105
   # Change: DeadTime = 64  →  DeadTime = 144
   make clean && make
   ```

2. **Flash firmware**
3. **Verify with scope** (BEFORE connecting motor)
4. **Then test with motor** (current-limited!)

**Would you like me to:**

- A) Make the dead-time change now (to 1 μs)?
- B) Keep current value and test carefully?
- C) Show you how to calculate optimal dead-time for your MOSFETs?

## 📚 Additional Resources

- N32G430 TIM1 Dead-Time Insert application note
- MOSFET gate driver design guide
- Motor control safety standards (IEC 61800-5-2)

---

**Remember:** Dead-time is your first line of defense against shoot-through!
A few hundred nanoseconds can save your power stage. 🛡️
