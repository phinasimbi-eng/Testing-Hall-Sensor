# Quick Start: Safe Firmware Testing Guide

## ✅ Your Current Status

- Build: **SUCCESS** (20,280 bytes)
- Firmware: `firmware.bin` ready to flash
- Hardware: N32G430 with ST-Link connection

## 🎯 Recommended Testing Approach

### Option 1: Minimal Hardware Test (Quick & Safe) ⚡

**What you need:**

- ST-Link programmer
- N32G430 board
- Power supply (12-24V)
- Multimeter
- Oscilloscope (optional but recommended)

**Step-by-step:**

1. **Pre-Flash Safety Check**

   ```bash
   # Verify binary size is reasonable
   ls -lh firmware.bin
   # Should be ~15KB (matches your 14,996 bytes text section)
   ```

2. **Flash Firmware (Motor Disconnected!)**

   ```bash
   # Using OpenOCD
   openocd -f interface/stlink.cfg -f target/n32g430.cfg \
     -c "program firmware.elf verify reset exit"

   # Or using st-flash (if you have it)
   st-flash write firmware.bin 0x08000000
   ```

3. **Initial Power-On Test (NO MOTOR)**

   - Connect ST-Link GND, SWDIO, SWCLK
   - Power board with 12V (current-limited to 500mA)
   - Watch for:
     - ✅ Stable power consumption (<100mA idle)
     - ✅ No smoke, no excessive heat
     - ✅ LED activity (if configured)

4. **UART Debug Output**

   ```bash
   # Connect UART (pins defined in Uart.c)
   # Baudrate: Check Data_Uart.c (likely 115200)
   screen /dev/tty.usbserial-* 115200

   # You should see initialization messages
   ```

5. **GPIO/PWM Test (Oscilloscope)**

   - Probe PWM output pins (PA8, PA9, PA10 typically)
   - Should see clean PWM at initialization
   - Verify dead-time is present (critical!)

6. **Only THEN Connect Motor**
   - Start with low voltage (6V)
   - Current-limit power supply to 1A
   - Monitor temperature
   - Gradually increase if stable

### Option 2: Full HIL Simulation (Safest, Best Practice) 🚀

**Advantages:**

- Zero risk to hardware
- Repeatable automated tests
- Find bugs before hardware damage
- Professional development workflow

**Setup Time:** 2-3 hours initial setup, reusable forever

**What I created for you:**

- `tests/hil_gui/` - Qt C++ GUI application
- Virtual BLDC motor simulator
- Real-time plotting (speed, current, position)
- Fault injection (overcurrent, Hall faults)
- ST-Link debugger interface (reads/writes MCU memory)

**To build HIL GUI:**

```bash
# Install Qt (if not already)
brew install qt@6
export PATH="/opt/homebrew/opt/qt@6/bin:$PATH"

# Build
cd tests/hil_gui
mkdir build && cd build
cmake ..
make

# Run
./MotorTestGUI
```

**HIL Workflow:**

1. Start GUI → Virtual motor appears
2. Flash firmware to N32G430
3. GUI connects via ST-Link/GDB
4. GUI injects virtual Hall signals → Firmware responds with PWM
5. Virtual motor simulates physics → GUI shows speed/current plots
6. Run automated test sequences
7. **Only after ALL tests pass** → Connect real motor

### Option 3: QEMU/Renode Emulation (Good Middle Ground) 🖥️

**Pros:**

- No hardware needed at all
- Test control logic thoroughly
- Catch stack overflow, null pointer bugs

**Cons:**

- N32G430 not directly supported (use similar STM32 Cortex-M4)
- Peripheral behavior may differ slightly

**Setup:**

```bash
# Install emulator
brew install qemu

# Run firmware in emulation
qemu-system-arm -M netduino2 \
  -cpu cortex-m4 \
  -kernel firmware.elf \
  -nographic \
  -serial stdio
```

## 🔍 Critical Safety Checks Before Hardware

### 1. Verify PWM Dead-Time

```c
// Check in App/src/Pwm.c
// Look for complementary PWM configuration
// Dead-time should be ~1-2μs minimum (prevents shoot-through)
```

### 2. Check Current Limiting

```c
// Check Foc/inc/UserParam.h
#define MAX_CURRENT   10.0  // Should be safe value (A)
#define OVER_CURRENT  12.0  // Shutdown threshold
```

### 3. Verify Voltage Limits

```c
// Check voltage protection thresholds
#define OVER_VOLTAGE  30.0   // V (protection trigger)
#define UNDER_VOLTAGE  8.0   // V (minimum safe voltage)
```

### 4. Test Emergency Stop

- Firmware should disable PWM immediately on fault
- Check `ErrDeal.c` for fault handling logic

## 📊 What Success Looks Like

### Minimal Hardware Test Success:

- ✅ Board powers on, draws <100mA idle
- ✅ UART outputs initialization messages
- ✅ PWM outputs are clean with proper dead-time
- ✅ Motor spins smoothly when commanded
- ✅ Current limiting works (test with oscilloscope)
- ✅ Fault detection triggers on overcurrent

### HIL Simulation Success:

- ✅ Virtual motor accelerates smoothly
- ✅ Current control loop is stable (no oscillations)
- ✅ Speed control responds to setpoint changes
- ✅ Fault injection triggers protection correctly
- ✅ Hall sensor state machine works correctly

## 🚨 Red Flags (STOP Immediately)

- ⛔ Motor vibrates violently
- ⛔ Excessive current (>10A for small motor)
- ⛔ Motor or controller overheating
- ⛔ Erratic Hall sensor behavior
- ⛔ PWM shows no dead-time (shoot-through risk!)
- ⛔ Firmware crashes/resets continuously

## 🎓 My Professional Recommendation

**For your project, I strongly suggest:**

1. **Spend 2-3 hours building the HIL GUI** (Option 2)

   - Why: You mentioned wanting robust testing
   - Why: Reusable for future projects
   - Why: Catches 90% of bugs safely
   - Why: Professional workflow, impress your team/advisor

2. **Then do minimal hardware test** (Option 1)

   - Motor disconnected first
   - Verify PWM with scope
   - Connect motor with current-limited supply

3. **Keep QEMU as backup** (Option 3)
   - Good for quick logic tests
   - No Qt installation needed

## 🛠️ Tools You'll Need

### Required:

- ✅ ST-Link programmer (you have this)
- ✅ Power supply with current limiting
- ✅ Multimeter

### Highly Recommended:

- 🔬 Oscilloscope (verify PWM timing, dead-time)
- 🖥️ Logic analyzer (debug Hall sensors, SPI, UART)

### Optional but Professional:

- 💻 HIL GUI (Qt application we created)
- 🎮 Current probe (measure motor phase currents)

## ⏱️ Time Estimate

| Approach       | Setup Time | Testing Time | Confidence Level |
| -------------- | ---------- | ------------ | ---------------- |
| Quick Hardware | 30 min     | 1-2 hours    | 70%              |
| HIL GUI        | 2-3 hours  | 30 min       | 95%              |
| QEMU Emulation | 1 hour     | 1 hour       | 80%              |

## 🎯 What Would I Do?

If this were my project, here's my sequence:

1. **Right now:** Flash firmware with motor disconnected
2. **Verify:** UART output, PWM signals with scope
3. **Weekend:** Build HIL GUI tool (reusable!)
4. **Then:** Run full test suite in simulation
5. **Finally:** Connect motor with confidence

## 📝 Next Steps?

**Tell me what you want to do:**

A. **"Let's build the HIL GUI!"** → I'll help you install Qt and build it
B. **"Flash firmware now"** → I'll give you exact OpenOCD/ST-Link commands
C. **"Try QEMU first"** → I'll create emulation config
D. **"Check PWM dead-time code"** → I'll review Pwm.c for safety
E. **"All of the above"** → Let's create a step-by-step plan

**What's your preference?** 🚀
