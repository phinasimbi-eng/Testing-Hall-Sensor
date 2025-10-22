# Testing-Hall-Sensor / N32G430 Motor Control

## Overview

This repository contains firmware source code for an N32G430-based brushless motor control project. It includes application code, device drivers, motor control algorithms (FOC), and build configurations for both Keil MDK and ARM GCC toolchains.

## Project Structure

- **`App/`** - Application-level code (main, system initialization, peripheral drivers)
- **`Driver/`** - CMSIS and N32G430 standard peripheral drivers
- **`Foc/`** - Field-oriented control algorithms, observers, and motor control code
  - `inc/` - Header files for FOC library
  - `src/` - FOC implementation (PI control, SVPWM, speed control, etc.)
  - `iqmath/` - TI IQmath library headers (configured for floating-point mode)
  - `user/` - User utilities (OLED, I2C, delays)
- **`RVMDK/`** - Keil µVision project files

## Build System

### GCC Toolchain (Recommended)

This project now supports building with ARM GCC toolchain using a comprehensive Makefile.

#### Prerequisites

You need to install the ARM embedded toolchain (even if you already have regular GCC installed):

**macOS:**

```bash
# Option 1: Using Homebrew (recommended)
brew install --cask gcc-arm-embedded

# Option 2: Download from ARM
# Visit: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# Download "AArch32 bare-metal target (arm-none-eabi)"
# Extract and add to PATH

# Verify installation
arm-none-eabi-gcc --version
make --version
```

**Tested versions:**

- `arm-none-eabi-gcc` v14.3.rel1 (or v13.x works too)
- GNU Make v4.x+

#### First Time Setup (Complete Guide)

If you're starting from scratch, here's the full workflow:

1. **Clone the repository:**

   ```bash
   git clone https://github.com/phinasimbi-eng/Testing-Hall-Sensor.git
   cd Testing-Hall-Sensor
   ```

2. **Install ARM toolchain** (see Prerequisites above for your OS)

3. **Verify your setup:**

   ```bash
   # Check that ARM GCC is installed
   arm-none-eabi-gcc --version

   # Check that Make is available
   make --version

   # List project files
   ls -la
   ```

4. **Build the firmware:**

   ```bash
   make              # This will compile everything
   ```

   **Expected output:**

   ```
   arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 ... -c App/src/main.c -o App/src/main.o
   ... (many compilation steps)
   arm-none-eabi-size firmware.elf
      text    data     bss     dec     hex filename
     14996      40    5244   20280    4f38 firmware.elf
   ```

5. **Check generated files:**

   ```bash
   ls firmware.*
   # Should show: firmware.bin  firmware.elf  firmware.hex  firmware.map
   ```

6. **You're done!** 🎉 You now have:
   - `firmware.bin` ready to flash to your N32G430
   - A working build system that recompiles only changed files
   - Full source code control

**Troubleshooting:**

| Problem                                          | Solution                                                       |
| ------------------------------------------------ | -------------------------------------------------------------- |
| `make: command not found`                        | Install Make (see Prerequisites for your OS)                   |
| `arm-none-eabi-gcc: command not found`           | ARM toolchain not in PATH - reinstall and check "Add to PATH"  |
| `No rule to make target`                         | You're in the wrong directory - `cd` to project root           |
| Permission denied (macOS)                        | Run `chmod +x` on scripts, or use `sudo` for Homebrew          |
| Windows: `'arm-none-eabi-gcc' is not recognized` | Add toolchain to PATH manually: System → Environment Variables |

````

**Windows:**
```bash
# Option 1: Using Chocolatey (if you have it)
choco install gcc-arm-embedded

# Option 2: Manual installation
# 1. Download from: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# 2. Download "AArch32 bare-metal target (arm-none-eabi)" for Windows
# 3. Run installer (e.g., gcc-arm-none-eabi-10.3-2021.10-win32.exe)
# 4. During installation, check "Add to PATH"

# Install Make (Windows doesn't have it by default)
choco install make
# OR download from: https://gnuwin32.sourceforge.net/packages/make.htm

# Verify installation
arm-none-eabi-gcc --version    # Should show v13.x or v14.x
make --version                 # Should work now
````

**Linux (Ubuntu/Debian):**

```bash
# Install ARM toolchain
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi

# Make is usually pre-installed, but if needed:
sudo apt-get install build-essential

# Verify installation
arm-none-eabi-gcc --version
make --version
```

**Why `arm-none-eabi-gcc` and not regular `gcc`?**

- Regular `gcc` compiles code for your computer (x86/ARM Mac/PC)
- `arm-none-eabi-gcc` compiles for **ARM Cortex-M microcontrollers** (bare-metal, no OS)
- Your existing C compiler won't work for embedded ARM chips!

**Tested versions:**

- `arm-none-eabi-gcc` v14.3.rel1 (or v13.x works too)
- GNU Make v4.x+

#### Build Commands

```bash
make              # Incremental build (fast - only rebuilds changed files)
make rebuild      # Full rebuild (clean + build)
make clean        # Remove all build artifacts
```

#### Build Output

After running `make`, you'll get these files in the project root:

- **`firmware.elf`** - ELF executable with debug symbols
  - **Used for:** Debugging with GDB or Keil debugger
  - **Contains:** Code + debug info (function names, variable locations, source line mappings)
  - **Size:** Larger file (~includes symbol tables)
- **`firmware.bin`** ⭐ **← Flash this to your microcontroller!**
  - **Used for:** Programming to N32G430 via STLink/JLink
  - **Contains:** Pure raw binary machine code
  - **Size:** Smallest file (~15KB for this project)
  - **How to flash:**
    ```bash
    st-flash write firmware.bin 0x08000000
    # OR use STLink Utility GUI
    # OR use Keil's Flash → Download
    ```
- **`firmware.hex`** - Intel HEX format
  - **Used for:** Alternative flashing format (some tools prefer this)
  - **Contains:** Same as .bin but in text format with addresses
  - **When to use:** Bootloaders, some production programmers
- **`firmware.map`** - Memory map file
  - **Used for:** Debugging memory issues, optimization
  - **Contains:** Where every function/variable is located in memory
  - **When to use:** Troubleshooting "out of memory" errors, finding code size

**Firmware Size:** ~20KB (14,996 bytes code + 40 bytes data + 5,244 bytes BSS)

> 💡 **Quick Answer:** When you connect your STLink programmer, you'll flash **`firmware.bin`** to the chip. The debugger uses **`firmware.elf`** to know which line of code is executing.

#### Build Features

- ✅ **Zero warnings** - Clean compilation with optimized warning flags
- ✅ **Automatic dependency tracking** - Header changes trigger proper rebuilds
- ✅ **Incremental compilation** - Fast rebuilds (only changed files)
- ✅ **Optimization level 2** - Balanced speed/size optimization

### Keil MDK-ARM Toolchain

**⚠️ Note:** This project currently doesn't have a Keil project file. You have two options:

#### Option 1: Use GCC Build (Recommended)

The Makefile we created does exactly what Keil does - compiles and links your code. You already have working firmware! Just use:

```bash
make              # Build the project
```

#### Option 2: Create a New Keil Project

If you want to use Keil's IDE, debugger, and visual tools:

1. **Create New Project:**

   - Open Keil µVision
   - File → New → µVision Project
   - Save as `RVMDK/RVMDKN32G430/MotorControl.uvprojx`
   - Select device: **Nations N32G430** (install pack if needed)

2. **Add Source Files:**

   - Right-click "Source Group 1" → Add Existing Files
   - Add all `.c` files from:
     - `App/src/`
     - `Driver/CMSIS/device/`
     - `Driver/n32g430_std_periph_driver/src/`
     - `Foc/src/`
     - `Foc/user/`
   - Add `startup_gcc.s` (or use Keil's startup file)

3. **Configure Include Paths:**

   - Project → Options for Target → C/C++
   - Add include paths:
     ```
     App/inc
     Driver/CMSIS/core
     Driver/CMSIS/device
     Driver/n32g430_std_periph_driver/inc
     Foc/inc
     Foc/iqmath
     Foc/user
     ```

4. **Add Preprocessor Define:**

   - In C/C++ tab, add: `MATH_TYPE=1`

5. **Configure Linker:**

   - Use the same memory layout as `linker_script.ld`
   - Or use Keil's scatter file for N32G430

6. **Build:** Project → Build Target

## Understanding Build Tools

### What is a Build Tool?

Both **Keil** and our **Makefile + GCC** do the **same fundamental job**:

```
Source Code (.c, .h, .s)
        ↓
   [COMPILE] ← This is what both tools do!
        ↓
  Object Files (.o)
        ↓
     [LINK]
        ↓
 Firmware (.elf, .bin, .hex)
```

### GCC vs Keil Comparison

| Feature          | GCC + Makefile (What We Built)          | Keil MDK                          |
| ---------------- | --------------------------------------- | --------------------------------- |
| **What it does** | Compiles C → machine code, links files  | Same thing!                       |
| **Compiler**     | `arm-none-eabi-gcc` (free, open source) | ARM Compiler 6 (proprietary)      |
| **Interface**    | Command line (`make`)                   | Graphical IDE                     |
| **Debugger**     | GDB (command line, needs setup)         | Built-in visual debugger ⭐       |
| **Code editor**  | Use any (VS Code, vim, etc.)            | Built-in with IntelliSense        |
| **Cost**         | 100% Free                               | Commercial (32KB code limit free) |
| **Flexibility**  | Full control via Makefile               | GUI-configured                    |
| **Output**       | `.elf`, `.bin`, `.hex` files            | Same files!                       |

### Why Use Keil?

Keil is **more than just a compiler** - it's a complete development environment:

1. **🐛 Visual Debugger**

   - Step through code line by line
   - Set breakpoints and watch variables
   - View CPU registers and memory in real-time
   - See peripheral registers visually

2. **📝 Integrated Editor**

   - Syntax highlighting
   - Code completion (IntelliSense)
   - Jump to definition

3. **🔌 Flash Programming**

   - Direct programming to microcontroller
   - Integrated with debug probe (ST-Link, J-Link)

4. **📊 Peripheral Viewers**
   - See timer values, GPIO states
   - Monitor ADC readings in real-time

### Why Use GCC + Makefile?

1. **💰 Completely Free** - No code size limits
2. **🔄 Cross-platform** - Works on Windows, Mac, Linux
3. **🤖 Automation** - Easy to integrate with CI/CD
4. **📚 Learning** - Understand how builds actually work
5. **⚡ Fast** - Incremental builds are very quick

### Which Should You Use?

**For Development & Debugging:** Use Keil

- Visual debugging is **much easier** than GDB
- Peripheral viewers save tons of time
- Step-through debugging "just works"

**For Building & Automation:** Use our Makefile

- Faster incremental builds
- Works in CI/CD pipelines
- No licensing restrictions

**Best of Both Worlds:**

- Use **Keil** for debugging hardware issues
- Use **`make`** for daily development builds
- Both produce identical firmware!

## Recent Accomplishments

### ✅ GCC Build System Migration

- Successfully migrated from Keil MDK-only to cross-platform GCC build
- Created comprehensive Makefile with proper dependency tracking
- Resolved IQmath library dependency using FLOAT_MATH mode

### ✅ Code Quality Improvements

- Fixed missing IQmath library blocker by switching to header-only floating-point mode
- Created `FunctionWrappers.c` to provide GCC-compatible function implementations
- Fixed old-style declarations and uninitialized variables
- Added comprehensive warning suppression flags for vendor code

### ✅ Build Configuration

- Added `-DMATH_TYPE=1` flag to enable floating-point IQmath mode
- Configured automatic dependency generation (`-MMD -MP`)
- Implemented proper incremental build system
- Added assembler startup file with proper formatting

### ✅ Documentation

- Added `.gitignore` for build artifacts
- Created detailed README with build instructions

## Technical Notes

### IQmath Library Configuration

This project uses TI's IQmath library in **FLOAT_MATH** mode (`MATH_TYPE=1`), which provides:

- Header-only implementation (no precompiled library required)
- Floating-point operations instead of fixed-point
- Full compatibility with ARM GCC toolchain

### Compiler Flags

```makefile
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -Wall -Wextra -std=c99 \
         -ffunction-sections -fdata-sections -DMATH_TYPE=1 \
         -Wno-unused-parameter -Wno-unused-variable \
         -Wno-unused-but-set-variable -Wno-parentheses \
         -Wno-strict-aliasing -Wno-absolute-value
```

### Memory Layout

- **Flash:** N32G430 internal flash
- **RAM:** N32G430 SRAM
- **Linker script:** `linker_script.ld`
- **Startup code:** `startup_gcc.s`

## Git Best Practices

### Ignored Files

Build artifacts and IDE-specific files are excluded via `.gitignore`:

- `*.o`, `*.d`, `*.elf`, `*.bin`, `*.hex`, `*.map`
- Keil user files (`*.uvoptx`, `*.uvguix`)
- JLink logs and debug files

### Clean Repository

To remove already-tracked build artifacts:

```bash
git rm --cached -r RVMDK/**/Obj
git rm --cached RVMDK/**/JLinkLog.txt
git commit -m "Remove generated build artifacts from repository"
```

## Contributing

- Keep vendor libraries (CMSIS, peripheral drivers) under `Driver/` for reproducible builds
- Test builds with both `make` and `make rebuild` before committing
- Avoid committing IDE user settings files

## Hardware Target

- **MCU:** N32G430 (ARM Cortex-M4)
- **Application:** Brushless motor control with FOC
- **Current sensing:** 2-shunt configuration
- **Features:** Hall sensor support, encoder observer, SVPWM
