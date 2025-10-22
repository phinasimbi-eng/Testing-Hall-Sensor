# Motor Control HIL Test GUI

Qt-based Hardware-in-Loop testing tool for N32G430 BLDC motor controller.

## Features

- **Virtual Motor Simulation:** Physics-based BLDC motor model
- **Real-time Monitoring:** Speed, current, voltage, position plots
- **ST-Link Integration:** Debug interface via OpenOCD/GDB
- **UART Console:** Monitor firmware debug output
- **Fault Injection:** Test overcurrent, overvoltage, Hall faults
- **Test Automation:** Scripted test sequences

## Build Instructions

### Prerequisites

```bash
# macOS
brew install qt@6
brew install openocd
brew install arm-none-eabi-gdb

# Set up environment
export PATH="/opt/homebrew/opt/qt@6/bin:$PATH"
```

### Build

```bash
cd tests/hil_gui
mkdir build && cd build
cmake ..
make
./MotorTestGUI
```

## Architecture

```
MotorTestGUI/
├── src/
│   ├── main.cpp                 # Application entry
│   ├── mainwindow.cpp/h         # Main GUI window
│   ├── motor_simulator.cpp/h    # Virtual motor physics
│   ├── debugger_interface.cpp/h # GDB/OpenOCD connection
│   ├── uart_monitor.cpp/h       # Serial port monitoring
│   ├── plot_widget.cpp/h        # Real-time plotting
│   └── test_sequence.cpp/h      # Automated test scripts
├── ui/
│   └── mainwindow.ui            # Qt Designer UI
├── resources/
│   └── icons/                   # GUI icons
├── CMakeLists.txt
└── README.md
```

## Usage

1. **Connect ST-Link** to N32G430
2. **Launch GUI:** `./MotorTestGUI`
3. **Start OpenOCD server** (GUI can auto-start)
4. **Load firmware:** Flash via GUI or manually
5. **Run tests:** Select test sequence or manual control

## Test Sequences

- **Smoke Test:** Basic GPIO/UART/ADC verification
- **PWM Test:** Verify all 6 PWM channels
- **Open Loop:** Test SVPWM generation
- **Closed Loop:** Test current/speed controllers
- **Fault Response:** Inject faults, verify protection
- **Performance:** Step response, frequency response

## Virtual Motor Parameters

Matches test motor specs (configure in GUI):

- Pole pairs: 7
- Resistance: 0.5Ω
- Inductance: 0.5mH
- Ke (back-EMF): 0.01 V/rpm
- Inertia: 0.0001 kg⋅m²
