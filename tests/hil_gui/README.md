# Motor Control Virtual Test Simulator

Qt-based **pure software simulation** tool for N32G430 BLDC motor controller.

**Note:** This is a development/testing tool that runs entirely in software on this Mac.
Hardware testing with ST-Link will be done on a separate Windows PC with Keil.

## Features

- **Virtual Motor Simulation:** Physics-based BLDC motor model (no hardware needed!)
- **Real-time Monitoring:** Speed, current, voltage, position plots
- **Control Logic Testing:** Simulate FOC algorithms, PID loops, state machines
- **Fault Injection:** Test overcurrent, overvoltage, Hall faults safely
- **Test Automation:** Scripted test sequences
- **Export Results:** Generate test reports for hardware validation

## Build Instructions

### Prerequisites

```bash
# macOS
brew install qt@6

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

1. **Launch GUI:** `./MotorTestGUI`
2. **Configure virtual motor parameters** (pole pairs, resistance, inductance, etc.)
3. **Load control parameters** from your firmware code (paste UserParam.h values)
4. **Run test sequences:**
   - Open Loop Test: Verify SVPWM generation
   - Closed Loop Test: Verify current/speed controllers
   - Fault Injection: Test protection logic
   - Performance Test: Step response, frequency response
5. **Export results** for documentation
6. **Transfer validated firmware to Windows PC** for hardware testing with Keil + ST-Link

## Test Sequences

- **Smoke Test:** Parameter validation, initialization checks
- **Open Loop:** Test SVPWM generation with virtual Hall sensors
- **Closed Loop:** Test current/speed controllers with virtual motor physics
- **Fault Response:** Inject faults, verify software protection triggers
- **Performance:** Step response, overshoot, settling time analysis

## Virtual Motor Parameters

Matches test motor specs (configure in GUI):

- Pole pairs: 7
- Resistance: 0.5Ω
- Inductance: 0.5mH
- Ke (back-EMF): 0.01 V/rpm
- Inertia: 0.0001 kg⋅m²
