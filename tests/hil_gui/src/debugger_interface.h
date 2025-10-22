#ifndef DEBUGGER_INTERFACE_H
#define DEBUGGER_INTERFACE_H

#include <QObject>
#include <QProcess>

/**
 * @brief Interface to ST-Link debugger via OpenOCD/GDB
 *
 * Connects to the N32G430 MCU and reads/writes memory in real-time
 */
class DebuggerInterface : public QObject
{
    Q_OBJECT

public:
    explicit DebuggerInterface(QObject *parent = nullptr);
    ~DebuggerInterface() override;

    // Connection management
    bool connect(const QString &device = "stlink");
    void disconnect();
    bool isConnected() const { return m_connected; }

    // Memory access
    bool readMemory(uint32_t address, uint8_t *data, size_t length);
    bool writeMemory(uint32_t address, const uint8_t *data, size_t length);

    // Register access
    uint32_t readRegister(const QString &regName);
    bool writeRegister(const QString &regName, uint32_t value);

    // Convenience methods for motor control variables
    struct MotorState
    {
        float speedTarget;
        float speedActual;
        float currentId;
        float currentIq;
        uint16_t pwmDutyU;
        uint16_t pwmDutyV;
        uint16_t pwmDutyW;
        uint8_t hallState;
        uint32_t faultFlags;
    };

    MotorState readMotorState();
    bool writeSpeedTarget(float rpm);
    bool injectHallSensor(uint8_t state);

signals:
    void connected();
    void disconnected();
    void errorOccurred(const QString &error);

private:
    bool m_connected = false;
    QProcess *m_openocdProcess = nullptr;
    QProcess *m_gdbProcess = nullptr;

    // Memory map (from linker script)
    static constexpr uint32_t RAM_BASE = 0x20000000;
    static constexpr uint32_t FLASH_BASE = 0x08000000;
};

#endif // DEBUGGER_INTERFACE_H
