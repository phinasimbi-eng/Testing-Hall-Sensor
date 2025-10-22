#include "debugger_interface.h"

DebuggerInterface::DebuggerInterface(QObject *parent)
    : QObject(parent)
{
}

DebuggerInterface::~DebuggerInterface()
{
    disconnect();
}

bool DebuggerInterface::connect(const QString &device)
{
    Q_UNUSED(device);
    // TODO: Implement OpenOCD connection
    // For now, return false (simulation only mode)
    return false;
}

void DebuggerInterface::disconnect()
{
    if (m_openocdProcess && m_openocdProcess->state() == QProcess::Running)
    {
        m_openocdProcess->terminate();
        m_openocdProcess->waitForFinished();
    }
    if (m_gdbProcess && m_gdbProcess->state() == QProcess::Running)
    {
        m_gdbProcess->terminate();
        m_gdbProcess->waitForFinished();
    }
    m_connected = false;
    emit disconnected();
}

bool DebuggerInterface::readMemory(uint32_t address, uint8_t *data, size_t length)
{
    Q_UNUSED(address);
    Q_UNUSED(data);
    Q_UNUSED(length);
    // TODO: Implement GDB memory read
    return false;
}

bool DebuggerInterface::writeMemory(uint32_t address, const uint8_t *data, size_t length)
{
    Q_UNUSED(address);
    Q_UNUSED(data);
    Q_UNUSED(length);
    // TODO: Implement GDB memory write
    return false;
}

uint32_t DebuggerInterface::readRegister(const QString &regName)
{
    Q_UNUSED(regName);
    // TODO: Implement GDB register read
    return 0;
}

bool DebuggerInterface::writeRegister(const QString &regName, uint32_t value)
{
    Q_UNUSED(regName);
    Q_UNUSED(value);
    // TODO: Implement GDB register write
    return false;
}

DebuggerInterface::MotorState DebuggerInterface::readMotorState()
{
    // TODO: Read actual motor control variables from MCU memory
    return MotorState();
}

bool DebuggerInterface::writeSpeedTarget(float rpm)
{
    Q_UNUSED(rpm);
    // TODO: Write speed target to MCU memory
    return false;
}

bool DebuggerInterface::injectHallSensor(uint8_t state)
{
    Q_UNUSED(state);
    // TODO: Override Hall sensor GPIO inputs
    return false;
}
