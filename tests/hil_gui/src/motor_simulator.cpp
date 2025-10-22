#include "motor_simulator.h"
#include <QDateTime>
#include <QtMath>

MotorSimulator::MotorSimulator(QObject *parent)
    : QObject(parent), m_simTimer(new QTimer(this)), m_faultTimer(new QTimer(this))
{
    connect(m_simTimer, &QTimer::timeout, this, &MotorSimulator::updateSimulation);
    m_simTimer->setInterval(1); // 1ms = 1kHz simulation rate

    m_faultTimer->setSingleShot(true);
    connect(m_faultTimer, &QTimer::timeout, this, [this]()
            { m_hallFaultActive = false; });

    reset();
}

MotorSimulator::~MotorSimulator()
{
    stop();
}

void MotorSimulator::setParameters(const Parameters &params)
{
    m_params = params;
}

void MotorSimulator::setPWM(double dutyCycleA, double dutyCycleB, double dutyCycleC)
{
    m_pwmA = qBound(0.0, dutyCycleA, 1.0);
    m_pwmB = qBound(0.0, dutyCycleB, 1.0);
    m_pwmC = qBound(0.0, dutyCycleC, 1.0);
}

void MotorSimulator::setBusVoltage(double voltage)
{
    m_state.vbus = voltage;
}

void MotorSimulator::setLoadTorque(double torque)
{
    m_params.loadTorque = torque;
}

void MotorSimulator::start()
{
    m_lastUpdateTime = QDateTime::currentMSecsSinceEpoch();
    m_simTimer->start();
}

void MotorSimulator::stop()
{
    m_simTimer->stop();
}

void MotorSimulator::reset()
{
    m_state = State();
    m_state.vbus = 24.0;
    m_pwmA = m_pwmB = m_pwmC = 0.0;
    updateHallSensors();
}

void MotorSimulator::setSimulationSpeed(double speed)
{
    m_simulationSpeed = qMax(0.1, speed);
}

void MotorSimulator::injectHallFault(int duration_ms)
{
    m_hallFaultActive = true;
    m_faultTimer->start(duration_ms);
    emit faultOccurred("Hall Sensor Fault Injected");
}

void MotorSimulator::injectOvercurrent(double current_A)
{
    m_state.ia = current_A;
    emit faultOccurred("Overcurrent Fault Injected");
}

void MotorSimulator::updateSimulation()
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    double dt = (currentTime - m_lastUpdateTime) * 0.001 * m_simulationSpeed; // Convert to seconds
    m_lastUpdateTime = currentTime;

    if (dt > 0.0 && dt < 0.1)
    { // Sanity check
        calculateMotorPhysics(dt);
        updateHallSensors();
        emit stateUpdated(m_state);
    }
}

void MotorSimulator::calculateMotorPhysics(double dt)
{
    // Calculate phase voltages from PWM
    double va = m_pwmA * m_state.vbus;
    double vb = m_pwmB * m_state.vbus;
    double vc = m_pwmC * m_state.vbus;

    // Calculate back-EMF
    double backEmfA = m_params.ke * m_state.velocity * qSin(m_state.position);
    double backEmfB = m_params.ke * m_state.velocity * qSin(m_state.position - 2.0 * M_PI / 3.0);
    double backEmfC = m_params.ke * m_state.velocity * qSin(m_state.position + 2.0 * M_PI / 3.0);

    // Simple current model (RL circuit with back-EMF)
    double tau = m_params.inductance / m_params.resistance;
    double alpha = dt / tau;

    m_state.ia += alpha * ((va - backEmfA) / m_params.resistance - m_state.ia);
    m_state.ib += alpha * ((vb - backEmfB) / m_params.resistance - m_state.ib);
    m_state.ic += alpha * ((vc - backEmfC) / m_params.resistance - m_state.ic);

    // Calculate electromagnetic torque (simplified Park transform)
    double torque = m_params.kt * (m_state.ia * qSin(m_state.position) +
                                   m_state.ib * qSin(m_state.position - 2.0 * M_PI / 3.0) +
                                   m_state.ic * qSin(m_state.position + 2.0 * M_PI / 3.0));

    // Mechanical equation: J*dω/dt = T_em - T_load - B*ω
    double acceleration = (torque - m_params.loadTorque - m_params.friction * m_state.velocity) / m_params.inertia;
    m_state.velocity += acceleration * dt;

    // Update electrical position (electrical = mechanical * pole_pairs)
    m_state.position += m_state.velocity * m_params.polePairs * dt;

    // Wrap position to [0, 2π]
    while (m_state.position >= 2.0 * M_PI)
        m_state.position -= 2.0 * M_PI;
    while (m_state.position < 0.0)
        m_state.position += 2.0 * M_PI;

    // Simple temperature model (increases with current²)
    double powerLoss = m_params.resistance * (m_state.ia * m_state.ia + m_state.ib * m_state.ib + m_state.ic * m_state.ic);
    m_state.temperature += (powerLoss * 0.1 - (m_state.temperature - 25.0) * 0.01) * dt;
}

void MotorSimulator::updateHallSensors()
{
    if (m_hallFaultActive)
    {
        // Randomize Hall signals during fault
        m_hallState.hallA = false;
        m_hallState.hallB = false;
        m_hallState.hallC = false;
        m_hallState.sector = 0xFF; // Invalid
        emit hallChanged(m_hallState);
        return;
    }

    m_hallState.sector = calculateSector(m_state.position);

    // 6-step Hall sensor truth table
    switch (m_hallState.sector)
    {
    case 0:
        m_hallState.hallA = false;
        m_hallState.hallB = false;
        m_hallState.hallC = true;
        break; // 001
    case 1:
        m_hallState.hallA = false;
        m_hallState.hallB = true;
        m_hallState.hallC = true;
        break; // 011
    case 2:
        m_hallState.hallA = false;
        m_hallState.hallB = true;
        m_hallState.hallC = false;
        break; // 010
    case 3:
        m_hallState.hallA = true;
        m_hallState.hallB = true;
        m_hallState.hallC = false;
        break; // 110
    case 4:
        m_hallState.hallA = true;
        m_hallState.hallB = false;
        m_hallState.hallC = false;
        break; // 100
    case 5:
        m_hallState.hallA = true;
        m_hallState.hallB = false;
        m_hallState.hallC = true;
        break; // 101
    default:
        break;
    }

    emit hallChanged(m_hallState);
}

uint8_t MotorSimulator::calculateSector(double position)
{
    // Normalize position to [0, 2π]
    while (position >= 2.0 * M_PI)
        position -= 2.0 * M_PI;
    while (position < 0.0)
        position += 2.0 * M_PI;

    // Divide full electrical rotation into 6 sectors (60° each)
    return static_cast<uint8_t>(position / (M_PI / 3.0)) % 6;
}
