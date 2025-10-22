#ifndef MOTOR_SIMULATOR_H
#define MOTOR_SIMULATOR_H

#include <QObject>
#include <QTimer>
#include <cmath>

/**
 * @brief Virtual BLDC motor physics simulator
 *
 * Simulates a 3-phase BLDC motor response to PWM commands.
 * Generates virtual Hall sensor signals and back-EMF.
 */
class MotorSimulator : public QObject
{
    Q_OBJECT

public:
    struct Parameters
    {
        double polePairs = 7.0;     // Number of pole pairs
        double resistance = 0.5;    // Phase resistance (Ohms)
        double inductance = 0.0005; // Phase inductance (H)
        double ke = 0.01;           // Back-EMF constant (V/rpm)
        double kt = 0.01;           // Torque constant (Nm/A)
        double inertia = 0.0001;    // Rotor inertia (kg⋅m²)
        double friction = 0.00001;  // Friction coefficient
        double loadTorque = 0.0;    // External load torque (Nm)
    };

    struct State
    {
        double position = 0.0;     // Electrical position (rad)
        double velocity = 0.0;     // Mechanical speed (rad/s)
        double ia = 0.0;           // Phase A current (A)
        double ib = 0.0;           // Phase B current (A)
        double ic = 0.0;           // Phase C current (A)
        double vbus = 24.0;        // Bus voltage (V)
        double temperature = 25.0; // Motor temperature (°C)
    };

    struct HallState
    {
        bool hallA = false;
        bool hallB = false;
        bool hallC = false;
        uint8_t sector = 0; // 0-5 for 6-step commutation
    };

    explicit MotorSimulator(QObject *parent = nullptr);
    ~MotorSimulator() override;

    void setParameters(const Parameters &params);
    Parameters getParameters() const { return m_params; }

    State getState() const { return m_state; }
    HallState getHallState() const { return m_hallState; }

    // Control inputs (from firmware)
    void setPWM(double dutyCycleA, double dutyCycleB, double dutyCycleC);
    void setBusVoltage(double voltage);
    void setLoadTorque(double torque);

    // Simulation control
    void start();
    void stop();
    void reset();
    void setSimulationSpeed(double speed); // 1.0 = real-time

    // Fault injection
    void injectHallFault(int duration_ms);
    void injectOvercurrent(double current_A);

signals:
    void stateUpdated(const State &state);
    void hallChanged(const HallState &hall);
    void faultOccurred(const QString &faultType);

private slots:
    void updateSimulation();

private:
    void calculateMotorPhysics(double dt);
    void updateHallSensors();
    void calculateBackEMF();
    void calculateCurrents(double dt);
    uint8_t calculateSector(double position);

    Parameters m_params;
    State m_state;
    HallState m_hallState;

    // Control inputs
    double m_pwmA = 0.0;
    double m_pwmB = 0.0;
    double m_pwmC = 0.0;

    // Simulation
    QTimer *m_simTimer;
    double m_simulationSpeed = 1.0;
    qint64 m_lastUpdateTime = 0;

    // Fault injection
    bool m_hallFaultActive = false;
    QTimer *m_faultTimer;
};

#endif // MOTOR_SIMULATOR_H
