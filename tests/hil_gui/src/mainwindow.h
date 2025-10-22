#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QChartView>
#include <QLineSeries>
#include "motor_simulator.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onMotorStateUpdated(const MotorSimulator::State &state);
    void onHallChanged(const MotorSimulator::HallState &hall);
    void onStartSimulation();
    void onStopSimulation();
    void onResetSimulation();
    void onInjectFault();

private:
    void setupUI();
    void setupCharts();
    void updateCharts();

    MotorSimulator *m_simulator;

    // Real-time charts
    QChart *m_speedChart;
    QChart *m_currentChart;
    QLineSeries *m_speedSeries;
    QLineSeries *m_iaSeries;
    QLineSeries *m_ibSeries;
    QLineSeries *m_icSeries;

    QVector<QPointF> m_speedData;
    QVector<QPointF> m_iaData;
    QVector<QPointF> m_ibData;
    QVector<QPointF> m_icData;
    double m_time = 0.0;
};

#endif // MAINWINDOW_H
