#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QChartView>
#include <QValueAxis>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_simulator(new MotorSimulator(this)), m_time(0.0)
{
    setupUI();
    setupCharts();

    // Connect simulator signals
    connect(m_simulator, &MotorSimulator::stateUpdated,
            this, &MainWindow::onMotorStateUpdated);
    connect(m_simulator, &MotorSimulator::hallChanged,
            this, &MainWindow::onHallChanged);

    setWindowTitle("Motor Control HIL Test System");
    resize(1200, 800);
}

MainWindow::~MainWindow()
{
    m_simulator->stop();
}

void MainWindow::setupUI()
{
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // Control Panel
    QGroupBox *controlGroup = new QGroupBox("Simulation Control");
    QHBoxLayout *controlLayout = new QHBoxLayout(controlGroup);

    QPushButton *startBtn = new QPushButton("▶ Start");
    QPushButton *stopBtn = new QPushButton("⏸ Stop");
    QPushButton *resetBtn = new QPushButton("↻ Reset");
    QPushButton *faultBtn = new QPushButton("⚠ Inject Fault");

    connect(startBtn, &QPushButton::clicked, this, &MainWindow::onStartSimulation);
    connect(stopBtn, &QPushButton::clicked, this, &MainWindow::onStopSimulation);
    connect(resetBtn, &QPushButton::clicked, this, &MainWindow::onResetSimulation);
    connect(faultBtn, &QPushButton::clicked, this, &MainWindow::onInjectFault);

    controlLayout->addWidget(startBtn);
    controlLayout->addWidget(stopBtn);
    controlLayout->addWidget(resetBtn);
    controlLayout->addWidget(faultBtn);
    controlLayout->addStretch();

    // Motor Parameters
    QGroupBox *paramGroup = new QGroupBox("Motor Parameters");
    QHBoxLayout *paramLayout = new QHBoxLayout(paramGroup);

    QLabel *polePairsLabel = new QLabel("Pole Pairs:");
    QSpinBox *polePairsSpin = new QSpinBox();
    polePairsSpin->setRange(1, 20);
    polePairsSpin->setValue(7);

    QLabel *voltageLabel = new QLabel("Bus Voltage (V):");
    QDoubleSpinBox *voltageSpin = new QDoubleSpinBox();
    voltageSpin->setRange(0, 48);
    voltageSpin->setValue(24.0);
    voltageSpin->setSuffix(" V");

    QLabel *loadLabel = new QLabel("Load Torque (Nm):");
    QDoubleSpinBox *loadSpin = new QDoubleSpinBox();
    loadSpin->setRange(0, 1.0);
    loadSpin->setValue(0.0);
    loadSpin->setSuffix(" Nm");
    loadSpin->setDecimals(3);

    connect(voltageSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_simulator, &MotorSimulator::setBusVoltage);
    connect(loadSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_simulator, &MotorSimulator::setLoadTorque);

    paramLayout->addWidget(polePairsLabel);
    paramLayout->addWidget(polePairsSpin);
    paramLayout->addWidget(voltageLabel);
    paramLayout->addWidget(voltageSpin);
    paramLayout->addWidget(loadLabel);
    paramLayout->addWidget(loadSpin);
    paramLayout->addStretch();

    // Tab widget for charts and logs
    QTabWidget *tabWidget = new QTabWidget();

    // Speed chart tab
    QChartView *speedChartView = new QChartView(m_speedChart);
    speedChartView->setRenderHint(QPainter::Antialiasing);
    tabWidget->addTab(speedChartView, "Speed");

    // Current chart tab
    QChartView *currentChartView = new QChartView(m_currentChart);
    currentChartView->setRenderHint(QPainter::Antialiasing);
    tabWidget->addTab(currentChartView, "Current");

    // Log tab
    QTextEdit *logView = new QTextEdit();
    logView->setReadOnly(true);
    logView->append("=== Motor Control HIL Test System ===");
    logView->append("Ready to start simulation...\n");
    tabWidget->addTab(logView, "Log");

    // Add everything to main layout
    mainLayout->addWidget(controlGroup);
    mainLayout->addWidget(paramGroup);
    mainLayout->addWidget(tabWidget, 1); // Stretch factor 1

    setCentralWidget(centralWidget);
}

void MainWindow::setupCharts()
{
    // Speed Chart
    m_speedChart = new QChart();
    m_speedChart->setTitle("Motor Speed");
    m_speedChart->setAnimationOptions(QChart::NoAnimation);

    m_speedSeries = new QLineSeries();
    m_speedSeries->setName("Speed (RPM)");
    m_speedChart->addSeries(m_speedSeries);

    QValueAxis *speedAxisX = new QValueAxis();
    speedAxisX->setTitleText("Time (s)");
    speedAxisX->setRange(0, 10);
    m_speedChart->addAxis(speedAxisX, Qt::AlignBottom);
    m_speedSeries->attachAxis(speedAxisX);

    QValueAxis *speedAxisY = new QValueAxis();
    speedAxisY->setTitleText("Speed (RPM)");
    speedAxisY->setRange(0, 5000);
    m_speedChart->addAxis(speedAxisY, Qt::AlignLeft);
    m_speedSeries->attachAxis(speedAxisY);

    // Current Chart
    m_currentChart = new QChart();
    m_currentChart->setTitle("Phase Currents");
    m_currentChart->setAnimationOptions(QChart::NoAnimation);

    m_iaSeries = new QLineSeries();
    m_iaSeries->setName("Ia");
    m_ibSeries = new QLineSeries();
    m_ibSeries->setName("Ib");
    m_icSeries = new QLineSeries();
    m_icSeries->setName("Ic");

    m_currentChart->addSeries(m_iaSeries);
    m_currentChart->addSeries(m_ibSeries);
    m_currentChart->addSeries(m_icSeries);

    QValueAxis *currentAxisX = new QValueAxis();
    currentAxisX->setTitleText("Time (s)");
    currentAxisX->setRange(0, 10);
    m_currentChart->addAxis(currentAxisX, Qt::AlignBottom);
    m_iaSeries->attachAxis(currentAxisX);
    m_ibSeries->attachAxis(currentAxisX);
    m_icSeries->attachAxis(currentAxisX);

    QValueAxis *currentAxisY = new QValueAxis();
    currentAxisY->setTitleText("Current (A)");
    currentAxisY->setRange(-10, 10);
    m_currentChart->addAxis(currentAxisY, Qt::AlignLeft);
    m_iaSeries->attachAxis(currentAxisY);
    m_ibSeries->attachAxis(currentAxisY);
    m_icSeries->attachAxis(currentAxisY);
}

void MainWindow::onMotorStateUpdated(const MotorSimulator::State &state)
{
    m_time += 0.001; // 1ms update rate

    // Convert rad/s to RPM
    double speedRPM = state.velocity * 60.0 / (2.0 * M_PI);

    // Update data
    m_speedData.append(QPointF(m_time, speedRPM));
    m_iaData.append(QPointF(m_time, state.ia));
    m_ibData.append(QPointF(m_time, state.ib));
    m_icData.append(QPointF(m_time, state.ic));

    // Keep only last 10 seconds
    if (m_speedData.size() > 10000)
    {
        m_speedData.removeFirst();
        m_iaData.removeFirst();
        m_ibData.removeFirst();
        m_icData.removeFirst();
    }

    // Update charts every 100ms
    static int updateCounter = 0;
    if (++updateCounter >= 100)
    {
        updateCounter = 0;
        updateCharts();
    }
}

void MainWindow::onHallChanged(const MotorSimulator::HallState &hall)
{
    // Update Hall sensor indicators
    // TODO: Add Hall sensor status display
    Q_UNUSED(hall);
}

void MainWindow::updateCharts()
{
    m_speedSeries->replace(m_speedData);
    m_iaSeries->replace(m_iaData);
    m_ibSeries->replace(m_ibData);
    m_icSeries->replace(m_icData);

    // Auto-scale X axis
    if (m_time > 10)
    {
        QValueAxis *speedAxisX = qobject_cast<QValueAxis *>(m_speedChart->axes(Qt::Horizontal).first());
        if (speedAxisX)
        {
            speedAxisX->setRange(m_time - 10, m_time);
        }

        QValueAxis *currentAxisX = qobject_cast<QValueAxis *>(m_currentChart->axes(Qt::Horizontal).first());
        if (currentAxisX)
        {
            currentAxisX->setRange(m_time - 10, m_time);
        }
    }
}

void MainWindow::onStartSimulation()
{
    m_simulator->start();
    // TODO: Connect to hardware via debugger interface
}

void MainWindow::onStopSimulation()
{
    m_simulator->stop();
}

void MainWindow::onResetSimulation()
{
    m_simulator->reset();
    m_speedData.clear();
    m_iaData.clear();
    m_ibData.clear();
    m_icData.clear();
    m_time = 0.0;
    updateCharts();
}

void MainWindow::onInjectFault()
{
    // Inject Hall sensor fault for 500ms
    m_simulator->injectHallFault(500);
}
