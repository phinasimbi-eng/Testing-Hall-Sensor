#include "uart_monitor.h"

UartMonitor::UartMonitor(QObject *parent)
    : QObject(parent), m_serialPort(new QSerialPort(this))
{
    connect(m_serialPort, &QSerialPort::readyRead,
            this, &UartMonitor::handleReadyRead);
    connect(m_serialPort, &QSerialPort::errorOccurred,
            this, &UartMonitor::handleError);
}

UartMonitor::~UartMonitor()
{
    close();
}

bool UartMonitor::open(const QString &portName, qint32 baudRate)
{
    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(baudRate);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    return m_serialPort->open(QIODevice::ReadOnly);
}

void UartMonitor::close()
{
    if (m_serialPort->isOpen())
    {
        m_serialPort->close();
    }
}

bool UartMonitor::isOpen() const
{
    return m_serialPort->isOpen();
}

void UartMonitor::handleReadyRead()
{
    QByteArray data = m_serialPort->readAll();
    emit dataReceived(QString::fromUtf8(data));
}

void UartMonitor::handleError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError)
    {
        emit errorOccurred(m_serialPort->errorString());
    }
}
