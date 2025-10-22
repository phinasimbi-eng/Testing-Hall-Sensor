#ifndef UART_MONITOR_H
#define UART_MONITOR_H

#include <QObject>
#include <QSerialPort>

class UartMonitor : public QObject
{
    Q_OBJECT

public:
    explicit UartMonitor(QObject *parent = nullptr);
    ~UartMonitor() override;

    bool open(const QString &portName, qint32 baudRate = 115200);
    void close();
    bool isOpen() const;

signals:
    void dataReceived(const QString &data);
    void errorOccurred(const QString &error);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *m_serialPort;
};

#endif // UART_MONITOR_H
