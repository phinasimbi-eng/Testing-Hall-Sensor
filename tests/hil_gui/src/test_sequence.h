#ifndef TEST_SEQUENCE_H
#define TEST_SEQUENCE_H

#include <QObject>

class TestSequence : public QObject
{
    Q_OBJECT

public:
    explicit TestSequence(QObject *parent = nullptr);

    void runSmokeTest();
    void runPWMTest();
    void runOpenLoopTest();
    void runClosedLoopTest();
    void runFaultResponseTest();

signals:
    void testStarted(const QString &testName);
    void testProgress(int percentage);
    void testCompleted(const QString &testName, bool passed);
};

#endif // TEST_SEQUENCE_H
