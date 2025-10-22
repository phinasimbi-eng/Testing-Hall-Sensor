#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    app.setApplicationName("Motor Control HIL Test");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("Eunice Sensor Project");

    MainWindow window;
    window.show();

    return app.exec();
}
