#ifndef PLOT_WIDGET_H
#define PLOT_WIDGET_H

#include <QWidget>

// Placeholder for custom plotting widget
// For now, we're using QChart which is sufficient

class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(QWidget *parent = nullptr);
};

#endif // PLOT_WIDGET_H
