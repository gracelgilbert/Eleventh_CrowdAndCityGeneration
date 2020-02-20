#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "simulationdriver.h"
#include <QGraphicsScene>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    SimulationDriver driver;
    ~MainWindow();
    void DisplayQImage(QImage &i);

private:
    Ui::MainWindow *ui;
    QGraphicsScene graphics_scene;
};

#endif // MAINWINDOW_H
