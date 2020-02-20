#include "mainwindow.h"
#include <QApplication>
#include "simulationdriver.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
//    SimulationDriver driver = SimulationDriver();
//    driver.run(1);
    w.show();

    return a.exec();
}
