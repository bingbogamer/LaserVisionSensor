#include "LaserVisionSensor.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    LaserVisionSensor w;
    w.show();
    return a.exec();
}
