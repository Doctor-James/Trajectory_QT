#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;
    lcm.subscribe("armor_msg", &MainWindow::handleMessage, &w);
    w.get_lcm(&lcm);
    w.show();
    return a.exec();
}
