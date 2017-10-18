#include "agrigatemainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    AgriGateMainWindow w;
    w.show();

    return a.exec();
}
