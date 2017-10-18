#include "agrigatemainwindow.h"
#include "ui_agrigatemainwindow.h"

AgriGateMainWindow::AgriGateMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AgriGateMainWindow)
{
    ui->setupUi(this);
}

AgriGateMainWindow::~AgriGateMainWindow()
{
    delete ui;
}
