#ifndef AGRIGATEMAINWINDOW_H
#define AGRIGATEMAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class AgriGateMainWindow;
}

class AgriGateMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit AgriGateMainWindow(QWidget *parent = 0);
    ~AgriGateMainWindow();

private:
    Ui::AgriGateMainWindow *ui;
};

#endif // AGRIGATEMAINWINDOW_H
