#ifndef MAINQSCRIPTWINDOW_H
#define MAINQSCRIPTWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainQScriptWindow;
}

class MainQScriptWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainQScriptWindow(QWidget *parent = 0);
    ~MainQScriptWindow();

public slots:
    void executeScript();
private:
    Ui::MainQScriptWindow *ui;
};

#endif // MAINQSCRIPTWINDOW_H
