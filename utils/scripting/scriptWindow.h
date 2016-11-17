#ifndef MAINQSCRIPTWINDOW_H
#define MAINQSCRIPTWINDOW_H

#include <QMainWindow>
#include <QScriptEngine>

namespace Ui {
class ScriptWindow;
}

class ScriptWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ScriptWindow(QWidget *parent = 0);
    ~ScriptWindow();

    QScriptEngine *engine = NULL;


public slots:
    QString checkScript();
    void executeScript();
private:
    Ui::ScriptWindow *ui;
};

#endif // MAINQSCRIPTWINDOW_H
