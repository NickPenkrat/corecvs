#ifndef SHADEDSCENECONTROLWIDGET_H
#define SHADEDSCENECONTROLWIDGET_H

#include <QWidget>

namespace Ui {
class ShadedSceneControlWidget;
}

class ShadedSceneControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ShadedSceneControlWidget(QWidget *parent = 0);
    ~ShadedSceneControlWidget();

private:
    Ui::ShadedSceneControlWidget *ui;
};

#endif // SHADEDSCENECONTROLWIDGET_H
