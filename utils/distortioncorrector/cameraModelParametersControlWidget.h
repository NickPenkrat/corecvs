#ifndef CAMERAMODELPARAMETERSCONTROLWIDGET_H
#define CAMERAMODELPARAMETERSCONTROLWIDGET_H

#include <QWidget>

namespace Ui {
class CameraModelParametersControlWidget;
}

class CameraModelParametersControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraModelParametersControlWidget(QWidget *parent = 0);
    ~CameraModelParametersControlWidget();

private:
    Ui::CameraModelParametersControlWidget *ui;
};

#endif // CAMERAMODELPARAMETERSCONTROLWIDGET_H
