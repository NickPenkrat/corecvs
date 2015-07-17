#ifndef CAMERAMODELPARAMETERSCONTROLWIDGET_H
#define CAMERAMODELPARAMETERSCONTROLWIDGET_H

#include <QWidget>
#include "lensDistortionModelParameters.h"

namespace Ui {
class CameraModelParametersControlWidget;
}

class CameraModelParametersControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraModelParametersControlWidget(QWidget *parent = 0);
    ~CameraModelParametersControlWidget();

    LensDistortionModelParameters lensDistortionParameters();

private:
    Ui::CameraModelParametersControlWidget *ui;
};

#endif // CAMERAMODELPARAMETERSCONTROLWIDGET_H
