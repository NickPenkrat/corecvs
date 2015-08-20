#ifndef CAMERAMODELPARAMETERSCONTROLWIDGET_H
#define CAMERAMODELPARAMETERSCONTROLWIDGET_H

#include <QWidget>
#include "parametersControlWidgetBase.h"
#include "lensDistortionModelParameters.h"

namespace Ui {
class CameraModelParametersControlWidget;
}

class CameraModelParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit CameraModelParametersControlWidget(QWidget *parent = 0);
    ~CameraModelParametersControlWidget();

    LensDistortionModelParameters lensDistortionParameters();
    void setLensDistortionParameters(const LensDistortionModelParameters &params);

    virtual void loadParamWidget(WidgetLoader &/*loader*/);
    virtual void saveParamWidget(WidgetSaver  &/*saver*/ );

private:
    Ui::CameraModelParametersControlWidget *ui;
};

#endif // CAMERAMODELPARAMETERSCONTROLWIDGET_H
