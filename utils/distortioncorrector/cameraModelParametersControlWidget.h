#ifndef CAMERAMODELPARAMETERSCONTROLWIDGET_H
#define CAMERAMODELPARAMETERSCONTROLWIDGET_H

#include <QWidget>
#include "parametersControlWidgetBase.h"
#include "lensDistortionModelParameters.h"

#include "quaternion.h"
#include "vector3d.h"

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

    void getCameraParameters(double &fx, double &fy, double &cx, double &cy, double &skew, corecvs::Vector3dd &pos, corecvs::Quaternion &dir);
    void setCameraParameters(double &fx, double &fy, double &cx, double &cy, double &skew, corecvs::Vector3dd &pos, corecvs::Quaternion &dir);

    virtual void loadParamWidget(WidgetLoader &/*loader*/);
    virtual void saveParamWidget(WidgetSaver  &/*saver*/ );

private:
    void writeUi();
    void readUi();

    // FIXME: decide what model to use as camera parameters and change this members to corresponding struct
    double fx = 1.0, fy = 1.0, skew = 0.0, cx = 100.0, cy = 100.0;
    corecvs::Vector3dd _pos = corecvs::Vector3dd(0, 0, -10);
    corecvs::Quaternion _orientation = corecvs::Quaternion(0, 0, 0, 1);
    Ui::CameraModelParametersControlWidget *ui;
};

#endif // CAMERAMODELPARAMETERSCONTROLWIDGET_H
