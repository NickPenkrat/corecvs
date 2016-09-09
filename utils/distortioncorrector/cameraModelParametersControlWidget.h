#ifndef CAMERAMODELPARAMETERSCONTROLWIDGET_H
#define CAMERAMODELPARAMETERSCONTROLWIDGET_H

#include <QWidget>
#include "parametersControlWidgetBase.h"
#include "lensDistortionModelParameters.h"

#include "calibrationCamera.h"

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


    CameraModel* createParameters() const;
    void getParameters(CameraModel &params) const;
    void setParameters(const CameraModel &input);
    virtual void setParametersVirtual(void *input);


    virtual void loadParamWidget(WidgetLoader &/*loader*/);
    virtual void saveParamWidget(WidgetSaver  &/*saver*/ );

public slots:
    /** We should consider who is responsible for what **/
    void loadPressed();
    void savePressed();

    void revertPressed();


    void paramsChangedInUI();

signals:
    void valueChanged();
    void paramsChanged();


    void loadRequest(QString filename);
    void saveRequest(QString filename);


protected:

    Ui::CameraModelParametersControlWidget *ui;
    CameraModel backup;

};


/* Should support additional protype field*/
class FixtureCameraParametersControlWidget : public CameraModelParametersControlWidget
{
public:
    explicit FixtureCameraParametersControlWidget(QWidget *parent = 0);


};

#endif // CAMERAMODELPARAMETERSCONTROLWIDGET_H
