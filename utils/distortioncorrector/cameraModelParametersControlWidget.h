#ifndef CAMERAMODELPARAMETERSCONTROLWIDGET_H
#define CAMERAMODELPARAMETERSCONTROLWIDGET_H

#include <QWidget>
#include <QComboBox>

#include "parametersControlWidgetBase.h"
#include "core/alignment/lensDistortionModelParameters.h"

#include "core/cameracalibration/cameraModel.h"

#include "core/math/quaternion.h"
#include "core/math/vector/vector3d.h"

#include "reflectionWidget.h"

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

    void zeroPressed ();
    void resetPressed();


    void paramsChangedInUI();

signals:
    void valueChanged();
    void paramsChanged();


    void loadRequest(QString filename);
    void saveRequest(QString filename);


protected:

    Ui::CameraModelParametersControlWidget *ui;
    CameraModel backup;

    /**/
    QComboBox *intrinsicsType = NULL;
    ReflectionWidget *intrinsicsWidget = NULL;

};


/* Should support additional protype field*/
class FixtureCameraParametersControlWidget : public CameraModelParametersControlWidget
{
public:
    explicit FixtureCameraParametersControlWidget(QWidget *parent = 0);


};

#endif // CAMERAMODELPARAMETERSCONTROLWIDGET_H
