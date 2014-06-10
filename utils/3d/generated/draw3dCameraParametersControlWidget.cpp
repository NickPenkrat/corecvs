/**
 * \file draw3dCameraParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "draw3dCameraParametersControlWidget.h"
#include "ui_draw3dCameraParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"

#include "rgbColorParametersControlWidget.h"
#include "rgbColorParametersControlWidget.h"

Draw3dCameraParametersControlWidget::Draw3dCameraParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::Draw3dCameraParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->fovHSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->fovVSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->nearPlaneSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->farPlaneSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->styleComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->colorControlWidget, SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->secondaryColorControlWidget, SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->showCaptionCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->fontSizeSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->textureCorrodinatesComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->textureAlphaSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->textureScaleSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->decalMatrixTypeSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->decalLeftCamCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->decalLeftAlphaSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->decalRightCamCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->decalRightAlphaSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
}

Draw3dCameraParametersControlWidget::~Draw3dCameraParametersControlWidget()
{

    delete mUi;
}

void Draw3dCameraParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    Draw3dCameraParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void Draw3dCameraParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    Draw3dCameraParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}


Draw3dCameraParameters *Draw3dCameraParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/

    RgbColorParameters *tmp5 = NULL;
    RgbColorParameters *tmp6 = NULL;

    Draw3dCameraParameters *result = new Draw3dCameraParameters(
          mUi->fovHSpinBox->value()
        , mUi->fovVSpinBox->value()
        , mUi->nearPlaneSpinBox->value()
        , mUi->farPlaneSpinBox->value()
        , static_cast<Draw3dStyle::Draw3dStyle>(mUi->styleComboBox->currentIndex())
        , * (tmp5 = mUi->colorControlWidget->createParameters())
        , * (tmp6 = mUi->secondaryColorControlWidget->createParameters())
        , mUi->showCaptionCheckBox->isChecked()
        , mUi->fontSizeSpinBox->value()
        , static_cast<Draw3dTextureGen::Draw3dTextureGen>(mUi->textureCorrodinatesComboBox->currentIndex())
        , mUi->textureAlphaSpinBox->value()
        , mUi->textureScaleSpinBox->value()
        , mUi->decalMatrixTypeSpinBox->value()
        , mUi->decalLeftCamCheckBox->isChecked()
        , mUi->decalLeftAlphaSpinBox->value()
        , mUi->decalRightCamCheckBox->isChecked()
        , mUi->decalRightAlphaSpinBox->value()
    );
    delete tmp5;
    delete tmp6;
    return result;
}

void Draw3dCameraParametersControlWidget::setParameters(const Draw3dCameraParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->fovHSpinBox->setValue(input.fovH());
    mUi->fovVSpinBox->setValue(input.fovV());
    mUi->nearPlaneSpinBox->setValue(input.nearPlane());
    mUi->farPlaneSpinBox->setValue(input.farPlane());
    mUi->styleComboBox->setCurrentIndex(input.style());
    mUi->colorControlWidget->setParameters(input.color());
    mUi->secondaryColorControlWidget->setParameters(input.secondaryColor());
    mUi->showCaptionCheckBox->setChecked(input.showCaption());
    mUi->fontSizeSpinBox->setValue(input.fontSize());
    mUi->textureCorrodinatesComboBox->setCurrentIndex(input.textureCorrodinates());
    mUi->textureAlphaSpinBox->setValue(input.textureAlpha());
    mUi->textureScaleSpinBox->setValue(input.textureScale());
    mUi->decalMatrixTypeSpinBox->setValue(input.decalMatrixType());
    mUi->decalLeftCamCheckBox->setChecked(input.decalLeftCam());
    mUi->decalLeftAlphaSpinBox->setValue(input.decalLeftAlpha());
    mUi->decalRightCamCheckBox->setChecked(input.decalRightCam());
    mUi->decalRightAlphaSpinBox->setValue(input.decalRightAlpha());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void Draw3dCameraParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    Draw3dCameraParameters *inputCasted = static_cast<Draw3dCameraParameters *>(input);
    setParameters(*inputCasted);
}
