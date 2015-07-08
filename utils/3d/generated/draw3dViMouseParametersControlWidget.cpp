/**
 * \file draw3dViMouseParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "draw3dViMouseParametersControlWidget.h"
#include "ui_draw3dViMouseParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


Draw3dViMouseParametersControlWidget::Draw3dViMouseParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::Draw3dViMouseParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->redDistSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->blueDistSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->flowZoomSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->pointColorTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->flowColorTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
}

Draw3dViMouseParametersControlWidget::~Draw3dViMouseParametersControlWidget()
{

    delete mUi;
}

void Draw3dViMouseParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    Draw3dViMouseParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void Draw3dViMouseParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    Draw3dViMouseParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void Draw3dViMouseParametersControlWidget::getParameters(Draw3dViMouseParameters& params) const
{

    params.setRedDist          (mUi->redDistSpinBox->value());
    params.setBlueDist         (mUi->blueDistSpinBox->value());
    params.setFlowZoom         (mUi->flowZoomSpinBox->value());
    params.setPointColorType   (static_cast<ViMouse3dStereoStyle::ViMouse3dStereoStyle>(mUi->pointColorTypeComboBox->currentIndex()));
    params.setFlowColorType    (static_cast<ViMouse3dFlowStyle::ViMouse3dFlowStyle>(mUi->flowColorTypeComboBox->currentIndex()));

}

Draw3dViMouseParameters *Draw3dViMouseParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    Draw3dViMouseParameters *result = new Draw3dViMouseParameters(
          mUi->redDistSpinBox->value()
        , mUi->blueDistSpinBox->value()
        , mUi->flowZoomSpinBox->value()
        , static_cast<ViMouse3dStereoStyle::ViMouse3dStereoStyle>(mUi->pointColorTypeComboBox->currentIndex())
        , static_cast<ViMouse3dFlowStyle::ViMouse3dFlowStyle>(mUi->flowColorTypeComboBox->currentIndex())
    );
    return result;
}

void Draw3dViMouseParametersControlWidget::setParameters(const Draw3dViMouseParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->redDistSpinBox->setValue(input.redDist());
    mUi->blueDistSpinBox->setValue(input.blueDist());
    mUi->flowZoomSpinBox->setValue(input.flowZoom());
    mUi->pointColorTypeComboBox->setCurrentIndex(input.pointColorType());
    mUi->flowColorTypeComboBox->setCurrentIndex(input.flowColorType());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void Draw3dViMouseParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    Draw3dViMouseParameters *inputCasted = static_cast<Draw3dViMouseParameters *>(input);
    setParameters(*inputCasted);
}
