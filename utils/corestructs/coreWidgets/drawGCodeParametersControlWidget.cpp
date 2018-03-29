/**
 * \file drawGCodeParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "drawGCodeParametersControlWidget.h"
#include "ui_drawGCodeParametersControlWidget.h"
#include <memory>
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


DrawGCodeParametersControlWidget::DrawGCodeParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::DrawGCodeParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->schemeComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->minTempSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxTempSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->minExtrudeSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxExtrudeSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->minSpeedSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxSpeedSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

DrawGCodeParametersControlWidget::~DrawGCodeParametersControlWidget()
{

    delete mUi;
}

void DrawGCodeParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    std::unique_ptr<DrawGCodeParameters> params(createParameters());
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
}

void DrawGCodeParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    saver.saveParameters(*std::unique_ptr<DrawGCodeParameters>(createParameters()), rootPath);
}

void DrawGCodeParametersControlWidget::getParameters(DrawGCodeParameters& params) const
{
    params = *std::unique_ptr<DrawGCodeParameters>(createParameters());
}


DrawGCodeParameters *DrawGCodeParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    return new DrawGCodeParameters(
          static_cast<GCodeColoringSheme::GCodeColoringSheme>(mUi->schemeComboBox->currentIndex())
        , mUi->minTempSpinBox->value()
        , mUi->maxTempSpinBox->value()
        , mUi->minExtrudeSpinBox->value()
        , mUi->maxExtrudeSpinBox->value()
        , mUi->minSpeedSpinBox->value()
        , mUi->maxSpeedSpinBox->value()
    );
}

void DrawGCodeParametersControlWidget::setParameters(const DrawGCodeParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->schemeComboBox->setCurrentIndex(input.scheme());
    mUi->minTempSpinBox->setValue(input.minTemp());
    mUi->maxTempSpinBox->setValue(input.maxTemp());
    mUi->minExtrudeSpinBox->setValue(input.minExtrude());
    mUi->maxExtrudeSpinBox->setValue(input.maxExtrude());
    mUi->minSpeedSpinBox->setValue(input.minSpeed());
    mUi->maxSpeedSpinBox->setValue(input.maxSpeed());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void DrawGCodeParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    DrawGCodeParameters *inputCasted = static_cast<DrawGCodeParameters *>(input);
    setParameters(*inputCasted);
}
