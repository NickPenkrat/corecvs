/**
 * \file distortionApplicationParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "distortionApplicationParametersControlWidget.h"
#include "ui_distortionApplicationParametersControlWidget.h"
#include <memory>
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


DistortionApplicationParametersControlWidget::DistortionApplicationParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::DistortionApplicationParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->forceScaleCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->adoptScaleCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->resizePolicyComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->newHSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->newWSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
}

DistortionApplicationParametersControlWidget::~DistortionApplicationParametersControlWidget()
{

    delete mUi;
}

void DistortionApplicationParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    std::unique_ptr<DistortionApplicationParameters> params(createParameters());
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
}

void DistortionApplicationParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    saver.saveParameters(*std::unique_ptr<DistortionApplicationParameters>(createParameters()), rootPath);
}

void DistortionApplicationParametersControlWidget::getParameters(DistortionApplicationParameters& params) const
{
    params = *std::unique_ptr<DistortionApplicationParameters>(createParameters());
}


DistortionApplicationParameters *DistortionApplicationParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    return new DistortionApplicationParameters(
          mUi->forceScaleCheckBox->isChecked()
        , mUi->adoptScaleCheckBox->isChecked()
        , static_cast<DistortionResizePolicy::DistortionResizePolicy>(mUi->resizePolicyComboBox->currentIndex())
        , mUi->newHSpinBox->value()
        , mUi->newWSpinBox->value()
    );
}

void DistortionApplicationParametersControlWidget::setParameters(const DistortionApplicationParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->forceScaleCheckBox->setChecked(input.forceScale());
    mUi->adoptScaleCheckBox->setChecked(input.adoptScale());
    mUi->resizePolicyComboBox->setCurrentIndex(input.resizePolicy());
    mUi->newHSpinBox->setValue(input.newH());
    mUi->newWSpinBox->setValue(input.newW());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void DistortionApplicationParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    DistortionApplicationParameters *inputCasted = static_cast<DistortionApplicationParameters *>(input);
    setParameters(*inputCasted);
}
