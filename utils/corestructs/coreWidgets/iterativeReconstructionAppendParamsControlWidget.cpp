/**
 * \file iterativeReconstructionAppendParamsControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "iterativeReconstructionAppendParamsControlWidget.h"
#include "ui_iterativeReconstructionAppendParamsControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


IterativeReconstructionAppendParamsControlWidget::IterativeReconstructionAppendParamsControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::IterativeReconstructionAppendParamsControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->maxPostAppendSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->inlierP3PThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxP3PIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->inlierP6PThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxP6PIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->gammaP6PSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->speculativitySpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->minimalInlierCountSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maximalFailureProbabilitySpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

IterativeReconstructionAppendParamsControlWidget::~IterativeReconstructionAppendParamsControlWidget()
{

    delete mUi;
}

void IterativeReconstructionAppendParamsControlWidget::loadParamWidget(WidgetLoader &loader)
{
    IterativeReconstructionAppendParams *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void IterativeReconstructionAppendParamsControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    IterativeReconstructionAppendParams *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void IterativeReconstructionAppendParamsControlWidget::getParameters(IterativeReconstructionAppendParams& params) const
{

    params.setMaxPostAppend    (mUi->maxPostAppendSpinBox->value());
    params.setInlierP3PThreshold(mUi->inlierP3PThresholdSpinBox->value());
    params.setMaxP3PIterations (mUi->maxP3PIterationsSpinBox->value());
    params.setInlierP6PThreshold(mUi->inlierP6PThresholdSpinBox->value());
    params.setMaxP6PIterations (mUi->maxP6PIterationsSpinBox->value());
    params.setGammaP6P         (mUi->gammaP6PSpinBox->value());
    params.setSpeculativity    (mUi->speculativitySpinBox->value());
    params.setMinimalInlierCount(mUi->minimalInlierCountSpinBox->value());
    params.setMaximalFailureProbability(mUi->maximalFailureProbabilitySpinBox->value());

}

IterativeReconstructionAppendParams *IterativeReconstructionAppendParamsControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    IterativeReconstructionAppendParams *result = new IterativeReconstructionAppendParams(
          mUi->maxPostAppendSpinBox->value()
        , mUi->inlierP3PThresholdSpinBox->value()
        , mUi->maxP3PIterationsSpinBox->value()
        , mUi->inlierP6PThresholdSpinBox->value()
        , mUi->maxP6PIterationsSpinBox->value()
        , mUi->gammaP6PSpinBox->value()
        , mUi->speculativitySpinBox->value()
        , mUi->minimalInlierCountSpinBox->value()
        , mUi->maximalFailureProbabilitySpinBox->value()
    );
    return result;
}

void IterativeReconstructionAppendParamsControlWidget::setParameters(const IterativeReconstructionAppendParams &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->maxPostAppendSpinBox->setValue(input.maxPostAppend());
    mUi->inlierP3PThresholdSpinBox->setValue(input.inlierP3PThreshold());
    mUi->maxP3PIterationsSpinBox->setValue(input.maxP3PIterations());
    mUi->inlierP6PThresholdSpinBox->setValue(input.inlierP6PThreshold());
    mUi->maxP6PIterationsSpinBox->setValue(input.maxP6PIterations());
    mUi->gammaP6PSpinBox->setValue(input.gammaP6P());
    mUi->speculativitySpinBox->setValue(input.speculativity());
    mUi->minimalInlierCountSpinBox->setValue(input.minimalInlierCount());
    mUi->maximalFailureProbabilitySpinBox->setValue(input.maximalFailureProbability());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void IterativeReconstructionAppendParamsControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    IterativeReconstructionAppendParams *inputCasted = static_cast<IterativeReconstructionAppendParams *>(input);
    setParameters(*inputCasted);
}
