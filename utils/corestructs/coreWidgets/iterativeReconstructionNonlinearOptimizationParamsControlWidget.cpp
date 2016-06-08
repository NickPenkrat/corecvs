/**
 * \file iterativeReconstructionNonlinearOptimizationParamsControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "iterativeReconstructionNonlinearOptimizationParamsControlWidget.h"
#include "ui_iterativeReconstructionNonlinearOptimizationParamsControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


IterativeReconstructionNonlinearOptimizationParamsControlWidget::IterativeReconstructionNonlinearOptimizationParamsControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::IterativeReconstructionNonlinearOptimizationParamsControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->errorTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->postAppendNonlinearIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->finalNonLinearIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->alternatingIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->excessiveQuaternionParametrizationCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
}

IterativeReconstructionNonlinearOptimizationParamsControlWidget::~IterativeReconstructionNonlinearOptimizationParamsControlWidget()
{

    delete mUi;
}

void IterativeReconstructionNonlinearOptimizationParamsControlWidget::loadParamWidget(WidgetLoader &loader)
{
    IterativeReconstructionNonlinearOptimizationParams *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void IterativeReconstructionNonlinearOptimizationParamsControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    IterativeReconstructionNonlinearOptimizationParams *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void IterativeReconstructionNonlinearOptimizationParamsControlWidget::getParameters(IterativeReconstructionNonlinearOptimizationParams& params) const
{

    params.setErrorType        (static_cast<ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType>(mUi->errorTypeComboBox->currentIndex()));
    params.setPostAppendNonlinearIterations(mUi->postAppendNonlinearIterationsSpinBox->value());
    params.setFinalNonLinearIterations(mUi->finalNonLinearIterationsSpinBox->value());
    params.setAlternatingIterations(mUi->alternatingIterationsSpinBox->value());
    params.setExcessiveQuaternionParametrization(mUi->excessiveQuaternionParametrizationCheckBox->isChecked());

}

IterativeReconstructionNonlinearOptimizationParams *IterativeReconstructionNonlinearOptimizationParamsControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    IterativeReconstructionNonlinearOptimizationParams *result = new IterativeReconstructionNonlinearOptimizationParams(
          static_cast<ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType>(mUi->errorTypeComboBox->currentIndex())
        , mUi->postAppendNonlinearIterationsSpinBox->value()
        , mUi->finalNonLinearIterationsSpinBox->value()
        , mUi->alternatingIterationsSpinBox->value()
        , mUi->excessiveQuaternionParametrizationCheckBox->isChecked()
    );
    return result;
}

void IterativeReconstructionNonlinearOptimizationParamsControlWidget::setParameters(const IterativeReconstructionNonlinearOptimizationParams &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->errorTypeComboBox->setCurrentIndex(input.errorType());
    mUi->postAppendNonlinearIterationsSpinBox->setValue(input.postAppendNonlinearIterations());
    mUi->finalNonLinearIterationsSpinBox->setValue(input.finalNonLinearIterations());
    mUi->alternatingIterationsSpinBox->setValue(input.alternatingIterations());
    mUi->excessiveQuaternionParametrizationCheckBox->setChecked(input.excessiveQuaternionParametrization());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void IterativeReconstructionNonlinearOptimizationParamsControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    IterativeReconstructionNonlinearOptimizationParams *inputCasted = static_cast<IterativeReconstructionNonlinearOptimizationParams *>(input);
    setParameters(*inputCasted);
}