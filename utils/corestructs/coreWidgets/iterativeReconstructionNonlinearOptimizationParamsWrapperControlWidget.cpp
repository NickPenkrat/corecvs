/**
 * \file iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.h"
#include "ui_iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"

#include "reconstructionFunctorOptimizationParamsControlWidget.h"

IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->optimizationParamsControlWidget, SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->errorTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->postAppendNonlinearIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->finalNonLinearIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->alternatingIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->excessiveQuaternionParametrizationCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
}

IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::~IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget()
{

    delete mUi;
}

void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::loadParamWidget(WidgetLoader &loader)
{
    IterativeReconstructionNonlinearOptimizationParamsWrapper *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    IterativeReconstructionNonlinearOptimizationParamsWrapper *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::getParameters(IterativeReconstructionNonlinearOptimizationParamsWrapper& params) const
{

//    params.setOptimizationParams(mUi->optimizationParamsControlWidget->createParameters());
    params.setErrorType        (static_cast<ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType>(mUi->errorTypeComboBox->currentIndex()));
    params.setPostAppendNonlinearIterations(mUi->postAppendNonlinearIterationsSpinBox->value());
    params.setFinalNonLinearIterations(mUi->finalNonLinearIterationsSpinBox->value());
    params.setAlternatingIterations(mUi->alternatingIterationsSpinBox->value());
    params.setExcessiveQuaternionParametrization(mUi->excessiveQuaternionParametrizationCheckBox->isChecked());

}

IterativeReconstructionNonlinearOptimizationParamsWrapper *IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/

    ReconstructionFunctorOptimizationParams *tmp0 = NULL;

    IterativeReconstructionNonlinearOptimizationParamsWrapper *result = new IterativeReconstructionNonlinearOptimizationParamsWrapper(
          * (tmp0 = mUi->optimizationParamsControlWidget->createParameters())
        , static_cast<ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType>(mUi->errorTypeComboBox->currentIndex())
        , mUi->postAppendNonlinearIterationsSpinBox->value()
        , mUi->finalNonLinearIterationsSpinBox->value()
        , mUi->alternatingIterationsSpinBox->value()
        , mUi->excessiveQuaternionParametrizationCheckBox->isChecked()
    );
    delete tmp0;
    return result;
}

void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::setParameters(const IterativeReconstructionNonlinearOptimizationParamsWrapper &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->optimizationParamsControlWidget->setParameters(input.optimizationParams());
    mUi->errorTypeComboBox->setCurrentIndex(input.errorType());
    mUi->postAppendNonlinearIterationsSpinBox->setValue(input.postAppendNonlinearIterations());
    mUi->finalNonLinearIterationsSpinBox->setValue(input.finalNonLinearIterations());
    mUi->alternatingIterationsSpinBox->setValue(input.alternatingIterations());
    mUi->excessiveQuaternionParametrizationCheckBox->setChecked(input.excessiveQuaternionParametrization());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    IterativeReconstructionNonlinearOptimizationParamsWrapper *inputCasted = static_cast<IterativeReconstructionNonlinearOptimizationParamsWrapper *>(input);
    setParameters(*inputCasted);
}