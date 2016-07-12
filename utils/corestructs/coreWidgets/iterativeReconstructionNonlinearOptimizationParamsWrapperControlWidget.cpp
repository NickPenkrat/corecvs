/**
 * \file iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.h"
#include "ui_iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.h"
#include <memory>
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
    std::unique_ptr<IterativeReconstructionNonlinearOptimizationParamsWrapper> params(createParameters());
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
}

void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    saver.saveParameters(*std::unique_ptr<IterativeReconstructionNonlinearOptimizationParamsWrapper>(createParameters()), rootPath);
}

void IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::getParameters(IterativeReconstructionNonlinearOptimizationParamsWrapper& params) const
{
    params = *std::unique_ptr<IterativeReconstructionNonlinearOptimizationParamsWrapper>(createParameters());
}


IterativeReconstructionNonlinearOptimizationParamsWrapper *IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    return new IterativeReconstructionNonlinearOptimizationParamsWrapper(
          *std::unique_ptr<ReconstructionFunctorOptimizationParams>(mUi->optimizationParamsControlWidget->createParameters())
        , static_cast<ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType>(mUi->errorTypeComboBox->currentIndex())
        , mUi->postAppendNonlinearIterationsSpinBox->value()
        , mUi->finalNonLinearIterationsSpinBox->value()
        , mUi->alternatingIterationsSpinBox->value()
        , mUi->excessiveQuaternionParametrizationCheckBox->isChecked()
    );
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
