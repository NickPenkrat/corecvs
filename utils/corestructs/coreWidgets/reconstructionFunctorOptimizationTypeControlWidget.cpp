/**
 * \file reconstructionFunctorOptimizationTypeControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "reconstructionFunctorOptimizationTypeControlWidget.h"
#include "ui_reconstructionFunctorOptimizationTypeControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


ReconstructionFunctorOptimizationTypeControlWidget::ReconstructionFunctorOptimizationTypeControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::ReconstructionFunctorOptimizationTypeControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

}

ReconstructionFunctorOptimizationTypeControlWidget::~ReconstructionFunctorOptimizationTypeControlWidget()
{

    delete mUi;
}

void ReconstructionFunctorOptimizationTypeControlWidget::loadParamWidget(WidgetLoader &loader)
{
    ReconstructionFunctorOptimizationType *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void ReconstructionFunctorOptimizationTypeControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    ReconstructionFunctorOptimizationType *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void ReconstructionFunctorOptimizationTypeControlWidget::getParameters(ReconstructionFunctorOptimizationType& params) const
{


}

ReconstructionFunctorOptimizationType *ReconstructionFunctorOptimizationTypeControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    ReconstructionFunctorOptimizationType *result = new ReconstructionFunctorOptimizationType(
    );
    return result;
}

void ReconstructionFunctorOptimizationTypeControlWidget::setParameters(const ReconstructionFunctorOptimizationType &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void ReconstructionFunctorOptimizationTypeControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    ReconstructionFunctorOptimizationType *inputCasted = static_cast<ReconstructionFunctorOptimizationType *>(input);
    setParameters(*inputCasted);
}
