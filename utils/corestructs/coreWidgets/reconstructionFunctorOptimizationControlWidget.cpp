/**
 * \file reconstructionFunctorOptimizationControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "reconstructionFunctorOptimizationControlWidget.h"
#include "ui_reconstructionFunctorOptimizationControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


ReconstructionFunctorOptimizationControlWidget::ReconstructionFunctorOptimizationControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::ReconstructionFunctorOptimizationControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->nON_DEGENERATE_ORIENTATIONSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->dEGENERATE_ORIENTATIONSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->nON_DEGENERATE_TRANSLATIONSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->dEGENERATE_TRANSLATIONSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->fOCALSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->pRINCIPALSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->pOINTSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->tUNE_GPSCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
}

ReconstructionFunctorOptimizationControlWidget::~ReconstructionFunctorOptimizationControlWidget()
{

    delete mUi;
}

void ReconstructionFunctorOptimizationControlWidget::loadParamWidget(WidgetLoader &loader)
{
    ReconstructionFunctorOptimization *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void ReconstructionFunctorOptimizationControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    ReconstructionFunctorOptimization *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void ReconstructionFunctorOptimizationControlWidget::getParameters(ReconstructionFunctorOptimization& params) const
{

    params.setNON_DEGENERATE_ORIENTATIONS(mUi->nON_DEGENERATE_ORIENTATIONSCheckBox->isChecked());
    params.setDEGENERATE_ORIENTATIONS(mUi->dEGENERATE_ORIENTATIONSCheckBox->isChecked());
    params.setNON_DEGENERATE_TRANSLATIONS(mUi->nON_DEGENERATE_TRANSLATIONSCheckBox->isChecked());
    params.setDEGENERATE_TRANSLATIONS(mUi->dEGENERATE_TRANSLATIONSCheckBox->isChecked());
    params.setFOCALS           (mUi->fOCALSCheckBox->isChecked());
    params.setPRINCIPALS       (mUi->pRINCIPALSCheckBox->isChecked());
    params.setPOINTS           (mUi->pOINTSCheckBox->isChecked());
    params.setTUNE_GPS         (mUi->tUNE_GPSCheckBox->isChecked());

}

ReconstructionFunctorOptimization *ReconstructionFunctorOptimizationControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    ReconstructionFunctorOptimization *result = new ReconstructionFunctorOptimization(
          mUi->nON_DEGENERATE_ORIENTATIONSCheckBox->isChecked()
        , mUi->dEGENERATE_ORIENTATIONSCheckBox->isChecked()
        , mUi->nON_DEGENERATE_TRANSLATIONSCheckBox->isChecked()
        , mUi->dEGENERATE_TRANSLATIONSCheckBox->isChecked()
        , mUi->fOCALSCheckBox->isChecked()
        , mUi->pRINCIPALSCheckBox->isChecked()
        , mUi->pOINTSCheckBox->isChecked()
        , mUi->tUNE_GPSCheckBox->isChecked()
    );
    return result;
}

void ReconstructionFunctorOptimizationControlWidget::setParameters(const ReconstructionFunctorOptimization &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->nON_DEGENERATE_ORIENTATIONSCheckBox->setChecked(input.nON_DEGENERATE_ORIENTATIONS());
    mUi->dEGENERATE_ORIENTATIONSCheckBox->setChecked(input.dEGENERATE_ORIENTATIONS());
    mUi->nON_DEGENERATE_TRANSLATIONSCheckBox->setChecked(input.nON_DEGENERATE_TRANSLATIONS());
    mUi->dEGENERATE_TRANSLATIONSCheckBox->setChecked(input.dEGENERATE_TRANSLATIONS());
    mUi->fOCALSCheckBox->setChecked(input.fOCALS());
    mUi->pRINCIPALSCheckBox->setChecked(input.pRINCIPALS());
    mUi->pOINTSCheckBox->setChecked(input.pOINTS());
    mUi->tUNE_GPSCheckBox->setChecked(input.tUNE_GPS());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void ReconstructionFunctorOptimizationControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    ReconstructionFunctorOptimization *inputCasted = static_cast<ReconstructionFunctorOptimization *>(input);
    setParameters(*inputCasted);
}