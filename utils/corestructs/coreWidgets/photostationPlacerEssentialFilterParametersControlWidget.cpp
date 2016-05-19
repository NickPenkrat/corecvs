/**
 * \file photostationPlacerEssentialFilterParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "photostationPlacerEssentialFilterParametersControlWidget.h"
#include "ui_photostationPlacerEssentialFilterParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


PhotostationPlacerEssentialFilterParametersControlWidget::PhotostationPlacerEssentialFilterParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::PhotostationPlacerEssentialFilterParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->b2bRansacP5RPThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->inlierP5RPThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxEssentialRansacIterationsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->b2bRansacP6RPThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->runEssentialFilteringCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->essentialTargetGammaSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

PhotostationPlacerEssentialFilterParametersControlWidget::~PhotostationPlacerEssentialFilterParametersControlWidget()
{

    delete mUi;
}

void PhotostationPlacerEssentialFilterParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    PhotostationPlacerEssentialFilterParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void PhotostationPlacerEssentialFilterParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    PhotostationPlacerEssentialFilterParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void PhotostationPlacerEssentialFilterParametersControlWidget::getParameters(PhotostationPlacerEssentialFilterParameters& params) const
{

    params.setB2bRansacP5RPThreshold(mUi->b2bRansacP5RPThresholdSpinBox->value());
    params.setInlierP5RPThreshold(mUi->inlierP5RPThresholdSpinBox->value());
    params.setMaxEssentialRansacIterations(mUi->maxEssentialRansacIterationsSpinBox->value());
    params.setB2bRansacP6RPThreshold(mUi->b2bRansacP6RPThresholdSpinBox->value());
    params.setRunEssentialFiltering(mUi->runEssentialFilteringCheckBox->isChecked());
    params.setEssentialTargetGamma(mUi->essentialTargetGammaSpinBox->value());

}

PhotostationPlacerEssentialFilterParameters *PhotostationPlacerEssentialFilterParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    PhotostationPlacerEssentialFilterParameters *result = new PhotostationPlacerEssentialFilterParameters(
          mUi->b2bRansacP5RPThresholdSpinBox->value()
        , mUi->inlierP5RPThresholdSpinBox->value()
        , mUi->maxEssentialRansacIterationsSpinBox->value()
        , mUi->b2bRansacP6RPThresholdSpinBox->value()
        , mUi->runEssentialFilteringCheckBox->isChecked()
        , mUi->essentialTargetGammaSpinBox->value()
    );
    return result;
}

void PhotostationPlacerEssentialFilterParametersControlWidget::setParameters(const PhotostationPlacerEssentialFilterParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->b2bRansacP5RPThresholdSpinBox->setValue(input.b2bRansacP5RPThreshold());
    mUi->inlierP5RPThresholdSpinBox->setValue(input.inlierP5RPThreshold());
    mUi->maxEssentialRansacIterationsSpinBox->setValue(input.maxEssentialRansacIterations());
    mUi->b2bRansacP6RPThresholdSpinBox->setValue(input.b2bRansacP6RPThreshold());
    mUi->runEssentialFilteringCheckBox->setChecked(input.runEssentialFiltering());
    mUi->essentialTargetGammaSpinBox->setValue(input.essentialTargetGamma());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void PhotostationPlacerEssentialFilterParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    PhotostationPlacerEssentialFilterParameters *inputCasted = static_cast<PhotostationPlacerEssentialFilterParameters *>(input);
    setParameters(*inputCasted);
}