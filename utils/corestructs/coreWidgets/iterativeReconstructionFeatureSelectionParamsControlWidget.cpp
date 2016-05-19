/**
 * \file iterativeReconstructionFeatureSelectionParamsControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "iterativeReconstructionFeatureSelectionParamsControlWidget.h"
#include "ui_iterativeReconstructionFeatureSelectionParamsControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


IterativeReconstructionFeatureSelectionParamsControlWidget::IterativeReconstructionFeatureSelectionParamsControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::IterativeReconstructionFeatureSelectionParamsControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->inlierThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->trackInlierThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->distanceLimitSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->detectorEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->descriptorEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->matcherEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->b2bThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->rmsePruningScalerSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->maxPruningScalerSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

IterativeReconstructionFeatureSelectionParamsControlWidget::~IterativeReconstructionFeatureSelectionParamsControlWidget()
{

    delete mUi;
}

void IterativeReconstructionFeatureSelectionParamsControlWidget::loadParamWidget(WidgetLoader &loader)
{
    IterativeReconstructionFeatureSelectionParams *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void IterativeReconstructionFeatureSelectionParamsControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    IterativeReconstructionFeatureSelectionParams *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void IterativeReconstructionFeatureSelectionParamsControlWidget::getParameters(IterativeReconstructionFeatureSelectionParams& params) const
{

    params.setInlierThreshold  (mUi->inlierThresholdSpinBox->value());
    params.setTrackInlierThreshold(mUi->trackInlierThresholdSpinBox->value());
    params.setDistanceLimit    (mUi->distanceLimitSpinBox->value());
    params.setDetector         (mUi->detectorEdit->text().toStdString());
    params.setDescriptor       (mUi->descriptorEdit->text().toStdString());
    params.setMatcher          (mUi->matcherEdit->text().toStdString());
    params.setB2bThreshold     (mUi->b2bThresholdSpinBox->value());
    params.setRmsePruningScaler(mUi->rmsePruningScalerSpinBox->value());
    params.setMaxPruningScaler (mUi->maxPruningScalerSpinBox->value());

}

IterativeReconstructionFeatureSelectionParams *IterativeReconstructionFeatureSelectionParamsControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    IterativeReconstructionFeatureSelectionParams *result = new IterativeReconstructionFeatureSelectionParams(
          mUi->inlierThresholdSpinBox->value()
        , mUi->trackInlierThresholdSpinBox->value()
        , mUi->distanceLimitSpinBox->value()
        , mUi->detectorEdit->text().toStdString()
        , mUi->descriptorEdit->text().toStdString()
        , mUi->matcherEdit->text().toStdString()
        , mUi->b2bThresholdSpinBox->value()
        , mUi->rmsePruningScalerSpinBox->value()
        , mUi->maxPruningScalerSpinBox->value()
    );
    return result;
}

void IterativeReconstructionFeatureSelectionParamsControlWidget::setParameters(const IterativeReconstructionFeatureSelectionParams &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->inlierThresholdSpinBox->setValue(input.inlierThreshold());
    mUi->trackInlierThresholdSpinBox->setValue(input.trackInlierThreshold());
    mUi->distanceLimitSpinBox->setValue(input.distanceLimit());
    mUi->detectorEdit->setText(input.detector().c_str());
    mUi->descriptorEdit->setText(input.descriptor().c_str());
    mUi->matcherEdit->setText(input.matcher().c_str());
    mUi->b2bThresholdSpinBox->setValue(input.b2bThreshold());
    mUi->rmsePruningScalerSpinBox->setValue(input.rmsePruningScaler());
    mUi->maxPruningScalerSpinBox->setValue(input.maxPruningScaler());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void IterativeReconstructionFeatureSelectionParamsControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    IterativeReconstructionFeatureSelectionParams *inputCasted = static_cast<IterativeReconstructionFeatureSelectionParams *>(input);
    setParameters(*inputCasted);
}
