/**
 * \file photostationPlacerFeatureSelectionParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "photostationPlacerFeatureSelectionParametersControlWidget.h"
#include "ui_photostationPlacerFeatureSelectionParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


PhotostationPlacerFeatureSelectionParametersControlWidget::PhotostationPlacerFeatureSelectionParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::PhotostationPlacerFeatureSelectionParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->inlierThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->trackInlierThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->pairCorrespondenceThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->distanceLimitSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

PhotostationPlacerFeatureSelectionParametersControlWidget::~PhotostationPlacerFeatureSelectionParametersControlWidget()
{

    delete mUi;
}

void PhotostationPlacerFeatureSelectionParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    PhotostationPlacerFeatureSelectionParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void PhotostationPlacerFeatureSelectionParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    PhotostationPlacerFeatureSelectionParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void PhotostationPlacerFeatureSelectionParametersControlWidget::getParameters(PhotostationPlacerFeatureSelectionParameters& params) const
{

    params.setInlierThreshold  (mUi->inlierThresholdSpinBox->value());
    params.setTrackInlierThreshold(mUi->trackInlierThresholdSpinBox->value());
    params.setPairCorrespondenceThreshold(mUi->pairCorrespondenceThresholdSpinBox->value());
    params.setDistanceLimit    (mUi->distanceLimitSpinBox->value());

}

PhotostationPlacerFeatureSelectionParameters *PhotostationPlacerFeatureSelectionParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    PhotostationPlacerFeatureSelectionParameters *result = new PhotostationPlacerFeatureSelectionParameters(
          mUi->inlierThresholdSpinBox->value()
        , mUi->trackInlierThresholdSpinBox->value()
        , mUi->pairCorrespondenceThresholdSpinBox->value()
        , mUi->distanceLimitSpinBox->value()
    );
    return result;
}

void PhotostationPlacerFeatureSelectionParametersControlWidget::setParameters(const PhotostationPlacerFeatureSelectionParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->inlierThresholdSpinBox->setValue(input.inlierThreshold());
    mUi->trackInlierThresholdSpinBox->setValue(input.trackInlierThreshold());
    mUi->pairCorrespondenceThresholdSpinBox->setValue(input.pairCorrespondenceThreshold());
    mUi->distanceLimitSpinBox->setValue(input.distanceLimit());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void PhotostationPlacerFeatureSelectionParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    PhotostationPlacerFeatureSelectionParameters *inputCasted = static_cast<PhotostationPlacerFeatureSelectionParameters *>(input);
    setParameters(*inputCasted);
}
