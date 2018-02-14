/**
 * \file calibrationDrawHelpersParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "calibrationDrawHelpersParametersControlWidget.h"
#include "ui_calibrationDrawHelpersParametersControlWidget.h"
#include <memory>
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


CalibrationDrawHelpersParametersControlWidget::CalibrationDrawHelpersParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::CalibrationDrawHelpersParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->backendComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->scaleForCamerasSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->gridStepForCamerasSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->useTexturesForCamerasCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->printNamesCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->billboardNamesCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->preferReprojectedCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->forceKnownCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->privateColorCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->largePointsCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->drawFixtureCamsCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->solidCamerasCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->drawObservationsCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->drawTrueLinesCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->projectionRayLengthSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->drawRaysCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
}

CalibrationDrawHelpersParametersControlWidget::~CalibrationDrawHelpersParametersControlWidget()
{

    delete mUi;
}

void CalibrationDrawHelpersParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    std::unique_ptr<CalibrationDrawHelpersParameters> params(createParameters());
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
}

void CalibrationDrawHelpersParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    saver.saveParameters(*std::unique_ptr<CalibrationDrawHelpersParameters>(createParameters()), rootPath);
}

void CalibrationDrawHelpersParametersControlWidget::getParameters(CalibrationDrawHelpersParameters& params) const
{
    params = *std::unique_ptr<CalibrationDrawHelpersParameters>(createParameters());
}


CalibrationDrawHelpersParameters *CalibrationDrawHelpersParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    return new CalibrationDrawHelpersParameters(
          static_cast<SceneDrawBackendType::SceneDrawBackendType>(mUi->backendComboBox->currentIndex())
        , mUi->scaleForCamerasSpinBox->value()
        , mUi->gridStepForCamerasSpinBox->value()
        , mUi->useTexturesForCamerasCheckBox->isChecked()
        , mUi->printNamesCheckBox->isChecked()
        , mUi->billboardNamesCheckBox->isChecked()
        , mUi->preferReprojectedCheckBox->isChecked()
        , mUi->forceKnownCheckBox->isChecked()
        , mUi->privateColorCheckBox->isChecked()
        , mUi->largePointsCheckBox->isChecked()
        , mUi->drawFixtureCamsCheckBox->isChecked()
        , mUi->solidCamerasCheckBox->isChecked()
        , mUi->drawObservationsCheckBox->isChecked()
        , mUi->drawTrueLinesCheckBox->isChecked()
        , mUi->projectionRayLengthSpinBox->value()
        , mUi->drawRaysCheckBox->isChecked()
    );
}

void CalibrationDrawHelpersParametersControlWidget::setParameters(const CalibrationDrawHelpersParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->backendComboBox->setCurrentIndex(input.backend());
    mUi->scaleForCamerasSpinBox->setValue(input.scaleForCameras());
    mUi->gridStepForCamerasSpinBox->setValue(input.gridStepForCameras());
    mUi->useTexturesForCamerasCheckBox->setChecked(input.useTexturesForCameras());
    mUi->printNamesCheckBox->setChecked(input.printNames());
    mUi->billboardNamesCheckBox->setChecked(input.billboardNames());
    mUi->preferReprojectedCheckBox->setChecked(input.preferReprojected());
    mUi->forceKnownCheckBox->setChecked(input.forceKnown());
    mUi->privateColorCheckBox->setChecked(input.privateColor());
    mUi->largePointsCheckBox->setChecked(input.largePoints());
    mUi->drawFixtureCamsCheckBox->setChecked(input.drawFixtureCams());
    mUi->solidCamerasCheckBox->setChecked(input.solidCameras());
    mUi->drawObservationsCheckBox->setChecked(input.drawObservations());
    mUi->drawTrueLinesCheckBox->setChecked(input.drawTrueLines());
    mUi->projectionRayLengthSpinBox->setValue(input.projectionRayLength());
    mUi->drawRaysCheckBox->setChecked(input.drawRays());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void CalibrationDrawHelpersParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    CalibrationDrawHelpersParameters *inputCasted = static_cast<CalibrationDrawHelpersParameters *>(input);
    setParameters(*inputCasted);
}
