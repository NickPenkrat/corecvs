/**
 * \file checkerboardDetectionParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "checkerboardDetectionParametersControlWidget.h"
#include "ui_checkerboardDetectionParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


CheckerboardDetectionParametersControlWidget::CheckerboardDetectionParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::CheckerboardDetectionParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->channelComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->vCrossesCountSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->hCrossesCountSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->cellSizeSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->cleanExistingCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
}

CheckerboardDetectionParametersControlWidget::~CheckerboardDetectionParametersControlWidget()
{

    delete mUi;
}

void CheckerboardDetectionParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    CheckerboardDetectionParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void CheckerboardDetectionParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    CheckerboardDetectionParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void CheckerboardDetectionParametersControlWidget::getParameters(CheckerboardDetectionParameters& params) const
{

    params.setChannel          (static_cast<ImageChannel::ImageChannel>(mUi->channelComboBox->currentIndex()));
    params.setVCrossesCount    (mUi->vCrossesCountSpinBox->value());
    params.setHCrossesCount    (mUi->hCrossesCountSpinBox->value());
    params.setCellSize         (mUi->cellSizeSpinBox->value());
    params.setCleanExisting    (mUi->cleanExistingCheckBox->isChecked());

}

CheckerboardDetectionParameters *CheckerboardDetectionParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    CheckerboardDetectionParameters *result = new CheckerboardDetectionParameters(
          static_cast<ImageChannel::ImageChannel>(mUi->channelComboBox->currentIndex())
        , mUi->vCrossesCountSpinBox->value()
        , mUi->hCrossesCountSpinBox->value()
        , mUi->cellSizeSpinBox->value()
        , mUi->cleanExistingCheckBox->isChecked()
    );
    return result;
}

void CheckerboardDetectionParametersControlWidget::setParameters(const CheckerboardDetectionParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->channelComboBox->setCurrentIndex(input.channel());
    mUi->vCrossesCountSpinBox->setValue(input.vCrossesCount());
    mUi->hCrossesCountSpinBox->setValue(input.hCrossesCount());
    mUi->cellSizeSpinBox->setValue(input.cellSize());
    mUi->cleanExistingCheckBox->setChecked(input.cleanExisting());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void CheckerboardDetectionParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    CheckerboardDetectionParameters *inputCasted = static_cast<CheckerboardDetectionParameters *>(input);
    setParameters(*inputCasted);
}
