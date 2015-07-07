/**
 * \file sobelParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "sobelParametersControlWidget.h"
#include "ui_sobelParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


SobelParametersControlWidget::SobelParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : FilterParametersControlWidgetBase(parent)
    , mUi(new Ui::SobelParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->mixingTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->horizontalCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->verticalCheckBox, SIGNAL(stateChanged(int)), this, SIGNAL(paramsChanged()));
}

SobelParametersControlWidget::~SobelParametersControlWidget()
{

    delete mUi;
}

void SobelParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    SobelParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void SobelParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    SobelParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void SobelParametersControlWidget::getParameters(SobelParameters& params) const
{

    params.setMixingType       (static_cast<SobelMixingType::SobelMixingType>(mUi->mixingTypeComboBox->currentIndex()));
    params.setHorizontal       (mUi->horizontalCheckBox->isChecked());
    params.setVertical         (mUi->verticalCheckBox->isChecked());

}

SobelParameters *SobelParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    SobelParameters *result = new SobelParameters(
          static_cast<SobelMixingType::SobelMixingType>(mUi->mixingTypeComboBox->currentIndex())
        , mUi->horizontalCheckBox->isChecked()
        , mUi->verticalCheckBox->isChecked()
    );
    return result;
}

void SobelParametersControlWidget::setParameters(const SobelParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->mixingTypeComboBox->setCurrentIndex(input.mixingType());
    mUi->horizontalCheckBox->setChecked(input.horizontal());
    mUi->verticalCheckBox->setChecked(input.vertical());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void SobelParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    SobelParameters *inputCasted = static_cast<SobelParameters *>(input);
    setParameters(*inputCasted);
}
