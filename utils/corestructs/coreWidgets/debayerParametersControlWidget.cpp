/**
 * \file debayerParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "debayerParametersControlWidget.h"
#include "ui_debayerParametersControlWidget.h"
#include <memory>
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


DebayerParametersControlWidget::DebayerParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::DebayerParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->methodComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->bayerPosSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->numBitsOutSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->gains, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->gamma, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
}

DebayerParametersControlWidget::~DebayerParametersControlWidget()
{

    delete mUi;
}

void DebayerParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    std::unique_ptr<DebayerParameters> params(createParameters());
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
}

void DebayerParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    saver.saveParameters(*std::unique_ptr<DebayerParameters>(createParameters()), rootPath);
}

void DebayerParametersControlWidget::getParameters(DebayerParameters& params) const
{
    params = *std::unique_ptr<DebayerParameters>(createParameters());
}


DebayerParameters *DebayerParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    return new DebayerParameters(
          static_cast<DebayerMethod::DebayerMethod>(mUi->methodComboBox->currentIndex())
        , mUi->bayerPosSpinBox->value()
        , mUi->numBitsOutSpinBox->value()
        , mUi->gains->value()
        , mUi->gamma->value()
    );
}

void DebayerParametersControlWidget::setParameters(const DebayerParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->methodComboBox->setCurrentIndex(input.method());
    mUi->bayerPosSpinBox->setValue(input.bayerPos());
    mUi->numBitsOutSpinBox->setValue(input.numBitsOut());
    mUi->gains->setValue(input.gains());
    mUi->gamma->setValue(input.gamma());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void DebayerParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    DebayerParameters *inputCasted = static_cast<DebayerParameters *>(input);
    setParameters(*inputCasted);
}
