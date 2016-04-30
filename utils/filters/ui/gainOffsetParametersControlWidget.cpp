/**
 * \file gainOffsetParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "gainOffsetParametersControlWidget.h"
#include "ui_gainOffsetParametersControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


GainOffsetParametersControlWidget::GainOffsetParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : FilterParametersControlWidgetBase(parent)
    , mUi(new Ui::GainOffsetParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->gainSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->offsetSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

GainOffsetParametersControlWidget::~GainOffsetParametersControlWidget()
{

    delete mUi;
}

void GainOffsetParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    GainOffsetParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void GainOffsetParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    GainOffsetParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void GainOffsetParametersControlWidget::getParameters(GainOffsetParameters& params) const
{

    params.setGain             (mUi->gainSpinBox->value());
    params.setOffset           (mUi->offsetSpinBox->value());

}

GainOffsetParameters *GainOffsetParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    GainOffsetParameters *result = new GainOffsetParameters(
          mUi->gainSpinBox->value()
        , mUi->offsetSpinBox->value()
    );
    return result;
}

void GainOffsetParametersControlWidget::setParameters(const GainOffsetParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->gainSpinBox->setValue(input.gain());
    mUi->offsetSpinBox->setValue(input.offset());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void GainOffsetParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    GainOffsetParameters *inputCasted = static_cast<GainOffsetParameters *>(input);
    setParameters(*inputCasted);
}
