/**
 * \file omnidirectionalBaseParametersControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "omnidirectionalBaseParametersControlWidget.h"
#include "ui_omnidirectionalBaseParametersControlWidget.h"
#include <memory>
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


OmnidirectionalBaseParametersControlWidget::OmnidirectionalBaseParametersControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::OmnidirectionalBaseParametersControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->principalXSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->principalYSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->focalSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->n, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->sizeXSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->sizeYSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->distortedSizeXSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->distortedSizeYSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
}

OmnidirectionalBaseParametersControlWidget::~OmnidirectionalBaseParametersControlWidget()
{

    delete mUi;
}

void OmnidirectionalBaseParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    std::unique_ptr<OmnidirectionalBaseParameters> params(createParameters());
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
}

void OmnidirectionalBaseParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    saver.saveParameters(*std::unique_ptr<OmnidirectionalBaseParameters>(createParameters()), rootPath);
}

void OmnidirectionalBaseParametersControlWidget::getParameters(OmnidirectionalBaseParameters& params) const
{
    params = *std::unique_ptr<OmnidirectionalBaseParameters>(createParameters());
}


OmnidirectionalBaseParameters *OmnidirectionalBaseParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    return new OmnidirectionalBaseParameters(
          mUi->principalXSpinBox->value()
        , mUi->principalYSpinBox->value()
        , mUi->focalSpinBox->value()
        , mUi->n->value()
        , mUi->sizeXSpinBox->value()
        , mUi->sizeYSpinBox->value()
        , mUi->distortedSizeXSpinBox->value()
        , mUi->distortedSizeYSpinBox->value()
    );
}

void OmnidirectionalBaseParametersControlWidget::setParameters(const OmnidirectionalBaseParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->principalXSpinBox->setValue(input.principalX());
    mUi->principalYSpinBox->setValue(input.principalY());
    mUi->focalSpinBox->setValue(input.focal());
    mUi->n->setValue(input.n());
    mUi->sizeXSpinBox->setValue(input.sizeX());
    mUi->sizeYSpinBox->setValue(input.sizeY());
    mUi->distortedSizeXSpinBox->setValue(input.distortedSizeX());
    mUi->distortedSizeYSpinBox->setValue(input.distortedSizeY());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void OmnidirectionalBaseParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    OmnidirectionalBaseParameters *inputCasted = static_cast<OmnidirectionalBaseParameters *>(input);
    setParameters(*inputCasted);
}
