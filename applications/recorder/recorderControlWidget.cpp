/**
 * \file recorderControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "recorderControlWidget.h"
#include "ui_recorderControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


RecorderControlWidget::RecorderControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::RecorderControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->pathEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->fileTemplateEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
}

RecorderControlWidget::~RecorderControlWidget()
{

    delete mUi;
}

void RecorderControlWidget::loadParamWidget(WidgetLoader &loader)
{
    Recorder *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void RecorderControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    Recorder *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void RecorderControlWidget::getParameters(Recorder& params) const
{

    params.setPath             (mUi->pathEdit->text().toStdString());
    params.setFileTemplate     (mUi->fileTemplateEdit->text().toStdString());

}

Recorder *RecorderControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    Recorder *result = new Recorder(
          mUi->pathEdit->text().toStdString()
        , mUi->fileTemplateEdit->text().toStdString()
    );
    return result;
}

void RecorderControlWidget::setParameters(const Recorder &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->pathEdit->setText(input.path().c_str());
    mUi->fileTemplateEdit->setText(input.fileTemplate().c_str());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void RecorderControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    Recorder *inputCasted = static_cast<Recorder *>(input);
    setParameters(*inputCasted);
}
