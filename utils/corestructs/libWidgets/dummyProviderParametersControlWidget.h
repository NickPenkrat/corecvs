#pragma once

#include <QtGui/QWidget>
#include <QtCore/QSettings>

#include "global.h"

#include "dummyProviderParameters.h"
#include "ui_dummyProviderParametersControlWidget.h"
#include "parametersControlWidgetBase.h"

namespace Ui {
    class DummyProviderParametersControlWidget;
}

class DummyProviderParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit DummyProviderParametersControlWidget(
            QWidget *parent = 0,
            bool autoInit = false,
            QString rootPath = QString()
            );
    ~DummyProviderParametersControlWidget();

    DummyProviderParameters* createParameters() const;
    void setParameters(const DummyProviderParameters &input);
    virtual void setParametersVirtual(void *input);


    virtual void loadParamWidget(WidgetLoader &loader);
    virtual void saveParamWidget(WidgetSaver  &saver);

public slots:
    void changeParameters()
    {
        // emit paramsChanged();
    }

signals:
    void valueChanged();
    void paramsChanged();

private:
    Ui_DummyProviderParametersControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

