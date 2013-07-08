#pragma once
#include <QtGui/QWidget>
#include "libElasParameters.h"
#include "ui_libElasParametersControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class LibElasParametersControlWidget;
}

class LibElasParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit LibElasParametersControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~LibElasParametersControlWidget();

    LibElasParameters* createParameters() const;
    void setParameters(const LibElasParameters &input);
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
    Ui_LibElasParametersControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

