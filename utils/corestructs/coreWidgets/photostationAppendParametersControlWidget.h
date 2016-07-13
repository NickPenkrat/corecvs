#pragma once
#include <QWidget>
#include "generated/photostationAppendParameters.h"
#include "ui_photostationAppendParametersControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class PhotostationAppendParametersControlWidget;
}

class PhotostationAppendParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit PhotostationAppendParametersControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~PhotostationAppendParametersControlWidget();

    PhotostationAppendParameters* createParameters() const;
    void getParameters(PhotostationAppendParameters &param) const;
    void setParameters(const PhotostationAppendParameters &input);
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
    Ui_PhotostationAppendParametersControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

