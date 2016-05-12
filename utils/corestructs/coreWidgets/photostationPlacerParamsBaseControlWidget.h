#pragma once
#include <QWidget>
#include "generated/photostationPlacerParamsBase.h"
#include "ui_photostationPlacerParamsBaseControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class PhotostationPlacerParamsBaseControlWidget;
}

class PhotostationPlacerParamsBaseControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit PhotostationPlacerParamsBaseControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~PhotostationPlacerParamsBaseControlWidget();

    PhotostationPlacerParamsBase* createParameters() const;
    void getParameters(PhotostationPlacerParamsBase &param) const;
    void setParameters(const PhotostationPlacerParamsBase &input);
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
    Ui_PhotostationPlacerParamsBaseControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

