#pragma once
#include <QWidget>
#include "generated/photostationPlacerEssentialFilterParameters.h"
#include "ui_photostationPlacerEssentialFilterParametersControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class PhotostationPlacerEssentialFilterParametersControlWidget;
}

class PhotostationPlacerEssentialFilterParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit PhotostationPlacerEssentialFilterParametersControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~PhotostationPlacerEssentialFilterParametersControlWidget();

    PhotostationPlacerEssentialFilterParameters* createParameters() const;
    void getParameters(PhotostationPlacerEssentialFilterParameters &param) const;
    void setParameters(const PhotostationPlacerEssentialFilterParameters &input);
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
    Ui_PhotostationPlacerEssentialFilterParametersControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

