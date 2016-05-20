#pragma once
#include <QWidget>
#include "generated/photostationPlacerFeatureSelectionParameters.h"
#include "ui_photostationPlacerFeatureSelectionParametersControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class PhotostationPlacerFeatureSelectionParametersControlWidget;
}

class PhotostationPlacerFeatureSelectionParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit PhotostationPlacerFeatureSelectionParametersControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~PhotostationPlacerFeatureSelectionParametersControlWidget();

    PhotostationPlacerFeatureSelectionParameters* createParameters() const;
    void getParameters(PhotostationPlacerFeatureSelectionParameters &param) const;
    void setParameters(const PhotostationPlacerFeatureSelectionParameters &input);
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
    Ui_PhotostationPlacerFeatureSelectionParametersControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

