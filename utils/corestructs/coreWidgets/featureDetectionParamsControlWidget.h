#pragma once
#include <QWidget>
#include "generated/featureDetectionParams.h"
#include "ui_featureDetectionParamsControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class FeatureDetectionParamsControlWidget;
}

class FeatureDetectionParamsControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit FeatureDetectionParamsControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~FeatureDetectionParamsControlWidget();

    FeatureDetectionParams* createParameters() const;
    void getParameters(FeatureDetectionParams &param) const;
    void setParameters(const FeatureDetectionParams &input);
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
    Ui_FeatureDetectionParamsControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

