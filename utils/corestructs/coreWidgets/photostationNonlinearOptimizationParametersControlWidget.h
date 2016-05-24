#pragma once
#include <QWidget>
#include "generated/photostationNonlinearOptimizationParameters.h"
#include "ui_photostationNonlinearOptimizationParametersControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class PhotostationNonlinearOptimizationParametersControlWidget;
}

class PhotostationNonlinearOptimizationParametersControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit PhotostationNonlinearOptimizationParametersControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~PhotostationNonlinearOptimizationParametersControlWidget();

    PhotostationNonlinearOptimizationParameters* createParameters() const;
    void getParameters(PhotostationNonlinearOptimizationParameters &param) const;
    void setParameters(const PhotostationNonlinearOptimizationParameters &input);
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
    Ui_PhotostationNonlinearOptimizationParametersControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

