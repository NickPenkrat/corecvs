#pragma once
#include <QWidget>
#include "generated/reconstructionFunctorOptimizationParams.h"
#include "ui_reconstructionFunctorOptimizationParamsControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class ReconstructionFunctorOptimizationParamsControlWidget;
}

class ReconstructionFunctorOptimizationParamsControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit ReconstructionFunctorOptimizationParamsControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~ReconstructionFunctorOptimizationParamsControlWidget();

    ReconstructionFunctorOptimizationParams* createParameters() const;
    void getParameters(ReconstructionFunctorOptimizationParams &param) const;
    void setParameters(const ReconstructionFunctorOptimizationParams &input);
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
    Ui_ReconstructionFunctorOptimizationParamsControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

