#pragma once
#include <QWidget>
#include "generated/iterativeReconstructionNonlinearOptimizationParams.h"
#include "ui_iterativeReconstructionNonlinearOptimizationParamsControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class IterativeReconstructionNonlinearOptimizationParamsControlWidget;
}

class IterativeReconstructionNonlinearOptimizationParamsControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit IterativeReconstructionNonlinearOptimizationParamsControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~IterativeReconstructionNonlinearOptimizationParamsControlWidget();

    IterativeReconstructionNonlinearOptimizationParams* createParameters() const;
    void getParameters(IterativeReconstructionNonlinearOptimizationParams &param) const;
    void setParameters(const IterativeReconstructionNonlinearOptimizationParams &input);
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
    Ui_IterativeReconstructionNonlinearOptimizationParamsControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

