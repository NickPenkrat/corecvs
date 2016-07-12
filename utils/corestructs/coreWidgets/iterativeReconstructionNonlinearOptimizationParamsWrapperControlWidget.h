#pragma once
#include <QWidget>
#include "generated/iterativeReconstructionNonlinearOptimizationParamsWrapper.h"
#include "ui_iterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget;
}

class IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget();

    IterativeReconstructionNonlinearOptimizationParamsWrapper* createParameters() const;
    void getParameters(IterativeReconstructionNonlinearOptimizationParamsWrapper &param) const;
    void setParameters(const IterativeReconstructionNonlinearOptimizationParamsWrapper &input);
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
    Ui_IterativeReconstructionNonlinearOptimizationParamsWrapperControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

