#pragma once
#include <QWidget>
#include "generated/reconstructionFunctorOptimizationType.h"
#include "ui_reconstructionFunctorOptimizationTypeControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class ReconstructionFunctorOptimizationTypeControlWidget;
}

class ReconstructionFunctorOptimizationTypeControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit ReconstructionFunctorOptimizationTypeControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~ReconstructionFunctorOptimizationTypeControlWidget();

    ReconstructionFunctorOptimizationType* createParameters() const;
    void getParameters(ReconstructionFunctorOptimizationType &param) const;
    void setParameters(const ReconstructionFunctorOptimizationType &input);
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
    Ui_ReconstructionFunctorOptimizationTypeControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

