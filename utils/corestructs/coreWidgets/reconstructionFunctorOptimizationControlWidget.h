#pragma once
#include <QWidget>
#include "generated/reconstructionFunctorOptimization.h"
#include "ui_reconstructionFunctorOptimizationControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class ReconstructionFunctorOptimizationControlWidget;
}

class ReconstructionFunctorOptimizationControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit ReconstructionFunctorOptimizationControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~ReconstructionFunctorOptimizationControlWidget();

    ReconstructionFunctorOptimization* createParameters() const;
    void getParameters(ReconstructionFunctorOptimization &param) const;
    void setParameters(const ReconstructionFunctorOptimization &input);
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
    Ui_ReconstructionFunctorOptimizationControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

