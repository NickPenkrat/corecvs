#pragma once
#include <QWidget>
#include "generated/iterativeReconstructionInitializationParams.h"
#include "ui_iterativeReconstructionInitializationParamsControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class IterativeReconstructionInitializationParamsControlWidget;
}

class IterativeReconstructionInitializationParamsControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit IterativeReconstructionInitializationParamsControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~IterativeReconstructionInitializationParamsControlWidget();

    IterativeReconstructionInitializationParams* createParameters() const;
    void getParameters(IterativeReconstructionInitializationParams &param) const;
    void setParameters(const IterativeReconstructionInitializationParams &input);
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
    Ui_IterativeReconstructionInitializationParamsControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

