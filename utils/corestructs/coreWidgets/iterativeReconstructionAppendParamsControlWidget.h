#pragma once
#include <QWidget>
#include "generated/iterativeReconstructionAppendParams.h"
#include "ui_iterativeReconstructionAppendParamsControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class IterativeReconstructionAppendParamsControlWidget;
}

class IterativeReconstructionAppendParamsControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit IterativeReconstructionAppendParamsControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~IterativeReconstructionAppendParamsControlWidget();

    IterativeReconstructionAppendParams* createParameters() const;
    void getParameters(IterativeReconstructionAppendParams &param) const;
    void setParameters(const IterativeReconstructionAppendParams &input);
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
    Ui_IterativeReconstructionAppendParamsControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

