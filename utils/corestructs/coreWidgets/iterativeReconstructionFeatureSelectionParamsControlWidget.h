#pragma once
#include <QWidget>
#include "generated/iterativeReconstructionFeatureSelectionParams.h"
#include "ui_iterativeReconstructionFeatureSelectionParamsControlWidget.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
    class IterativeReconstructionFeatureSelectionParamsControlWidget;
}

class IterativeReconstructionFeatureSelectionParamsControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit IterativeReconstructionFeatureSelectionParamsControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~IterativeReconstructionFeatureSelectionParamsControlWidget();

    IterativeReconstructionFeatureSelectionParams* createParameters() const;
    void getParameters(IterativeReconstructionFeatureSelectionParams &param) const;
    void setParameters(const IterativeReconstructionFeatureSelectionParams &input);
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
    Ui_IterativeReconstructionFeatureSelectionParamsControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};

