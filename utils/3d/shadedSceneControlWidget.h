#ifndef SHADEDSCENECONTROLWIDGET_H
#define SHADEDSCENECONTROLWIDGET_H

#include "parametersControlWidgetBase.h"

#include <QWidget>

namespace Ui {
class ShadedSceneControlWidget;
}

class ShaderPreset {
public:
    QString name;
    enum ShaderType{
        PRESET,
        IMUTABLE,
        SAVEABLE
    };
    ShaderType type;

    QString vertex;
    QString fragment;

};

class ShadedSceneControlParameters : public BaseReflectionStatic{
public:
    ShaderPreset point;
    ShaderPreset edge;
    ShaderPreset face;
};

class ShadedSceneControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit ShadedSceneControlWidget(QWidget *parent = 0);
    ~ShadedSceneControlWidget();

    ShadedSceneControlParameters* createParameters() const;
    void getParameters(ShadedSceneControlParameters &params) const;
    void setParameters(const ShadedSceneControlParameters &input);
    virtual void setParametersVirtual(void *input);


    virtual BaseReflectionStatic *createParametersVirtual() const
    {
        return createParameters();
    }

private:
    Ui::ShadedSceneControlWidget *ui;
};

#endif // SHADEDSCENECONTROLWIDGET_H
