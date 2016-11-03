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
        NONE,
        PRESET,
        IMUTABLE,
        SAVEABLE
    };
    ShaderType type = ShaderPreset::PRESET;

    QString vertex;
    QString fragment;

};

class ShadedSceneControlParameters : public BaseReflectionStatic{
public:

    ShadedSceneControlParameters()
    {
        point.type = ShaderPreset::PRESET;
        edge.type = ShaderPreset::PRESET;
        face.type = ShaderPreset::PRESET;
    }

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

signals:
    void paramsChanged();

};

#endif // SHADEDSCENECONTROLWIDGET_H
