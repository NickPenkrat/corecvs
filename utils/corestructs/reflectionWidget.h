#ifndef REFLECTIONWIDGET_H
#define REFLECTIONWIDGET_H

#include <vector>

#include "parametersControlWidgetBase.h"

class ReflectionWidget : public ParametersControlWidgetBase
{
protected:
    const corecvs::Reflection *reflection;
    std::vector<QWidget *> fieldToWidget;

public:
    ReflectionWidget(const Reflection *reflection);


    // ParametersControlWidgetBase interface
public:
    //virtual BaseReflectionStatic *createParametersVirtual() const;
    bool getParameters(void *param) const;
    bool setParameters(void *param) const;


};

#endif // REFLECTIONWIDGET_H
