#ifndef REFLECTIONWIDGET_H
#define REFLECTIONWIDGET_H

#include "parametersControlWidgetBase.h"

class ReflectionWidget : public ParametersControlWidgetBase
{
public:
    const corecvs::Reflection *reflection;

    ReflectionWidget(const Reflection *reflection);




    // ParametersControlWidgetBase interface
public:
    virtual BaseReflectionStatic *createParametersVirtual() const;

};

#endif // REFLECTIONWIDGET_H
