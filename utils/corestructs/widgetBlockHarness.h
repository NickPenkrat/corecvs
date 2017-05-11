#ifndef WIDGETBLOCKHARNESS_H
#define WIDGETBLOCKHARNESS_H

#include <QWidget>

#include "newStyleBlock.h"
#include "reflection.h"
#include "reflectionWidget.h"

class WidgetBlockHarness : public QWidget
{
    Q_OBJECT
public:
    corecvs::DynamicObject *blockReflection = NULL;
    NewStyleBlock *block = NULL;


    ReflectionWidget *inputs = NULL;
    ReflectionWidget *params = NULL;
    ReflectionWidget *outputs = NULL;


    WidgetBlockHarness(corecvs::DynamicObject *blockReflection, NewStyleBlock *block, QWidget *parent = NULL);

public slots:
    void executeCalled();
public:
    virtual ~WidgetBlockHarness();

};

#endif // WIDGETBLOCKHARNESS_H
