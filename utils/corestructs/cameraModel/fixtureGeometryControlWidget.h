#ifndef FIXTUREGEOMETRYCONTROLWIDGET_H
#define FIXTUREGEOMETRYCONTROLWIDGET_H

#include <QWidget>

#include "fixtureScenePart.h"
#include "parametersControlWidgetBase.h"


namespace Ui {
class FixtureGeometryControlWidget;
}

class FixtureGeometryControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit FixtureGeometryControlWidget(QWidget *parent = 0);
    ~FixtureGeometryControlWidget();

    corecvs::FlatPolygon* createParameters() const;
    void getParameters(corecvs::FlatPolygon &params) const;
    void setParameters(const corecvs::FlatPolygon &input);
    virtual void setParametersVirtual(void *input);


    virtual void loadParamWidget(WidgetLoader &/*loader*/);
    virtual void saveParamWidget(WidgetSaver  &/*saver*/ );

public slots:
    void paramsChangedInUI();

signals:
    void valueChanged();
    void paramsChanged();


private:
    Ui::FixtureGeometryControlWidget *ui;
};

#endif // FIXTUREGEOMETRYCONTROLWIDGET_H
