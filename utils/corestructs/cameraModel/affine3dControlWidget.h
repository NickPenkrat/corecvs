#ifndef CAMERALOCATIONDATACONTROLWIDGET_H
#define CAMERALOCATIONDATACONTROLWIDGET_H

#include <QDialog>

#include "calibrationLocation.h"
#include "parametersControlWidgetBase.h"

namespace Ui {
class Affine3dControlWidget;
}

class Affine3dControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit Affine3dControlWidget(QWidget *parent = 0);
    ~Affine3dControlWidget();

    Affine3DQ* createParameters() const;
    void getParameters(Affine3DQ &params) const;
    void setParameters(const Affine3DQ &input);
    virtual void setParametersVirtual(void *input);

signals:
    void valueChanged();
    void paramsChanged();

private:
    Ui::Affine3dControlWidget *ui;
};

#endif // CAMERALOCATIONDATACONTROLWIDGET_H
