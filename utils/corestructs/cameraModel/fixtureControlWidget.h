#ifndef FIXTURECONTROLWIDGET_H
#define FIXTURECONTROLWIDGET_H

#include <QWidget>
#include "affine3dControlWidget.h"

namespace Ui {
class FixtureControlWidget;
}

class FixtureControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FixtureControlWidget(QWidget *parent = 0);
    ~FixtureControlWidget();

#if 0
    CameraLocationData* createParameters() const;
    void getParameters(CameraLocationData &params) const;
    void setParameters(const CameraLocationData &input);
    virtual void setParametersVirtual(void *input);

signals:
    void valueChanged();
    void paramsChanged();
#endif

public slots:
    /** We should consider who is responsible for what **/
    void loadPressed();
    void savePressed();

signals:
    void loadRequest(QString filename);
    void saveRequest(QString filename);

public:
    Ui::FixtureControlWidget *ui;
    Affine3dControlWidget *getLocationWidget();
private:

};

#endif // FIXTURECONTROLWIDGET_H