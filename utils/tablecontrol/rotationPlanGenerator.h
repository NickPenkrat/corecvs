#ifndef ROTATIONPLANGENERATOR_H
#define ROTATIONPLANGENERATOR_H

#include <QWidget>
#include "calibrationCamera.h"

namespace Ui {
class RotationPlanGenerator;
}

class RotationPlanGenerator : public QWidget
{
    Q_OBJECT

public:
    explicit RotationPlanGenerator(QWidget *parent = 0);
    ~RotationPlanGenerator();

public slots:
    void generate();

signals:
    void newListGenerated(vector<CameraLocationAngles> output);

private:
    Ui::RotationPlanGenerator *ui;
};

#endif // ROTATIONPLANGENERATOR_H
