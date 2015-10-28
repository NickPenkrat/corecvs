#include "rotationPlanGenerator.h"
#include "ui_rotationPlanGenerator.h"

RotationPlanGenerator::RotationPlanGenerator(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::RotationPlanGenerator)
{
    ui->setupUi(this);

    ui->widgetYawMin->setValue(0);
    ui->widgetYawMax->setValue(0);

    //cint pitchNum = ui->stepPitchSpinBox->value();  // =5
    ui->widgetPitchMin->setValue(degToRad(-40));
    ui->widgetPitchMax->setValue(degToRad(+40));

    cint rollNum = ui->stepRollSpinBox->value();    // =24
    ui->widgetRollMin->setValue(0.0);
    ui->widgetRollMax->setValue(degToRad(360 - 360. / rollNum));

    connect(ui->generatePushButton, SIGNAL(released()), this, SLOT(generate()));
}

static double interpolate(double min, double max, int index, int maxIndex)
{
    if (maxIndex <= 1)      return (min + max) / 2.0;

    if (index <         0)  return min;
    if (index >= maxIndex)  return max;

    return min + ((max - min) * index / (maxIndex - 1));
}

void RotationPlanGenerator::generate()
{
    vector<CameraLocationAngles> positions;

    int ny = ui->stepYawSpinBox  ->value();
    int np = ui->stepPitchSpinBox->value();
    int nr = ui->stepRollSpinBox ->value();

    double miny = ui->widgetYawMin  ->value();
    double minp = ui->widgetPitchMin->value();
    double minr = ui->widgetRollMin ->value();

    double maxy = ui->widgetYawMax  ->value();
    double maxp = ui->widgetPitchMax->value();
    double maxr = ui->widgetRollMax ->value();

    positions.reserve(ny * np * nr);

    for (int iy = 0; iy < ny; iy++)
    {
        for (int ir = 0; ir < nr; ir++)
        {
            for (int ip = 0; ip < np; ip++)
            {
                positions.push_back(CameraLocationAngles(
                    interpolate(miny, maxy, iy, ny),
                    interpolate(minp, maxp, ip, np),
                    interpolate(minr, maxr, ir, nr)
                ));
            }
        }
    }

    emit newListGenerated(positions);
}

RotationPlanGenerator::~RotationPlanGenerator()
{
    delete ui;
}
