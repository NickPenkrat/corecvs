#include "rotationPlanGenerator.h"
#include "ui_rotationPlanGenerator.h"

RotationPlanGenerator::RotationPlanGenerator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RotationPlanGenerator)
{
    ui->setupUi(this);

    cint pitchNum = 24;

    ui->widgetYawMin->setValue(0);
    ui->widgetYawMax->setValue(degToRad(360 - 360 / pitchNum));

    ui->widgetPitchMin->setValue(0.0);
    ui->widgetPitchMax->setValue(0.0);

    ui->widgetRollMin->setValue(-M_PI / pitchNum);
    ui->widgetRollMax->setValue( M_PI / pitchNum);

    connect(ui->generatePushButton, SIGNAL(released()), this, SLOT(generate()));
}

double intrpolate(double min, double max, int val, int maxval)
{
    if (maxval <= 1) return (min + max) / 2.0;

    if (val <       0) return min;
    if (val >= maxval) return max;

    return min + ((max - min) * val / (maxval - 1));
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
        for (int ip = 0; ip < np; ip++)
        {
            for (int ir = 0; ir < nr; ir++)
            {
                positions.push_back(CameraLocationAngles(
                    intrpolate(miny, maxy, iy, ny),
                    intrpolate(minp, maxp, ip, np),
                    intrpolate(minr, maxr, ir, nr)
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
