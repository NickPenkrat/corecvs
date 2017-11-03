#include "rotationPlanGenerator.h"
#include "ui_rotationPlanGenerator.h"
#include "core/math/mathUtils.h"
#include "core/utils/log.h"

using namespace corecvs;

RotationPlanGenerator::RotationPlanGenerator(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::RotationPlanGenerator)
{
    ui->setupUi(this);

    ui->widgetYawMin->setValue(0);
    ui->widgetYawMax->setValue(0);

    //cint pitchNum = ui->stepPitchSpinBox->value();                // =6:  -30, -15, 0, 15, 30, 45
    ui->widgetPitchMin->setValue(degToRad(-30));
    ui->widgetPitchMax->setValue(degToRad(+45));

    cint rollNum = ui->stepRollSpinBox->value();                    // =24:  0, 15, 30, ..., 345
    ui->widgetRollMin->setValue(0.0);
    ui->widgetRollMax->setValue(degToRad(360 - 360. / rollNum));

    connect(ui->generatePushButton, SIGNAL(released()), this, SLOT(generate()));
}

RotationPlanGenerator::~RotationPlanGenerator()
{
    delete ui;
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
        for (int ir = 0; ir < nr; ir++)                     // roll position of prototype with further iterations by pitch position
        {
            // implement the next iteration from the pitch position from previous iteration
            int ipStrt = (ir & 1) ? np - 1 : 0;
            int ipStop = (ir & 1) ?      0 : np - 1;
            int ipStep = (ir & 1) ?     -1 : 1;

            for (int ip = ipStrt; ip >= CORE_MIN(ipStrt, ipStop) && ip <= CORE_MAX(ipStrt, ipStop); ip += ipStep)
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
