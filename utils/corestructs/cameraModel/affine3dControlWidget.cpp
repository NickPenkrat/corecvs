#include "affine3dControlWidget.h"
#include "ui_affine3dControlWidget.h"

#include <QClipboard>

Affine3dControlWidget::Affine3dControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::Affine3dControlWidget)
{
    ui->setupUi(this);

    QObject::connect(ui->spinBoxX, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->spinBoxY, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->spinBoxZ, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->widgetYaw,   SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->widgetPitch, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->widgetRoll,  SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->copyTextButton,  SIGNAL(released()), this, SLOT(copyText()));
}

Affine3dControlWidget::~Affine3dControlWidget()
{
    delete ui;
}

void Affine3dControlWidget::getParameters(Affine3DQ& params) const
{
    /**/
    params.shift.x() = ui->spinBoxX->value();
    params.shift.y() = ui->spinBoxY->value();
    params.shift.z() = ui->spinBoxZ->value();

    CameraLocationAngles angles;
    angles.setYaw  (ui->widgetYaw  ->value());
    angles.setPitch(ui->widgetPitch->value());
    angles.setRoll (ui->widgetRoll ->value());

    qDebug("Affine3dControlWidget::getParameters(): called" );
    angles.prettyPrint();
    params.rotor = angles.toQuaternion();
}

Affine3DQ *Affine3dControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/
    Affine3DQ *result = new Affine3DQ();
    getParameters(*result);
    return result;
}

void Affine3dControlWidget::setParameters(const Affine3DQ &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    ui->spinBoxX->setValue(input.shift.x());
    ui->spinBoxY->setValue(input.shift.y());
    ui->spinBoxZ->setValue(input.shift.z());

    CameraLocationAngles angles = CameraLocationAngles::FromQuaternion(input.rotor);

    ui->widgetYaw  ->setValue(angles.yaw  ());
    ui->widgetPitch->setValue(angles.pitch());
    ui->widgetRoll ->setValue(angles.roll ());

    blockSignals(wasBlocked);
    emit paramsChanged();
}

void Affine3dControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    Affine3DQ *inputCasted = static_cast<Affine3DQ *>(input);
    setParameters(*inputCasted);
}

void Affine3dControlWidget::copyText()
{
    SYNC_PRINT(("Affine3dControlWidget::copyText()\n"));
//    Affine3DQ params;
//    getParameters(params);
    QClipboard *clipboard = QApplication::clipboard();
    QString result = QString(
        "X = %1\n"
        "Y = %2\n"
        "Z = %3\n"
        "alpha = %4 °\n"
        "beta  = %5 °\n"
        "gamma = %6 °\n"
    ).arg(ui->spinBoxX->value()).arg(ui->spinBoxY->value()).arg(ui->spinBoxZ->value())
     .arg(radToDeg(ui->widgetYaw->value()))
     .arg(radToDeg(ui->widgetPitch->value()))
     .arg(radToDeg(ui->widgetRoll->value()));

    clipboard->setText(result);
    qDebug() << result;

}
