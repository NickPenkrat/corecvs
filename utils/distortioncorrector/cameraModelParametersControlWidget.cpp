#include <QFileDialog>

#include "cameraModelParametersControlWidget.h"
#include "ui_cameraModelParametersControlWidget.h"

CameraModelParametersControlWidget::CameraModelParametersControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::CameraModelParametersControlWidget)
{
    ui->setupUi(this);

    ui->lensDistortionWidget->toggleAdvanced(false);

    QObject::connect(ui->lensDistortionWidget, SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->extrinsicWidget     , SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));


    QObject::connect(ui->spinBoxFocalX, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->spinBoxFocalY, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->spinBoxCx, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->spinBoxCy, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->spinBoxSkew, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
//    writeUi();
    /* Addintional buttons */

    QObject::connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadPressed()));
    QObject::connect(ui->savePushButton, SIGNAL(released()), this, SLOT(savePressed()));

}

CameraModelParametersControlWidget::~CameraModelParametersControlWidget()
{
    delete_safe(ui);
}

LensDistortionModelParameters CameraModelParametersControlWidget::lensDistortionParameters()
{
    LensDistortionModelParameters result;
    ui->lensDistortionWidget->getParameters(result);
    return result;
}

void CameraModelParametersControlWidget::setLensDistortionParameters(const LensDistortionModelParameters &params)
{
    ui->lensDistortionWidget->setParameters(params);
}

void CameraModelParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    ui->lensDistortionWidget->loadParamWidget(loader);
   /**/
}


void CameraModelParametersControlWidget::saveParamWidget(WidgetSaver &saver)
{
    ui->lensDistortionWidget->saveParamWidget(saver);
    /**/
}

void CameraModelParametersControlWidget::loadPressed()
{
    QString filename = QFileDialog::getOpenFileName(
        this,
        "LOAD: Choose an camera config name",
        ".",
        "Text (*.json)"
    );
    if (!filename.isEmpty()) {
        emit loadRequest(filename);
    }
}

void CameraModelParametersControlWidget::savePressed()
{
    QString filename = QFileDialog::getSaveFileName(
        this,
        "SAVE: Choose an camera config name",
        ".",
        "Text (*.json)"
    );
    if (!filename.isEmpty()) {
        emit saveRequest(filename);
    }
}

/**/

void CameraModelParametersControlWidget::getParameters(CameraModel& params) const
{    
    ui->lensDistortionWidget->getParameters(params.distortion);

    Affine3DQ location;
    ui->extrinsicWidget->getParameters(location);
    params.setLocation(location);

    params.intrinsics.focal.x() = ui->spinBoxFocalX->value();
    params.intrinsics.focal.y() = ui->spinBoxFocalY->value();

    params.intrinsics.principal.x() = ui->spinBoxCx->value();
    params.intrinsics.principal.y() = ui->spinBoxCy->value();

    params.intrinsics.skew = ui->spinBoxSkew->value();
}

CameraModel *CameraModelParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/
    CameraModel *result = new CameraModel();
    getParameters(*result);
    return result;
}

void CameraModelParametersControlWidget::setParameters(const CameraModel &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    ui->lensDistortionWidget->setParameters(input.distortion);

    ui->extrinsicWidget->setParameters(input.getAffine());

    ui->spinBoxFocalX->setValue(input.intrinsics.fx());
    ui->spinBoxFocalY->setValue(input.intrinsics.fy());

    ui->spinBoxCx->setValue(input.intrinsics.cx());
    ui->spinBoxCy->setValue(input.intrinsics.cy());

    ui->spinBoxSkew->setValue(input.intrinsics.skew);

    blockSignals(wasBlocked);
    emit paramsChanged();
}

void CameraModelParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    CameraModel *inputCasted = static_cast<CameraModel *>(input);
    setParameters(*inputCasted);
}

#if 0

void CameraModelParametersControlWidget::getCameraParameters(double &fx, double &fy, double &cx, double &cy, double &skew, corecvs::Vector3dd &_pos, corecvs::Quaternion &_orientation, corecvs::Vector2dd &size, corecvs::Vector2dd &distortedSize)
{
    readUi();
    fx = this->fx;
    fy = this->fy;
    cx = this->cx;
    cy = this->cy;
    skew = this->skew;
    _pos = this->_pos;
    _orientation = this->_orientation;
    size = _size;
    distortedSize = _distortedSize;
}

void CameraModelParametersControlWidget::setCameraParameters(double &fx, double &fy, double &cx, double &cy, double &skew, corecvs::Vector3dd &_pos, corecvs::Quaternion &_dir, corecvs::Vector2dd &size, corecvs::Vector2dd &distortedSize)
{
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    this->skew = skew;
    this->_pos = _pos;
    this->_orientation = _dir;
    _size = size;
    _distortedSize = distortedSize;
    writeUi();
}

void CameraModelParametersControlWidget::readUi()
{
    fx = ui->spinBoxFocalX->value();
    fy = ui->spinBoxFocalY->value();
    cx = ui->spinBoxCx->value();
    cy = ui->spinBoxCy->value();
    skew = ui->spinBoxSkew->value();
    _pos[0] =  ui->spinBoxX->value();
    _pos[1] =  ui->spinBoxY->value();
    _pos[2] =  ui->spinBoxZ->value();

    double yaw = ui->widgetYaw->value() / 2.0, pitch = ui->widgetPitch->value() / 2.0, roll = ui->widgetRoll->value() / 2.0;
    // FIXME: I just selected arbitrary values
    double t = cos(roll)*cos(pitch)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    double x = sin(roll)*cos(pitch)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw);
    double y = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*cos(pitch)*sin(yaw);
    double z = cos(roll)*cos(pitch)*sin(yaw)-sin(roll)*sin(pitch)*cos(yaw);


    _orientation = Quaternion(x, y, z, t);
}

void CameraModelParametersControlWidget::writeUi()
{
    ui->spinBoxFocalX->setValue(fx);
    ui->spinBoxFocalY->setValue(fy);
    ui->spinBoxCx->setValue(cx);
    ui->spinBoxCy->setValue(cy);
    ui->spinBoxSkew->setValue(skew);
    ui->spinBoxX->setValue(_pos[0]);
    ui->spinBoxY->setValue(_pos[1]);
    ui->spinBoxZ->setValue(_pos[2]);
    // FIXME: hope it works
    double q0 = _orientation[3], q1 = _orientation[0], q2 = _orientation[1], q3 = _orientation[2];
    double roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    double pitch= asin(2.0 * (q0 * q2 - q3 * q1));
    double yaw  = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    ui->widgetYaw->setValue(yaw);
    ui->widgetPitch->setValue(pitch);
    ui->widgetRoll->setValue(roll);
}



#endif


