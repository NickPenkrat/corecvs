#include "cameraModelParametersControlWidget.h"
#include "ui_cameraModelParametersControlWidget.h"

CameraModelParametersControlWidget::CameraModelParametersControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::CameraModelParametersControlWidget)
{
    ui->setupUi(this);
    writeUi();
}

CameraModelParametersControlWidget::~CameraModelParametersControlWidget()
{
    delete ui;
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


}

void CameraModelParametersControlWidget::getCameraParameters(double &fx, double &fy, double &cx, double &cy, double &skew, corecvs::Vector3dd &_pos, corecvs::Quaternion &_orientation)
{
    readUi();
    fx = this->fx;
    fy = this->fy;
    cx = this->cx;
    cy = this->cy;
    skew = this->skew;
    _pos = this->_pos;
    _orientation = this->_orientation;
}

void CameraModelParametersControlWidget::setCameraParameters(double &fx, double &fy, double &cx, double &cy, double &skew, corecvs::Vector3dd &_pos, corecvs::Quaternion &_dir)
{
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    this->skew = skew;
    this->_pos = _pos;
    this->_orientation = _dir;
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

    double yaw = ui->widgetYaw->value(), pitch = ui->widgetPitch->value(), roll = ui->widgetRoll->value();
    // FIXME: I just selected arbitrary values
    corecvs::Matrix33 R = Matrix33::RotationX(roll) * Matrix33::RotationY(pitch) * Matrix33::RotationZ(yaw);
    _orientation = Quaternion::FromMatrix(R);
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


void CameraModelParametersControlWidget::saveParamWidget(WidgetSaver &saver)
{
    ui->lensDistortionWidget->saveParamWidget(saver);


}
