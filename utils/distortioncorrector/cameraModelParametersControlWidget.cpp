#include "cameraModelParametersControlWidget.h"
#include "ui_cameraModelParametersControlWidget.h"

CameraModelParametersControlWidget::CameraModelParametersControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraModelParametersControlWidget)
{
    ui->setupUi(this);
}

CameraModelParametersControlWidget::~CameraModelParametersControlWidget()
{
    delete ui;
}
