#include "cameraModelParametersControlWidget.h"
#include "ui_cameraModelParametersControlWidget.h"

CameraModelParametersControlWidget::CameraModelParametersControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::CameraModelParametersControlWidget)
{
    ui->setupUi(this);
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

void CameraModelParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    ui->lensDistortionWidget->loadParamWidget(loader);
}

void CameraModelParametersControlWidget::saveParamWidget(WidgetSaver &saver)
{
    ui->lensDistortionWidget->saveParamWidget(saver);
}
