#include "shadedSceneControlWidget.h"
#include "ui_shadedSceneControlWidget.h"

ShadedSceneControlWidget::ShadedSceneControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::ShadedSceneControlWidget)
{
    ui->setupUi(this);
}

ShadedSceneControlWidget::~ShadedSceneControlWidget()
{
    delete ui;
}
