#include "shadedSceneControlWidget.h"
#include "ui_shadedSceneControlWidget.h"

ShadedSceneControlWidget::ShadedSceneControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ShadedSceneControlWidget)
{
    ui->setupUi(this);
}

ShadedSceneControlWidget::~ShadedSceneControlWidget()
{
    delete ui;
}
