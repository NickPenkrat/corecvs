#include "fixtureGlobalParametersWidget.h"
#include "ui_fixtureGlobalParametersWidget.h"

FixtureGlobalParametersWidget::FixtureGlobalParametersWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FixtureGlobalParametersWidget)
{
    ui->setupUi(this);
    QObject::connect(ui->relativePathEdit           , SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->hasFinalCoordintesCheckBox , SIGNAL(toggled(bool))       , this, SIGNAL(paramsChanged()));

    QObject::connect(ui->replacePathPushButton, SIGNAL(released()), this, SLOT(replacePath()));
    QObject::connect(ui->changePathPushButton , SIGNAL(released()), this, SLOT(changePath()));
}

FixtureGlobalParametersWidget::~FixtureGlobalParametersWidget()
{
    delete ui;
}

void FixtureGlobalParametersWidget::setData(const FixtureScene *scene)
{
    ui->relativePathEdit->setText(QString::fromStdString(scene->getImageSearchPath()));
    ui->hasFinalCoordintesCheckBox->setEnabled(scene->hasTargetCoordSystem);
}

void FixtureGlobalParametersWidget::changePath()
{
   SYNC_PRINT(("FixtureGlobalParametersWidget::changePath()"));
}

void FixtureGlobalParametersWidget::replacePath()
{
    SYNC_PRINT(("FixtureGlobalParametersWidget::replacePath()"));
}
