#include <QFileDialog>

#include "fixtureControlWidget.h"
#include "ui_fixtureControlWidget.h"

FixtureControlWidget::FixtureControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FixtureControlWidget)
{
    ui->setupUi(this);

    QObject::connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadPressed()));
    QObject::connect(ui->savePushButton, SIGNAL(released()), this, SLOT(savePressed()));

}

FixtureControlWidget::~FixtureControlWidget()
{
    delete ui;
}

Affine3dControlWidget *FixtureControlWidget::getLocationWidget()
{
    return ui->locationWidget;
}

void FixtureControlWidget::loadPressed()
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

void FixtureControlWidget::savePressed()
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

