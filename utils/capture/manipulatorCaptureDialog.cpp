#include "manipulatorCaptureDialog.h"
#include "ui_manipulatorCaptureDialog.h"

ManipulatorCaptureDialog::ManipulatorCaptureDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ManipulatorCaptureDialog)
{
    ui->setupUi(this);

    connect(ui->captureButton, SIGNAL(released()), this, SLOT(captureWithManipulator()));
}

ManipulatorCaptureDialog::~ManipulatorCaptureDialog()
{
    delete ui;
}

void ManipulatorCaptureDialog::captureNextPosition()
{
    if(mPosition <=  ui->finalPositionBox->value())
    {
        setupManipulator(mPosition);
        emit captureAtPosition(mPosition);
        mPosition++;
    }
    else
    {
        toggleUIEnabled(true);
        emit manipulatorCaptureFinalise(ui->advanceCheckBox->isChecked());
    }
}

void ManipulatorCaptureDialog::setupManipulator(int position)
{
    //TODO implement manipulator setup here - wiil be called befor capturing farame
}

void ManipulatorCaptureDialog::toggleUIEnabled(bool enable)
{
    ui->advanceCheckBox->setDisabled(!enable);
    ui->captureButton->setDisabled(!enable);
    ui->finalPositionBox->setDisabled(!enable);
    ui->initialPositionBox->setDisabled(!enable);
    ui->stepBox->setDisabled((!enable));
}

void ManipulatorCaptureDialog::captureWithManipulator()
{
    toggleUIEnabled(false);
    mPosition = ui->initialPositionBox->value();
    captureNextPosition();
}
