#include "manipulatorCaptureDialog.h"
#include "ui_manipulatorCaptureDialog.h"

#include <QMessageBox>

ManipulatorCaptureDialog::ManipulatorCaptureDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ManipulatorCaptureDialog)
{
    ui->setupUi(this);

    connect(ui->captureButton, SIGNAL(released()), this, SLOT(captureWithManipulator()));
    connect(ui->configureButton, SIGNAL(released()), this, SLOT(configureInterface()));
}

ManipulatorCaptureDialog::~ManipulatorCaptureDialog()
{
    delete ui;
}

void ManipulatorCaptureDialog::captureNextPosition()
{
    if(!mManipulator)
    {
        QMessageBox::critical(this, tr("Error"), tr("Manipulator failure"));
        return;
    }
    if(mPosition <=  ui->finalPositionBox->value())
    {
        if(mManipulator->moveToPosition(1))
        {
            emit captureAtPosition(mPosition);
            mPosition++;
        }
        else
        {
            toggleUIEnabled(true);
            mManipulator->moveToInitialPosition();
            mManipulator->disableManipulator();
            emit manipulatorCaptureFinalise(ui->advanceCheckBox->isChecked());
            QMessageBox::critical(this, tr("Error"), tr("Manipulator failure"));
        }
    }
    else
    {
        toggleUIEnabled(true);
        mManipulator->moveToInitialPosition();
        mManipulator->disableManipulator();
        emit manipulatorCaptureFinalise(ui->advanceCheckBox->isChecked());
    }
}

void ManipulatorCaptureDialog::setManipulator(AbstractManipulatorInterface *manipulator)
{
    mManipulator = manipulator;
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
    if(mManipulator && mManipulator->manipulatorGetReady())
    {
        toggleUIEnabled(false);
        mPosition = ui->initialPositionBox->value();
        mManipulator->moveToPosition(mPosition - 1);
        captureNextPosition();
    }
    else
    {
        QMessageBox::critical(this, tr("Error"), tr("Manipulator failure"));
    }
}

void ManipulatorCaptureDialog::configureInterface()
{
    if(mManipulator != nullptr)
        mManipulator->configureManipulator();
}
