#ifndef MANIPULATORCAPTUREDIALOG_H
#define MANIPULATORCAPTUREDIALOG_H

#include <QDialog>

#include "abstractManipulatorInterface.h"

namespace Ui {
class ManipulatorCaptureDialog;
}

class ManipulatorCaptureDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ManipulatorCaptureDialog(QWidget *parent = 0);
    ~ManipulatorCaptureDialog();

    void captureNextPosition();
    void setManipulator(AbstractManipulatorInterface *manipulator);

signals:
    void captureAtPosition(int);
    void manipulatorCaptureFinalise(bool);

private:
    Ui::ManipulatorCaptureDialog *ui;
    int mPosition;
    AbstractManipulatorInterface *mManipulator;

    void toggleUIEnabled(bool enable);

private slots:
    void captureWithManipulator();
    void configureInterface();
};

#endif // MANIPULATORCAPTUREDIALOG_H
