#ifndef MANIPULATORCAPTUREDIALOG_H
#define MANIPULATORCAPTUREDIALOG_H

#include <QDialog>

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

signals:
    void captureAtPosition(int);
    void manipulatorCaptureFinalise(bool);

private:
    Ui::ManipulatorCaptureDialog *ui;
    int mPosition;

    void setupManipulator(int position);
    void toggleUIEnabled(bool enable);

private slots:
    void captureWithManipulator();
};

#endif // MANIPULATORCAPTUREDIALOG_H
