#ifndef ABSTARCTMANIPULATORINTERFACE_H
#define ABSTARCTMANIPULATORINTERFACE_H

#include <QDialog>

class AbstractManipulatorInterface : public QDialog
{
    Q_OBJECT
public:
    explicit AbstractManipulatorInterface(QWidget *parent) : QDialog(parent) {}
public slots:
    virtual void captureNextSimulatedCamera() = 0;
    virtual void captureSimulatedStationFinshed(bool success = true) = 0;

signals:
    void captureAndFinalize(bool);
    void captureAtPosition(int);
    void manipulatorCaptureFinalise(bool);
};

/*class AbstractManipulatorInterface
{
public:
    virtual void configureManipulator() = 0;
    virtual bool disableManipulator() = 0;
    virtual bool enableManipulator() = 0;
    virtual bool manipulatorGetReady() = 0;
    virtual bool moveToPosition(int delta) = 0;
    virtual bool moveToInitialPosition() = 0;
};*/


#endif // ABSTARCTMANIPULATORINTERFACE_H
