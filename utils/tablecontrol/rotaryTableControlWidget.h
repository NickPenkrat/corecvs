#ifndef TABLECONTROLWIDGET_H
#define TABLECONTROLWIDGET_H

#include "calibrationCamera.h"
#include "rotationPlanGenerator.h"

#include <QWidget>
#include <QtSerialPort/QSerialPort>

namespace Ui {
class RotaryTableControlWidget;
}


class Mesh3DScene;

class RotaryTableControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RotaryTableControlWidget(QWidget *parent = 0);
    ~RotaryTableControlWidget();

    enum {
        COLUMN_AXIS_1,
        COLUMN_AXIS_2,
        COLUMN_AXIS_3,
        COLUMN_NUMBER
    };

    vector<CameraLocationAngles> positions;
    int selected;

    void loadCommands(QString filename);
    void saveCommands(QString filename);
    void updateTable ();

    RotationPlanGenerator generator;

    QSerialPort mPort;

public slots:
    void execute();
    void executeAndIncrement();


    void save();
    void load();
    void generate();

    void updateState();
    void selectedToCurrent();
    void tableCellClicked(int row, int column);
    void tableCellDoubleClicked(int row, int column);

    void newList(const vector<CameraLocationAngles> &input);


private:
    QSharedPointer<Mesh3DScene> scene;
    Ui::RotaryTableControlWidget *ui;
};

#endif // TABLECONTROLWIDGET_H
