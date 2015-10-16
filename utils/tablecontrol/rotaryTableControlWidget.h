#ifndef TABLECONTROLWIDGET_H
#define TABLECONTROLWIDGET_H

#include "calibrationCamera.h"

#include <QWidget>

namespace Ui {
class TableControlWidget;
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
    void loadCommands ();
    void updateTable ();

public slots:
    void updateState();
    void tableCellClicked(int row, int column);

private:
    QSharedPointer<Mesh3DScene> scene;
    Ui::TableControlWidget *ui;
};

#endif // TABLECONTROLWIDGET_H
