#ifndef CALIBRATIONFEATURESWIDGET_H
#define CALIBRATIONFEATURESWIDGET_H

#include <QWidget>

#include "vector2d.h"
#include "vector3d.h"
#include "calibrationFeaturesWidget.h"
#include "selectableGeometryFeatures.h"

using corecvs::Vector2dd;
using corecvs::Vector3dd;
using corecvs::SelectableGeometryFeatures;
using corecvs::ObservationList;
using corecvs::PointObservation;



namespace Ui {
class CalibrationFeaturesWidget;
}

class CalibrationFeaturesWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CalibrationFeaturesWidget(QWidget *parent = 0);
    ~CalibrationFeaturesWidget();

    enum {
        COLUMN_X,
        COLUMN_Y,
        COLUMN_Z,
        COLUMN_U,
        COLUMN_V
    };


    ObservationList observationList;
    SelectableGeometryFeatures *geometryFeatures;


    void manualAddPoint(const Vector2dd &point);
    void clearObservationPoints();
    void addObservation(PointObservation &observation);

    void savePoints();
    void loadPoints();

public slots:
    void choosePoint(int row, int column);
    void addVector();
    void deleteObservation();


    void updateWidget();

signals:
    void dataUpdated();

private:
    Ui::CalibrationFeaturesWidget *ui;
};

#endif // CALIBRATIONFEATURESWIDGET_H
