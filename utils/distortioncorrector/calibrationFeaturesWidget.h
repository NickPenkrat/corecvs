#ifndef CALIBRATIONFEATURESWIDGET_H
#define CALIBRATIONFEATURESWIDGET_H

#include <QWidget>

#include "vector2d.h"
#include "vector3d.h"

using corecvs::Vector2dd;
using corecvs::Vector3dd;

/* Move this to separate class */
struct PointObservation
{
    Vector3dd point;
    Vector2dd projection;

    PointObservation(
            Vector3dd _point      = Vector3dd(0),
            Vector2dd _projection = Vector2dd(0)
    ) : point(_point),
        projection(_projection)
    {}


    inline double &x()
    {
        return point.x();
    }

    inline double &y()
    {
        return point.y();
    }

    inline double &z()
    {
        return point.z();
    }

    inline double &u()
    {
        return projection.x();
    }

    inline double &v()
    {
        return projection.y();
    }
};


class ObservationList : public std::vector<PointObservation>
{
    public:

};

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
