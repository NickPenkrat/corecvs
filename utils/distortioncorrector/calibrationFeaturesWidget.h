#ifndef CALIBRATIONFEATURESWIDGET_H
#define CALIBRATIONFEATURESWIDGET_H

#include <QWidget>
#include <QAbstractItemModel>

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

class ObservationListModel : public QAbstractItemModel
{
    Q_OBJECT

signals:


public:

    enum {
        COLUMN_X,
        COLUMN_Y,
        COLUMN_Z,
        COLUMN_U,
        COLUMN_V,
        COLUMN_NUM
    };

    ObservationListModel(QObject *parent = NULL);

    virtual Qt::ItemFlags flags  (const QModelIndex &index) const;
    virtual QVariant       data  ( const QModelIndex & index, int role = Qt::DisplayRole ) const;
    virtual bool        setData  (const QModelIndex &index, const QVariant &value, int role);

    virtual int rowCount    (const QModelIndex &parent) const;
    virtual int columnCount (const QModelIndex &parent) const;

    virtual bool insertRows(int row, int count, const QModelIndex &parent);
    virtual bool removeRows(int row, int count, const QModelIndex &parent);

    virtual QVariant headerData(int section, Qt::Orientation orientation, int role ) const;

    virtual QModelIndex index(int row, int column, const QModelIndex &parent ) const;
    virtual QModelIndex parent(const QModelIndex &index) const;

public:

    void clearObservationPoints();
    void setObservationList(ObservationList *observationList);

//private:

    ObservationList *mObservationList;

};


class CalibrationFeaturesWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CalibrationFeaturesWidget(QWidget *parent = 0);
    ~CalibrationFeaturesWidget();

    static const int REASONABLE_INF;

    enum {
        COLUMN_X,
        COLUMN_Y,
        COLUMN_Z,
        COLUMN_U,
        COLUMN_V
    };


    ObservationListModel *mObservationListModel;
    SelectableGeometryFeatures *geometryFeatures;


    void manualAddPoint(const Vector2dd &point);
    void setObservationModel(ObservationListModel *observationListModel);

    void savePoints();
    void loadPoints();

public slots:
    void addPoint();
    void removePoint();
    void deleteObservation();


    void updateWidget();

signals:
    void dataUpdated();

private:
    Ui::CalibrationFeaturesWidget *ui;
};

#endif // CALIBRATIONFEATURESWIDGET_H
