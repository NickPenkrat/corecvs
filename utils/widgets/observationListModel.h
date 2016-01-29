#ifndef OBSERVATION_LIST_MODEL_H
#define OBSERVATION_LIST_MODEL_H

#include <QAbstractItemModel>

#include "selectableGeometryFeatures.h"
#include "fixtureScene.h"

using corecvs::Vector2dd;
using corecvs::Vector3dd;
using corecvs::SelectableGeometryFeatures;
using corecvs::ObservationList;
using corecvs::PointObservation;


Q_DECLARE_METATYPE(ObservationList *);

class PointImageEditorInterface : public QObject
{
    Q_OBJECT
public:

    virtual size_t getPointCount() { return 0; }

    virtual Vector2dd getPoint(size_t id) = 0;
    virtual void setPoint(size_t id, const Vector2dd &value) = 0;

    virtual QString getMeta(size_t id) = 0;

    virtual bool deletePoint(size_t id) = 0;
    virtual bool appendPoint() = 0;

signals:
    void updateView();
    void modelInvalidated();
};

class PointImageEditorInterfaceAbstractItemModelWrapper : public PointImageEditorInterface
{
public:
    /* Should I use reference instead of pointer here*/
    QAbstractItemModel *wrappee;
    int xColumn;
    int yColumn;


    PointImageEditorInterfaceAbstractItemModelWrapper(
        QAbstractItemModel *wrappee, int xColumn, int yColumn);

    virtual size_t getPointCount() override;

    virtual Vector2dd getPoint(size_t id) override;
    virtual void setPoint(size_t id, const Vector2dd &value) override;

    virtual QString getMeta(size_t id) override;

    virtual bool deletePoint(size_t id) override;
    virtual bool appendPoint() override;


};



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
    virtual QVariant       data  (const QModelIndex &index, int role = Qt::DisplayRole) const;
    virtual bool        setData  (const QModelIndex &index, const QVariant &value, int role);

    virtual int         rowCount    (const QModelIndex &parent = QModelIndex()) const;
    virtual int         columnCount (const QModelIndex &parent = QModelIndex()) const;

    virtual bool        insertRows(int row, int count, const QModelIndex &parent = QModelIndex());
    virtual bool        removeRows(int row, int count, const QModelIndex &parent = QModelIndex());

    virtual QVariant    headerData(int section, Qt::Orientation orientation, int role ) const;

    virtual QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const;
    virtual QModelIndex parent(const QModelIndex &index) const;

public:
    void    clearObservationPoints();
    void    setObservationList(ObservationList *observationList);

    /* Some additional methods */
    int     elementCount();

//private:
    ObservationList *mObservationList;
};


/* Ok. So far a copy */
class CameraObserverListModel : public QAbstractItemModel
{
    Q_OBJECT

signals:

public:
    enum {
        COLUMN_X,
        COLUMN_Y,
        COLUMN_NAME,
        COLUMN_NUM
    };

    CameraObserverListModel(QObject *parent = NULL);

    virtual Qt::ItemFlags flags  (const QModelIndex &index) const;
    virtual QVariant       data  (const QModelIndex &index, int role = Qt::DisplayRole) const;
    virtual bool        setData  (const QModelIndex &index, const QVariant &value, int role);

    virtual int         rowCount    (const QModelIndex &parent = QModelIndex()) const;
    virtual int         columnCount (const QModelIndex &parent = QModelIndex()) const;

    virtual bool        insertRows(int row, int count, const QModelIndex &parent = QModelIndex());
    virtual bool        removeRows(int row, int count, const QModelIndex &parent = QModelIndex());

    virtual QVariant    headerData(int section, Qt::Orientation orientation, int role ) const;

    virtual QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const;
    virtual QModelIndex parent(const QModelIndex &index) const;

public:
    void    clearObservationPoints();
    void    setObservationList(vector<SceneObservation *> *observationList);

    /* Some additional methods */
    int     elementCount();

//private:
    vector<SceneObservation *> *mObservationList;
    //ObservationList *mObservationList;
};


#endif // OBSERVATION_LIST_MODEL_H
