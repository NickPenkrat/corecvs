#include "observationListModel.h"

/* Model */

ObservationListModel::ObservationListModel(QObject *parent) :
    QAbstractItemModel(parent),
    mObservationList(NULL)
{}

Qt::ItemFlags ObservationListModel::flags(const QModelIndex & /*index*/) const
{
    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable;
    return flags;
}

QVariant ObservationListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || mObservationList == NULL)
    {
        return QVariant();
    }

    if ((int)mObservationList->size() <= index.row())
    {
        return QVariant();
    }

    PointObservation &P = mObservationList->at(index.row());

    if (role == Qt::DisplayRole)
    {
        switch (index.column())
        {
        case COLUMN_X: return QString::number(P.x());
        case COLUMN_Y: return QString::number(P.y());
        case COLUMN_Z: return QString::number(P.z());
        case COLUMN_U: return QString::number(P.u());
        case COLUMN_V: return QString::number(P.v());
        default:
            break;
        }
    }

    if (role == Qt::EditRole)
    {
        switch (index.column())
        {
        case COLUMN_X: return P.x();
        case COLUMN_Y: return P.y();
        case COLUMN_Z: return P.z();
        case COLUMN_U: return P.u();
        case COLUMN_V: return P.v();
        default:
            break;
        }
    }

    if (role == Qt::ToolTipRole)
    {
        switch (index.column())
        {
        case COLUMN_X: return QString( "Space X" );
        case COLUMN_Y: return QString( "Space Y" );
        case COLUMN_Z: return QString( "Space Z" );
        case COLUMN_U: return QString( "Image X" );
        case COLUMN_V: return QString( "Image Y" );
        default:
            break;
        }
    }

    return QVariant();
}

bool ObservationListModel::setData(const QModelIndex &index, const QVariant &value, int /*role*/)
{
    if (!index.isValid())
    {
        return false;
    }
    PointObservation &P = mObservationList->at(index.row());

    bool ok = false;
    double newValue = value.toDouble(&ok);
    if (!ok) {
        return false;
    }

    switch (index.column())
    {
         case COLUMN_X: P.x() = newValue; break;
         case COLUMN_Y: P.y() = newValue; break;
         case COLUMN_Z: P.z() = newValue; break;
         case COLUMN_U: P.u() = newValue; break;
         case COLUMN_V: P.v() = newValue; break;
         default:
             return false;
    }

    emit dataChanged(index, index);
    return true;
}

int ObservationListModel::rowCount(const QModelIndex &/*parent*/) const
{
    return (mObservationList == NULL) ? 0 : (int)mObservationList->size();
}

int ObservationListModel::columnCount(const QModelIndex &/*parent*/) const
{
    return COLUMN_NUM;
}

bool ObservationListModel::insertRows(int row, int count, const QModelIndex &parent)
{
    if (mObservationList == NULL) {
        return false;
    }

    emit beginInsertRows(parent, row, row + count - 1);
    mObservationList->insert(mObservationList->begin() + row, count, PointObservation());
    emit endInsertRows();
    // FIXME: I just do not know what happens here, but without return statement
    //        it does not builds on windows
    return false;
}

bool ObservationListModel::removeRows(int row, int count, const QModelIndex &parent)
{
    if (mObservationList == NULL) {
        return false;
    }

    emit beginRemoveRows(parent, row, row + count - 1);
    mObservationList->erase(mObservationList->begin() + row, mObservationList->begin() + row + count);
    emit endRemoveRows();
    // FIXME: I just do not know what happens here, but without return statement
    //        it does not builds on windows
    return false;
}

QVariant ObservationListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if( role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal )
    {
        switch( section )
        {
        case COLUMN_X:
            return QString( "Space X" );
        case COLUMN_Y:
            return QString( "Space Y" );
        case COLUMN_Z:
            return QString( "Space Z" );

        case COLUMN_U:
            return QString( "Image X" );
        case COLUMN_V:
            return QString( "Image Y" );

        default:
            return QVariant();
        }
    }

    if (orientation == Qt::Vertical )
    {
        return QString::number(section + 1);
    }

    return QVariant();
}

QModelIndex ObservationListModel::index(int row, int column, const QModelIndex &/*parent*/) const
{
    if (mObservationList == NULL) {
        return QModelIndex();
    }

    if (row < (int)mObservationList->size() && column < COLUMN_NUM) {
        return createIndex(row, column);
    }
    return QModelIndex();
}

QModelIndex ObservationListModel::parent(const QModelIndex &/*index*/) const
{
    return QModelIndex();
}

void ObservationListModel::clearObservationPoints()
{
    emit beginResetModel();
    mObservationList->clear();
    emit endResetModel();
}

void ObservationListModel::setObservationList(ObservationList *observationList)
{
    emit beginResetModel();
    mObservationList = observationList;
    emit endResetModel();
}

int ObservationListModel::elementCount()
{
    if (mObservationList == NULL)
        return 0;

    return (int)mObservationList->size();
}

/*========= Model for a list of SceneObsertvation ======== */


CameraObserverListModel::CameraObserverListModel(QObject *parent) :
    QAbstractItemModel(parent),
    mObservationList(NULL)
{}

Qt::ItemFlags CameraObserverListModel::flags(const QModelIndex & /*index*/) const
{
    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable;
    return flags;
}

QVariant CameraObserverListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || mObservationList == NULL)
    {
        return QVariant();
    }

    if ((int)mObservationList->size() <= index.row())
    {
        return QVariant();
    }

    SceneObservation &P = *mObservationList->at(index.row());

    if (role == Qt::DisplayRole)
    {
        switch (index.column())
        {
        case COLUMN_X:    return QString::number(P.x());
        case COLUMN_Y:    return QString::number(P.y());
        case COLUMN_NAME: return QString::fromStdString(P.getPointName());
        default:
            break;
        }
    }

    if (role == Qt::EditRole)
    {
        switch (index.column())
        {
        case COLUMN_X: return P.x();
        case COLUMN_Y: return P.y();
        case COLUMN_NAME: return QString::fromStdString(P.getPointName());
        default:
            break;
        }
    }

    if (role == Qt::ToolTipRole)
    {
        switch (index.column())
        {
        case COLUMN_X:    return QString( "Image X" );
        case COLUMN_Y:    return QString( "Image Y" );
        case COLUMN_NAME: return QString( "Name" );
        default:
            break;
        }
    }

    return QVariant();
}

bool CameraObserverListModel::setData(const QModelIndex &index, const QVariant &value, int /*role*/)
{
    if (!index.isValid())
    {
        return false;
    }
    SceneObservation &P = *mObservationList->at(index.row());

    bool ok = false;
    double newValue = value.toDouble(&ok);
    if (!ok) {
        return false;
    }

    switch (index.column())
    {
         case COLUMN_X: P.x() = newValue; break;
         case COLUMN_Y: P.y() = newValue; break;
         case COLUMN_NAME:
         default:
             return false;
    }

    emit dataChanged(index, index);
    return true;
}

int CameraObserverListModel::rowCount(const QModelIndex &/*parent*/) const
{
    return (mObservationList == NULL) ? 0 : (int)mObservationList->size();
}

int CameraObserverListModel::columnCount(const QModelIndex &/*parent*/) const
{
    return COLUMN_NUM;
}

bool CameraObserverListModel::insertRows(int row, int count, const QModelIndex &parent)
{
    if (mObservationList == NULL) {
        return false;
    }

    emit beginInsertRows(parent, row, row + count - 1);
    /*TODO: Fix*/
    mObservationList->insert(mObservationList->begin() + row, count, new SceneObservation());
    emit endInsertRows();
    return true;
}

bool CameraObserverListModel::removeRows(int row, int count, const QModelIndex &parent)
{
    if (mObservationList == NULL) {
        return false;
    }

    emit beginRemoveRows(parent, row, row + count - 1);
    mObservationList->erase(mObservationList->begin() + row, mObservationList->begin() + row + count);
    emit endRemoveRows();
    return true;
}

QVariant CameraObserverListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if( role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal )
    {
        switch( section )
        {
        case COLUMN_X:
            return QString( "Image X" );
        case COLUMN_Y:
            return QString( "Image Y" );
        case COLUMN_NAME:
            return QString( "Name" );

        default:
            return QVariant();
        }
    }

    if (orientation == Qt::Vertical )
    {
        return QString::number(section + 1);
    }

    return QVariant();
}

QModelIndex CameraObserverListModel::index(int row, int column, const QModelIndex &/*parent*/) const
{
    if (mObservationList == NULL) {
        return QModelIndex();
    }

    if (row < (int)mObservationList->size() && column < COLUMN_NUM) {
        return createIndex(row, column);
    }
    return QModelIndex();
}

QModelIndex CameraObserverListModel::parent(const QModelIndex &/*index*/) const
{
    return QModelIndex();
}

void CameraObserverListModel::clearObservationPoints()
{
    emit beginResetModel();
    mObservationList->clear();
    emit endResetModel();
}

void CameraObserverListModel::setObservationList(vector<SceneObservation *> *observationList)
{
    emit beginResetModel();
    mObservationList = observationList;
    emit endResetModel();
}

int CameraObserverListModel::elementCount()
{
    if (mObservationList == NULL)
        return 0;

    return (int)mObservationList->size();
}




PointImageEditorInterfaceAbstractItemModelWrapper::PointImageEditorInterfaceAbstractItemModelWrapper(QAbstractItemModel *wrappee, int xColumn, int yColumn) :
    wrappee(wrappee),
    xColumn(xColumn),
    yColumn(yColumn)
{
    connect(wrappee, SIGNAL(dataChanged(QModelIndex,QModelIndex,QVector<int>)),
            this, SIGNAL(updateView()));

    connect(wrappee, SIGNAL(columnsInserted(QModelIndex,int,int))             , this, SIGNAL(modelInvalidated()));
    connect(wrappee, SIGNAL(columnsMoved(QModelIndex,int,int,QModelIndex,int)), this, SIGNAL(modelInvalidated()));
    connect(wrappee, SIGNAL(columnsRemoved(QModelIndex,int,int))              , this, SIGNAL(modelInvalidated()));

    connect(wrappee, SIGNAL(rowsInserted(QModelIndex,int,int))             , this, SIGNAL(modelInvalidated()));
    connect(wrappee, SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)), this, SIGNAL(modelInvalidated()));
    connect(wrappee, SIGNAL(rowsRemoved(QModelIndex,int,int))              , this, SIGNAL(modelInvalidated()));

    connect(wrappee, SIGNAL(modelReset())                                  , this, SIGNAL(modelInvalidated()));


}


size_t PointImageEditorInterfaceAbstractItemModelWrapper::getPointCount()
{
    QModelIndex topLevel = wrappee->parent(QModelIndex());
    return wrappee->rowCount(topLevel);
}

bool PointImageEditorInterfaceAbstractItemModelWrapper::appendPoint()
{
    wrappee->insertRow(wrappee->rowCount());
    return true;
}

bool PointImageEditorInterfaceAbstractItemModelWrapper::deletePoint(size_t id)
{
    wrappee->removeRow(id);
    return true;
}

QString PointImageEditorInterfaceAbstractItemModelWrapper::getMeta(size_t id)
{
    Vector2dd pos = getPoint(id);
    return QString("(%1, %2)").arg(pos.x()).arg(pos.y());
}

void PointImageEditorInterfaceAbstractItemModelWrapper::setPoint(size_t id, const Vector2dd &value)
{
    QModelIndex topLevel = wrappee->parent(QModelIndex());
    QModelIndex indexX = wrappee->index(id, xColumn, topLevel);
    QModelIndex indexY = wrappee->index(id, yColumn, topLevel);
    wrappee->setData(indexY, QVariant(value.y()), Qt::EditRole);
    wrappee->setData(indexX, QVariant(value.x()), Qt::EditRole);
}

Vector2dd PointImageEditorInterfaceAbstractItemModelWrapper::getPoint(size_t id)
{
    QModelIndex topLevel = wrappee->parent(QModelIndex());
    QModelIndex indexX   = wrappee->index(id, xColumn, topLevel);
    QModelIndex indexY   = wrappee->index(id, yColumn, topLevel);

    double x = wrappee->data(indexX).toDouble();
    double y = wrappee->data(indexY).toDouble();

    return Vector2dd(x,y);
}


