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

void ObservationListModel::dataTemporaryInvalid()
{
    emit beginResetModel();
}

void ObservationListModel::dataIsValid()
{
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

