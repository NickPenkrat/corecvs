#include "pointListEditImageWidget.h"
#include "painterHelpers.h"
#include "qtHelper.h"

PointListEditImageWidget::PointListEditImageWidget(QWidget *parent, bool showHeader) :
    AdvancedImageWidget(parent, showHeader),
    mObservationListModel(NULL),
    mSelectedPoint(-1)
{
    mMoveButton    = addToolButton("Select",            QIcon(":/new/prefix1/select_by_color.png"));
    mAddButton     = addToolButton("Add Point To Path", QIcon(":/new/prefix1/vector_add.png"     ), false);
    mDeleteButton  = addToolButton("Delete Point",      QIcon(":/new/prefix1/vector_delete.png"  ), false);
    mAddInfoButton = addToolButton("Toggle info",       QIcon(":/new/prefix1/info_rhombus.png"   ), false);
    mAddInfoButton ->setCheckable(true);
}

void PointListEditImageWidget::setObservationModel(ObservationListModel *observationListModel)
{
    disconnect(mObservationListModel, 0, this, 0);

    mObservationListModel = observationListModel;

    connect(mObservationListModel, SIGNAL(dataChanged(QModelIndex,QModelIndex,QVector<int>)),
            this, SLOT(update()));

    connect(mObservationListModel, SIGNAL(columnsInserted(QModelIndex,int,int))             , this, SLOT(invalidateModel()));
    connect(mObservationListModel, SIGNAL(columnsMoved(QModelIndex,int,int,QModelIndex,int)), this, SLOT(invalidateModel()));
    connect(mObservationListModel, SIGNAL(columnsRemoved(QModelIndex,int,int))              , this, SLOT(invalidateModel()));

    connect(mObservationListModel, SIGNAL(rowsInserted(QModelIndex,int,int))             , this, SLOT(invalidateModel()));
    connect(mObservationListModel, SIGNAL(rowsMoved(QModelIndex,int,int,QModelIndex,int)), this, SLOT(invalidateModel()));
    connect(mObservationListModel, SIGNAL(rowsRemoved(QModelIndex,int,int))              , this, SLOT(invalidateModel()));

    connect(mObservationListModel, SIGNAL(modelReset())                                     , this, SLOT(invalidateModel()));
}

/* This is called when model indexes are changed, and our cache is no longer valid */
void PointListEditImageWidget::invalidateModel()
{
    mSelectedPoint = -1;
    update();
}

void PointListEditImageWidget::childRepaint(QPaintEvent *event, QWidget *who)
{
    AdvancedImageWidget::childRepaint(event, who);
    if (mImage.isNull())
    {
        return;
    }

    if (mObservationListModel == NULL)
    {
        return;
    }

    /* Now the points */
    QPainter painter(who);

    QModelIndex topLevel = mObservationListModel->parent(QModelIndex());

    int rows = mObservationListModel->rowCount(topLevel);

    for (int i = 0; i < rows; i ++)
    {
        Vector2dd point = getPointById(i);
        Vector2dd imageCoords = imageToWidgetF(point);
        painter.setPen(Qt::yellow);
        drawCircle(painter, imageCoords, 5);
        painter.setPen(Qt::blue);
        drawCircle(painter, imageCoords, 10);

        if (mAddInfoButton->isChecked())
        {
            QString meta = getMetaById(i);
            QPointF pos = Core2Qt::QPointFromVector2dd(imageCoords + Vector2dd(5, -10));
            painter.setPen(Qt::black);
            painter.drawText(pos, meta);
            pos += QPointF(1,1);
            painter.setPen(Qt::white);
            painter.drawText(pos, meta);
        }

        if (i == mSelectedPoint) {
            painter.setPen(Qt::red);
            drawCircle(painter, imageCoords, 7);

            /* Test it a bit and use QSelectionModel */
            imageCoords = imageToWidgetF(widgetToImageF(imageCoords));
            painter.setPen(Qt::cyan);
            drawCircle(painter, imageCoords, 3);
        }
    }
}


void PointListEditImageWidget::toolButtonReleased(QWidget *button)
{

    mUi->widget->unsetCursor();
    AdvancedImageWidget::toolButtonReleased(button);

    if (button == mMoveButton)
    {
        qDebug() << "Move Button";
        mCurrentToolClass = (ToolClass)MOVE_POINT_TOOL;
        mUi->widget->setCursor(Qt::ArrowCursor);
    }
    else if (button == mAddButton)
    {
        qDebug() << "Add Button";
        //mCurrentToolClass = (ToolClass)ADD_POINT_TOOL;
        mObservationListModel->insertRow(mObservationListModel->rowCount());
        mSelectedPoint = mObservationListModel->rowCount() - 1;

        QModelIndex indexX = mObservationListModel->index(mSelectedPoint, ObservationListModel::COLUMN_U);
        QModelIndex indexY = mObservationListModel->index(mSelectedPoint, ObservationListModel::COLUMN_V);

        mObservationListModel->setData(indexY, QVariant(mZoomCenter.y()), Qt::EditRole);
        mObservationListModel->setData(indexX, QVariant(mZoomCenter.x()), Qt::EditRole);
        mUi->widget->update();
    }
    else if (button == mDeleteButton)
    {
        qDebug() << "Delete Button";
        if (mSelectedPoint >= 0)
        {
            mObservationListModel->removeRow(mSelectedPoint);
            mSelectedPoint = -1;
        }
        mUi->widget->update();
    } else if (button == mAddInfoButton)
    {
        mUi->widget->update();
    }
}

Vector2dd PointListEditImageWidget::getPointById(int row)
{
    QModelIndex topLevel = mObservationListModel->parent(QModelIndex());
    QModelIndex indexX = mObservationListModel->index(row, ObservationListModel::COLUMN_U, topLevel);
    QModelIndex indexY = mObservationListModel->index(row, ObservationListModel::COLUMN_V, topLevel);

    double x = mObservationListModel->data(indexX).toDouble();
    double y = mObservationListModel->data(indexY).toDouble();

    return Vector2dd(x,y);
}

QString PointListEditImageWidget::getMetaById(int row)
{
    QModelIndex topLevel = mObservationListModel->parent(QModelIndex());
    QModelIndex indexX = mObservationListModel->index(row, ObservationListModel::COLUMN_X, topLevel);
    QModelIndex indexY = mObservationListModel->index(row, ObservationListModel::COLUMN_Y, topLevel);
    QModelIndex indexZ = mObservationListModel->index(row, ObservationListModel::COLUMN_Z, topLevel);


    double x = mObservationListModel->data(indexX).toDouble();
    double y = mObservationListModel->data(indexY).toDouble();
    double z = mObservationListModel->data(indexZ).toDouble();

    return QString("%1 [%2 %3 %4]").arg(row + 1).arg(x).arg(y).arg(z);
}

int PointListEditImageWidget::findClosest(Vector2dd imagePoint, double limitDistance )
{
    QModelIndex topLevel = mObservationListModel->parent(QModelIndex());
    int rows = mObservationListModel->rowCount(topLevel);

    double bestDistance = limitDistance;
    int bestIndex = -1;

    for (int i = 0; i < rows; i ++)
    {
        Vector2dd currentPoint = getPointById(i);
        double dist = (imagePoint - currentPoint).l2Metric();
        if (dist < bestDistance) {
            bestDistance = dist;
            bestIndex = i;
        }
    }

    return bestIndex;
}

void PointListEditImageWidget::childMousePressed(QMouseEvent *event)
{
    AdvancedImageWidget::childMousePressed(event);

    PaintToolClass tool = (PaintToolClass)mCurrentToolClass;

    if (tool == ADD_POINT_TOOL)
    {
    }

    if (tool == DEL_POINT_TOOL)
    {
    }

    if (tool == MOVE_POINT_TOOL)
    {

        Vector2dd releasePoint = Vector2dd(event->x(), event->y());
//        bool shiftPressed = event->modifiers().testFlag(Qt::ShiftModifier);

        Vector2dd imagePoint = widgetToImageF(releasePoint);
        mSelectedPoint = findClosest(imagePoint, 5);
        mUi->widget->update();

    }
}

void PointListEditImageWidget::childMouseMoved(QMouseEvent * event)
{
    QModelIndex topLevel = mObservationListModel->parent(QModelIndex());

    switch (mCurrentToolClass)
    {
    case MOVE_POINT_TOOL:
    {
        //   qDebug() << "Drag in selected tool";
        if (!mIsMousePressed)
            break;

        Vector2dd dragStart    = widgetToImageF(Qt2Core::Vector2ddFromQPoint(mSelectionEnd));
        Vector2dd currentPoint = widgetToImageF(Qt2Core::Vector2ddFromQPoint(event->pos()));
        Vector2dd shift = (dragStart - currentPoint);


       /* for (unsigned i = 0; i < mFeatures.mSelectedPoints.size(); i++)
        {
            mFeatures.mSelectedPoints[i]->position -= shift;
        }*/
        if (mSelectedPoint >= 0) {
            Vector2dd currentPoint = getPointById(mSelectedPoint);
            currentPoint -= shift;

            QModelIndex indexX = mObservationListModel->index(mSelectedPoint, ObservationListModel::COLUMN_U, topLevel);
            QModelIndex indexY = mObservationListModel->index(mSelectedPoint, ObservationListModel::COLUMN_V, topLevel);
            mObservationListModel->setData(indexY, QVariant(currentPoint.y()), Qt::EditRole);
            mObservationListModel->setData(indexX, QVariant(currentPoint.x()), Qt::EditRole);
        }


        mUi->widget->update();
    }
    default:
        break;
    }
    AdvancedImageWidget::childMouseMoved(event);
}

/*********************************************************************************************************
 *
 *
 *
 *********************************************************************************************************/

PointListEditImageWidgetUnited::PointListEditImageWidgetUnited(QWidget *parent, bool showHeader) :
    AdvancedImageWidget(parent, showHeader),
    mObservationListModel(NULL),
    selectionModel(NULL),
    mSelectedPoint(-1)
{
    mMoveButton    = addToolButton("Select",            QIcon(":/new/prefix1/select_by_color.png"));
    mAddButton     = addToolButton("Add Point To Path", QIcon(":/new/prefix1/vector_add.png"     ), false);
    mDeleteButton  = addToolButton("Delete Point",      QIcon(":/new/prefix1/vector_delete.png"  ), false);
    mAddInfoButton = addToolButton("Toggle info",       QIcon(":/new/prefix1/info_rhombus.png"   ), false);
    mAddInfoButton ->setCheckable(true);

}

void PointListEditImageWidgetUnited::setObservationModel(PointImageEditorInterface *observationListModel)
{
    disconnect(mObservationListModel, 0, this, 0);
    mObservationListModel = observationListModel;

    connect(mObservationListModel, SIGNAL(updateView())      , this, SLOT(update()));
    connect(mObservationListModel, SIGNAL(modelInvalidated()), this, SLOT(invalidateModel()));
}

void PointListEditImageWidgetUnited::setSelectionModel(QItemSelectionModel *_selectionModel)
{
    disconnect(selectionModel, 0, this, 0);
    selectionModel = _selectionModel;

    connect(selectionModel, SIGNAL(selectionChanged(QItemSelection,QItemSelection)), this, SLOT(update()));
}



/* This is called when model indexes are changed, and our cache is no longer valid */
void PointListEditImageWidgetUnited::invalidateModel()
{
    mSelectedPoint = -1;
    update();
}

void PointListEditImageWidgetUnited::selectPoint(int id)
{
    mSelectedPoint = id;
    if (selectionModel != NULL && mObservationListModel != NULL) {
        QModelIndex pos = QModelIndex().child(id, 0);
        selectionModel->select(pos, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    }
}

void PointListEditImageWidgetUnited::childRepaint(QPaintEvent *event, QWidget *who)
{
    AdvancedImageWidget::childRepaint(event, who);
    if (mImage.isNull())
    {
        return;
    }

    if (mObservationListModel == NULL)
    {
        return;
    }

    /* Now the points */
    QPainter painter(who);

    int rows = mObservationListModel->getPointCount();

    for (int i = 0; i < rows; i ++)
    {
        Vector2dd point = mObservationListModel->getPoint(i);
        Vector2dd imageCoords = imageToWidgetF(point);
        painter.setPen(Qt::yellow);
        drawCircle(painter, imageCoords, 5);
        painter.setPen(Qt::blue);
        drawCircle(painter, imageCoords, 10);

        if (mAddInfoButton->isChecked())
        {
            QString meta = mObservationListModel->getMeta(i);
            QPointF pos = Core2Qt::QPointFromVector2dd(imageCoords + Vector2dd(5, -10));
            painter.setPen(Qt::black);
            painter.drawText(pos, meta);
            pos += QPointF(1,1);
            painter.setPen(Qt::white);
            painter.drawText(pos, meta);
        }

        bool isSelected = false;
        if (selectionModel != NULL)
        {
            isSelected = selectionModel->isRowSelected(i, QModelIndex());
        } else {
            isSelected = (i == mSelectedPoint);
        }

        if (isSelected) {
            painter.setPen(Qt::red);
            drawCircle(painter, imageCoords, 7);

            /* Test it a bit and use QSelectionModel */
            imageCoords = imageToWidgetF(widgetToImageF(imageCoords));
            painter.setPen(Qt::cyan);
            drawCircle(painter, imageCoords, 3);
        }
    }
}


void PointListEditImageWidgetUnited::toolButtonReleased(QWidget *button)
{

    mUi->widget->unsetCursor();
    AdvancedImageWidget::toolButtonReleased(button);

    if (button == mMoveButton)
    {
        qDebug() << "Move Button";
        mCurrentToolClass = (ToolClass)MOVE_POINT_TOOL;
        mUi->widget->setCursor(Qt::ArrowCursor);
    }
    else if (button == mAddButton)
    {
        qDebug() << "Add Button";
        //mCurrentToolClass = (ToolClass)ADD_POINT_TOOL;
        mObservationListModel->appendPoint();
        mSelectedPoint = mObservationListModel->getPointCount() - 1;
        /*if (selectionModel != NULL) {
            QItemSelection::Se
            selectionModel->select(QModelIndex(mSelectedPoint, 0), QI);
        }*/


        mObservationListModel->setPoint(mSelectedPoint, Qt2Core::Vector2ddFromQPoint(mZoomCenter));
        mUi->widget->update();
    }
    else if (button == mDeleteButton)
    {
        qDebug() << "Delete Button";
        if (mSelectedPoint >= 0)
        {
            mObservationListModel->deletePoint(mSelectedPoint);
            mSelectedPoint = -1;
        }
        mUi->widget->update();
    } else if (button == mAddInfoButton)
    {
        mUi->widget->update();
    }
}

int PointListEditImageWidgetUnited::findClosest(Vector2dd imagePoint, double limitDistance )
{
    int rows = mObservationListModel->getPointCount();

    double bestDistance = limitDistance;
    int bestIndex = -1;

    for (int i = 0; i < rows; i ++)
    {
        Vector2dd currentPoint = mObservationListModel->getPoint(i);
        double dist = (imagePoint - currentPoint).l2Metric();
        if (dist < bestDistance) {
            bestDistance = dist;
            bestIndex = i;
        }
    }

    return bestIndex;
}

void PointListEditImageWidgetUnited::childMousePressed(QMouseEvent *event)
{
    AdvancedImageWidget::childMousePressed(event);

    PaintToolClass tool = (PaintToolClass)mCurrentToolClass;

    if (tool == ADD_POINT_TOOL)
    {
    }

    if (tool == DEL_POINT_TOOL)
    {
    }

    if (tool == MOVE_POINT_TOOL)
    {

        Vector2dd releasePoint = Vector2dd(event->x(), event->y());
//        bool shiftPressed = event->modifiers().testFlag(Qt::ShiftModifier);

        Vector2dd imagePoint = widgetToImageF(releasePoint);
        int selectedPoint = findClosest(imagePoint, 5);
        selectPoint(selectedPoint);
        mUi->widget->update();

    }
}

void PointListEditImageWidgetUnited::childMouseMoved(QMouseEvent * event)
{
    switch (mCurrentToolClass)
    {
    case MOVE_POINT_TOOL:
    {
        //   qDebug() << "Drag in selected tool";
        if (!mIsMousePressed)
            break;

        Vector2dd dragStart    = widgetToImageF(Qt2Core::Vector2ddFromQPoint(mSelectionEnd));
        Vector2dd currentPoint = widgetToImageF(Qt2Core::Vector2ddFromQPoint(event->pos()));
        Vector2dd shift = (dragStart - currentPoint);


       /* for (unsigned i = 0; i < mFeatures.mSelectedPoints.size(); i++)
        {
            mFeatures.mSelectedPoints[i]->position -= shift;
        }*/
        if (mSelectedPoint >= 0) {
            Vector2dd currentPoint = mObservationListModel->getPoint(mSelectedPoint);
            currentPoint -= shift;
            mObservationListModel->setPoint(mSelectedPoint, currentPoint);
        }


        mUi->widget->update();
    }
    default:
        break;
    }
    AdvancedImageWidget::childMouseMoved(event);
}

