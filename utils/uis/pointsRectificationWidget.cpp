#include "pointsRectificationWidget.h"
#include "ui_pointsRectificationWidget.h"
#include "g12Image.h"
#include "qtHelper.h"


PointsRectificationWidget::PointsRectificationWidget(QWidget *parent) :
      QWidget(parent)
    , mUi(new Ui::PointsRectificationWidget)
    , mCorrespondancePoints(NULL)
    , mLeftBuffer(NULL)
    , mRightBuffer(NULL)
{
    mUi->setupUi(this);
    initModel();
    connect(mUi->deleteButton , SIGNAL(released()), this, SLOT(deletePairs()));
    connect(mUi->addButton    , SIGNAL(released()), this, SLOT(addPair()));
    connect(mUi->rectifyButton, SIGNAL(released()), this, SLOT(initCorrespondancePoints()));

    connect(mUi->leftWidget,  SIGNAL(editPoint(QPointF,QPointF)), this, SLOT(editPointLeftImage(QPointF,QPointF)));
    connect(mUi->rightWidget, SIGNAL(editPoint(QPointF,QPointF)), this, SLOT(editPointRightImage(QPointF,QPointF)));

    connect(mUi->leftWidget , SIGNAL(notifyCenterPointChanged(QPoint)), this, SLOT(updateRightCenter(QPoint)));
    connect(mUi->rightWidget, SIGNAL(notifyCenterPointChanged(QPoint)), this, SLOT(updateLeftCenter (QPoint)));

    connect(mUi->leftWidget , SIGNAL(notifyZoomChanged(double)), this, SLOT(updateRightZoom(double)));
    connect(mUi->rightWidget, SIGNAL(notifyZoomChanged(double)), this, SLOT(updateLeftZoom (double)));

    connect(mUi->rightWidget, SIGNAL(toolButtonClicked(ToolButtonType)), mUi->leftWidget, SLOT(clickToolButton(ToolButtonType)));
    connect(mUi->leftWidget,  SIGNAL(toolButtonClicked(ToolButtonType)), mUi->rightWidget, SLOT(clickToolButton(ToolButtonType)));

}

void PointsRectificationWidget::editPointLeftImage(const QPointF &prevPoint, const QPointF &newPoint)
{
    setNewPointsCoord(prevPoint, newPoint, true);
}

void PointsRectificationWidget::editPointRightImage(const QPointF &prevPoint, const QPointF &newPoint)
{
    setNewPointsCoord(prevPoint, newPoint, false);
}

void PointsRectificationWidget::setNewPointsCoord(const QPointF &prevPoint, const QPointF &newPoint, bool isLeftImage)
{
    int xColumn = isLeftImage ? 0 : 2;
    int yColumn = isLeftImage ? 1 : 3;
    for (int i = 0; i < mPointModel->rowCount(); i ++)
    {
        if (fabs(mPointModel->item(i, xColumn)->text().toDouble() - prevPoint.x()) < 0.001 &&
            fabs(mPointModel->item(i, yColumn)->text().toDouble() - prevPoint.y()) < 0.001)
        {
            QStandardItem *itemX = new QStandardItem(QString::number(newPoint.x()));
            QStandardItem *itemY = new QStandardItem(QString::number(newPoint.y()));
            mPointModel->setItem(i, xColumn, itemX);
            mPointModel->setItem(i, yColumn, itemY);
        }
    }
}

void PointsRectificationWidget::initModel()
{
    mPointModel = new QStandardItemModel();
    QStringList headers;
    headers << "Left point x" << "Left point y" << "Right point x" << "Right point y";
    mPointModel->setHorizontalHeaderLabels(headers);
    mUi->pointsMatchingTableView->setModel(mPointModel);
    connect(mUi->pointsMatchingTableView->selectionModel(),
            SIGNAL(selectionChanged(QItemSelection,QItemSelection)),
            this,
            SLOT  (selectionChanged(QItemSelection,QItemSelection)));
}

void PointsRectificationWidget::selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    foreach (QModelIndex const &index, selected.indexes())
    {
        //FIXME:: check all collumns?
        if (index.column() == 0)
        {
            int row = index.row();
            QPointF leftPoint(mPointModel->item(row, 0)->text().toDouble(), mPointModel->item(row, 1)->text().toDouble());
            QPointF rightPoint(mPointModel->item(row, 2)->text().toDouble(), mPointModel->item(row, 3)->text().toDouble());
            mUi->leftWidget ->setSelectedPoint(leftPoint, true);
            mUi->rightWidget->setSelectedPoint(rightPoint, true);
        }
    }
    foreach (QModelIndex const &index, deselected.indexes())
    {
        if (index.column() == 0)
        {
            int row = index.row();
            QPointF leftPoint(mPointModel->item(row, 0)->text().toDouble(), mPointModel->item(row, 1)->text().toDouble());
            QPointF rightPoint(mPointModel->item(row, 2)->text().toDouble(), mPointModel->item(row, 3)->text().toDouble());
            mUi->leftWidget->setSelectedPoint(leftPoint, false);
            mUi->rightWidget->setSelectedPoint(rightPoint, false);
        }
    }
}

PointsRectificationWidget::~PointsRectificationWidget()
{
    delete_safe(mUi);
    delete_safe(mPointModel);
    delete_safe(mCorrespondancePoints);
    delete_safe(mLeftBuffer);
    delete_safe(mRightBuffer);
}

void PointsRectificationWidget::setImage(G12Buffer *buffer, Frames::FrameSourceId id)
{
    if (buffer == NULL) {
        return;
    }
    switch (id)
    {
    case Frames::LEFT_FRAME :
        delete mLeftBuffer;
        mLeftBuffer = new G12Buffer(buffer);
        mUi->leftWidget->setImage(QSharedPointer<QImage>(new G12Image(buffer)));
        break;
    case Frames::RIGHT_FRAME :
        delete mRightBuffer;
        mRightBuffer = new G12Buffer(buffer);
        mUi->rightWidget->setImage(QSharedPointer<QImage>(new G12Image(buffer)));
        break;
    default:
        break;
    }
}

void PointsRectificationWidget::addPointPair(const QPointF &leftPoint, const QPointF &rightPoint)
{
    QList<QStandardItem *> items;
    items << new QStandardItem(QString::number(leftPoint.x()))  << new QStandardItem(QString::number(leftPoint.y()))
          << new QStandardItem(QString::number(rightPoint.x())) << new QStandardItem(QString::number(rightPoint.y()));
    foreach (QStandardItem *item, items)
    {
        item->setEditable(false);
    }
    mPointModel->appendRow(items);
    mUi->leftWidget ->addVertex(Qt2Core::Vector2ddFromQPointF(leftPoint ));
    mUi->rightWidget->addVertex(Qt2Core::Vector2ddFromQPointF(rightPoint));
}

void PointsRectificationWidget::deletePairs()
{
    QModelIndexList selectedIndices = mUi->pointsMatchingTableView->selectionModel()->selectedIndexes();
    QMap<int, QModelIndex> toDelete;
    foreach (QModelIndex const &index, selectedIndices)
    {
        if (toDelete.contains(index.row()))
            continue;

        toDelete[index.row()] = index;
    }

    for (int i = toDelete.values().size() - 1; i >= 0; i--)
    {
        int selectedRow = toDelete.keys().at(i);

        QPointF leftPoint(mPointModel->item(selectedRow, 0)->text().toDouble(),
                          mPointModel->item(selectedRow, 1)->text().toDouble());
        QPointF rightPoint(mPointModel->item(selectedRow, 2)->text().toDouble(),
                           mPointModel->item(selectedRow, 3)->text().toDouble());

        mUi->leftWidget->deleteVertex(Qt2Core::Vector2ddFromQPointF(leftPoint));
        mUi->rightWidget->deleteVertex(Qt2Core::Vector2ddFromQPointF(rightPoint));

        mPointModel->removeRow(selectedRow);
    }
}

bool PointsRectificationWidget::isNextRow(const QModelIndex &index1, const QModelIndex &index2)
{
    return index1.row() > index2.row();
}

void PointsRectificationWidget::addPair()
{
    QPointF leftPoint  = mUi->leftWidget->currentPoint();
    QPointF rightPoint = mUi->rightWidget->currentPoint();
    addPointPair(leftPoint, rightPoint);
}

void PointsRectificationWidget::initCorrespondancePoints()
{
    delete mCorrespondancePoints;
    mCorrespondancePoints = new CorrespondanceList();
    for (int i = 0; i < mPointModel->rowCount(); i ++)
    {
        Vector2dd left (mPointModel->item(i, 0)->text().toDouble(), mPointModel->item(i,1)->text().toDouble());
        Vector2dd right(mPointModel->item(i, 2)->text().toDouble(), mPointModel->item(i, 3)->text().toDouble());
        mCorrespondancePoints->push_back(Correspondance(left, right));
    }
    mCorrespondancePoints->h = mLeftBuffer->h;
    mCorrespondancePoints->w = mLeftBuffer->w;

    emit readyCorrespondancePoints(mCorrespondancePoints, mLeftBuffer, mRightBuffer);
}

/* Lock functionality */

void PointsRectificationWidget::updateRightZoom  (double zoom)
{
    if (!mUi->lockButton->isChecked())
        return;
    mUi->rightWidget->blockSignals(true);
    mUi->rightWidget->changeZoom(zoom);
    mUi->rightWidget->blockSignals(false);
}

void PointsRectificationWidget::updateLeftZoom   (double zoom)
{
    if (!mUi->lockButton->isChecked())
        return;
    mUi->leftWidget->blockSignals(true);
    mUi->leftWidget->changeZoom(zoom);
    mUi->leftWidget->blockSignals(false);
}

void PointsRectificationWidget::updateRightCenter(QPoint center)
{
    if (!mUi->lockButton->isChecked())
        return;
    mUi->rightWidget->blockSignals(true);
    mUi->rightWidget->changeCenterPoint(center);
    mUi->rightWidget->blockSignals(false);
}

void PointsRectificationWidget::updateLeftCenter (QPoint center)
{
    if (!mUi->lockButton->isChecked())
        return;
    mUi->leftWidget->blockSignals(true);
    mUi->leftWidget->changeCenterPoint(center);
    mUi->leftWidget->blockSignals(false);
}

