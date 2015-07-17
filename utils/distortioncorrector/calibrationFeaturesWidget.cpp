#include "calibrationFeaturesWidget.h"
#include "ui_calibrationFeaturesWidget.h"
#include <QSettings>

using namespace corecvs;


const int CalibrationFeaturesWidget::REASONABLE_INF = 999999;

CalibrationFeaturesWidget::CalibrationFeaturesWidget(QWidget *parent) :
    QWidget(parent),
    geometryFeatures(NULL),
    ui(new Ui::CalibrationFeaturesWidget)
{
    ui->setupUi(this);

    ui->imageXSpinBox->setMaximum( REASONABLE_INF);
    ui->imageYSpinBox->setMaximum( REASONABLE_INF);
    ui->imageXSpinBox->setMinimum(-REASONABLE_INF);
    ui->imageYSpinBox->setMinimum(-REASONABLE_INF);


    ui->xCoordSpinBox->setMaximum( REASONABLE_INF);
    ui->yCoordSpinBox->setMaximum( REASONABLE_INF);
    ui->zCoordSpinBox->setMaximum( REASONABLE_INF);
    ui->xCoordSpinBox->setMinimum(-REASONABLE_INF);
    ui->yCoordSpinBox->setMinimum(-REASONABLE_INF);
    ui->zCoordSpinBox->setMinimum(-REASONABLE_INF);

    connect(ui->pointsTableWidget, SIGNAL(cellDoubleClicked(int,int)), this, SLOT(choosePoint(int,int)));
    connect(ui->saveButton       , SIGNAL(released())                , this, SLOT(addVector()));

}



CalibrationFeaturesWidget::~CalibrationFeaturesWidget()
{
    delete ui;
}

void CalibrationFeaturesWidget::manualAddPoint(const corecvs::Vector2dd &point)
{

}

void CalibrationFeaturesWidget::addVector()
{
    Vector3dd key(ui->xCoordSpinBox->value(), ui->yCoordSpinBox->value(), ui->zCoordSpinBox->value());
    Vector2dd value(ui->imageXSpinBox->value(), ui->imageYSpinBox->value());
    PointObservation observation(key, value);
    addObservation(observation);
    updateWidget();
}

void CalibrationFeaturesWidget::addObservation(PointObservation &observation)
{
    observationList.push_back(observation);
    /*unsigned i = 0;
    while (i < mCorrectionMap.size())
    {
        if ((mCorrectionMap.at(i).projection - value).l2Metric() < EPSILON)
        {
            mCorrectionMap.erase(mCorrectionMap.begin() + i);
        } else {
            i ++;
        }
    }
    mCorrectionMap.push_back(PointObservation(key, value));

    printVectorPair(key, value);

    mUi->widget->addVertex(value);*/
}

void CalibrationFeaturesWidget::clearObservationPoints()
{
    observationList.clear();
    int row = ui->pointsTableWidget->rowCount();
    for (int i = 0; i < row; i ++)
    {
        ui->pointsTableWidget->removeRow(0);
    }
}

void CalibrationFeaturesWidget::choosePoint(int row, int /*column*/)
{
    QTableWidget *table = ui->pointsTableWidget;

    for (int i = 0; i < table->rowCount(); i ++)
    {
        for (int j = 0; j < table->columnCount(); j ++)
        {
            table->item(i, j)->setBackgroundColor(Qt::color0);
        }
    }
    for (int i = 0; i < table->columnCount(); i ++)
    {
        table->item(row, i)->setBackgroundColor(Qt::yellow);
    }
    ui->imageXSpinBox->setValue(table->item(row, 3)->text().toDouble());
    ui->imageYSpinBox->setValue(table->item(row, 4)->text().toDouble());

    ui->xCoordSpinBox->setValue(table->item(row, 0)->text().toDouble());
    ui->yCoordSpinBox->setValue(table->item(row, 1)->text().toDouble());
    ui->zCoordSpinBox->setValue(table->item(row, 2)->text().toDouble());
    //ui->widget->setCurrentPoint(QPointF(ui->dsbImageX->value(), ui->dsbImageY->value()));
}

void CalibrationFeaturesWidget::deleteObservation()
{
   /* QTableWidget *table = mUi->pointsTableWidget;
    Vector2dd value(mUi->imageXSpinBox->value(), mUi->imageYSpinBox->value());
    unsigned i = 0;
    while (i < mCorrectionMap.size())
    {
        if ((mCorrectionMap.at(i).projection - value).l2Metric() < EPSILON)
        {
            mCorrectionMap.erase(mCorrectionMap.begin() + i);
        } else {
            i ++;
        }
    }

    for (int i = 0; i < table->rowCount(); i ++)
    {
        if (table->item(i, 3)->text().toDouble() == value.x() &&
            table->item(i, 4)->text().toDouble() == value.y())
        {
            table->removeRow(i);
            break;
        }
    }
    // TODO: PaintFeature
    // ui->widget->deletePoint(QPointF(value.x(), value.y()));*/
}

void CalibrationFeaturesWidget::updateWidget()
{
    QTableWidget *table = ui->pointsTableWidget;
    table->setRowCount(0);
    for (unsigned i = 0; i < observationList.size(); i++)
    {
        PointObservation &observation = observationList[i];
        table->insertRow(table->rowCount());
        int row = table->rowCount() - 1;
        table->setItem(row, COLUMN_X, new QTableWidgetItem(QString::number(observation.x())));
        table->setItem(row, COLUMN_Y, new QTableWidgetItem(QString::number(observation.y())));
        table->setItem(row, COLUMN_Z, new QTableWidgetItem(QString::number(observation.z())));
        table->setItem(row, COLUMN_U, new QTableWidgetItem(QString::number(observation.u())));
        table->setItem(row, COLUMN_V, new QTableWidgetItem(QString::number(observation.v())));
    }


    /* And tree */

    QTreeWidget *tree = ui->treeWidget;
    tree->clear();
    tree->setColumnCount(4);
    tree->header()->resizeSection(0, 200);
    tree->header()->resizeSection(1, 100);
    tree->header()->resizeSection(2, 100);

    if (geometryFeatures)
    {
        for(unsigned i = 0; i < geometryFeatures->mPaths.size();i++ )
        {
            SelectableGeometryFeatures::VertexPath *path = geometryFeatures->mPaths[i];
            QTreeWidgetItem *item = new QTreeWidgetItem(QStringList("Line"));
            tree->insertTopLevelItem(tree->topLevelItemCount(),item);
            for(unsigned j = 0; j < path->vertexes.size(); j++ )
            {
                SelectableGeometryFeatures::Vertex* vertex = path->vertexes[j];

                QTreeWidgetItem *subitem = new QTreeWidgetItem(QStringList("Vertex"));
                item->addChild(subitem);
                QDoubleSpinBox *xSpinBox = new QDoubleSpinBox();
                xSpinBox->setMaximum( REASONABLE_INF);
                xSpinBox->setMinimum(-REASONABLE_INF);
                xSpinBox->setValue(vertex->position.x());

                QDoubleSpinBox *ySpinBox = new QDoubleSpinBox();
                ySpinBox->setMaximum( REASONABLE_INF);
                ySpinBox->setMinimum(-REASONABLE_INF);
                ySpinBox->setValue(vertex->position.y());


                tree->setItemWidget(subitem, 1, xSpinBox);
                tree->setItemWidget(subitem, 2, ySpinBox);
            }
        }
    }
}
/*
void CalibrationFeaturesWidget::editPoint(const QPointF &prevPoint, const QPointF &newPoint)
{
    QTableWidget *table = mUi->pointsTableWidget;
    for (int i = 0; i < table->rowCount(); i ++)
    {
        QPointF tablePoint(table->item(i, 3)->text().toDouble(), table->item(i, 4)->text().toDouble());
        if ((prevPoint - tablePoint).manhattanLength() < EPSILON)
        {
            table->item(i, 3)->setText(QString::number(newPoint.x()));
            table->item(i, 4)->setText(QString::number(newPoint.y()));
            choosePoint(i, 0);
        }
    }
    Vector2dd prevVector(prevPoint.x(), prevPoint.y());
    Vector3dd spacePos(-1, -1, -1);
    for (unsigned i = 0; i < mCorrectionMap.size(); i ++)
    {
        if ((mCorrectionMap.at(i).projection - prevVector).l2Metric() < EPSILON)
        {
            spacePos = mCorrectionMap.at(i).point;
            mCorrectionMap.erase(mCorrectionMap.begin() + i);
            break;
        }
    }
    Vector2dd newVector(newPoint.x(), newPoint.y());
    mCorrectionMap.push_back(PointObservation(spacePos, newVector));
}
*/


/**
 *   Loading and saving of current points
 **/
void CalibrationFeaturesWidget::savePoints()
{
    QSettings settings("distortionCorrection.conf", QSettings::IniFormat);
    settings.beginWriteArray("points");
    for (unsigned i = 0; i < observationList.size(); i ++)
    {
        PointObservation & observation = observationList.at(i);
        settings.setArrayIndex(i);
        settings.setValue("spacePoint_X", observation.x());
        settings.setValue("spacePoint_Y", observation.y());
        settings.setValue("spacePoint_Z", observation.z());
        settings.setValue("imagePoint_X", observation.u());
        settings.setValue("imagePoint_Y", observation.v());
    }
    settings.endArray();

    /*vector<vector<Vector2dd> > straights = mUi->widget->getPaths();
    settings.beginWriteArray("lines");
    for (unsigned i = 0; i < straights.size(); i++) {
        settings.setArrayIndex(i);
        settings.beginWriteArray("subpoints");
        for (unsigned j = 0; j < straights[i].size(); j++) {
            settings.setArrayIndex(j);

            settings.setValue("point.x", straights[i][j].x());
            settings.setValue("point.y", straights[i][j].y());
        }
        settings.endArray();
    }
    settings.endArray();*/
}

void CalibrationFeaturesWidget::loadPoints()
{
    QSettings settings("distortionCorrection.conf", QSettings::IniFormat);
    int size = settings.beginReadArray("points");
    for (int i = 0; i < size; i ++)
    {
        PointObservation point;
        settings.setArrayIndex(i);
        point.x() = settings.value("spacePoint_X").toDouble();
        point.y() = settings.value("spacePoint_Y").toDouble();
        point.z() = settings.value("spacePoint_Z").toDouble();
        point.u() = settings.value("imagePoint_X").toDouble();
        point.v() = settings.value("imagePoint_Y").toDouble();
        addObservation(point);
    }
    settings.endArray();

    updateWidget();

    /* TODO: Make an interface for this */
    /*PaintImageWidget *canvas = mUi->widget;

    int lines = settings.beginReadArray("lines");
    for (int i = 0; i < lines; i++) {
        canvas->mPaths.append(PaintImageWidget::VertexPath());
        PaintImageWidget::VertexPath &path = canvas->mPaths.last();

        settings.setArrayIndex(i);
        int subpoints = settings.beginReadArray("subpoints");
        for (int j = 0; j < subpoints; j++) {
            settings.setArrayIndex(j);
            Vector2dd pos;
            pos.x() = settings.value("point.x").toDouble();
            pos.y() = settings.value("point.y").toDouble();
            canvas->addVertex(pos);
            canvas->addVertexToPath(&canvas->mPoints.last(), &path);
        }
        settings.endArray();
    }
    settings.endArray();*/
}


