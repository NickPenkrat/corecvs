#include "fixtureGeometryControlWidget.h"
#include "ui_fixtureGeometryControlWidget.h"

using namespace corecvs;

FixtureGeometryControlWidget::FixtureGeometryControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::FixtureGeometryControlWidget)
{
    ui->setupUi(this);

    QObject::connect(ui->originXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
    QObject::connect(ui->originYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
    QObject::connect(ui->originZSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));

    QObject::connect(ui->ort1XSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
    QObject::connect(ui->ort1YSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
    QObject::connect(ui->ort1ZSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));

    QObject::connect(ui->ort2XSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
    QObject::connect(ui->ort2YSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
    QObject::connect(ui->ort2ZSpinBox, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));

    QObject::connect(ui->polygonWidget, SIGNAL(cellChanged(int,int)), this, SLOT(paramsChangedInUI()));

}

FixtureGeometryControlWidget::~FixtureGeometryControlWidget()
{
     delete_safe(ui);
}


void FixtureGeometryControlWidget::paramsChangedInUI()
{
    emit paramsChanged();
}

/*
void FixtureGeometryControlWidget::addEntry()
{
    QListWidgetItem *item = new QListWidgetItem();
    ui->listWidget->insertItem(ui->polygonWidget->count(), item);
    QDoubleSpinBox *widget = new QDoubleSpinBox;
    widget->show();
    widget->setMaximum(mMaximum);
    widget->setMinimum(mMinimum);

    ui->listWidget->setItemWidget(item, widget);
    connect(widget, SIGNAL(valueChanged(double)), this, SIGNAL(valueChanged()));

    emit valueChanged();
}

void FixtureGeometryControlWidget::removeEntry()
{
    QListWidgetItem *item = ui->polygonWidget->takeItem(ui->listWidget->count() - 1);
    delete item;
    emit valueChanged();
}

void FixtureGeometryControlWidget::resize(int size)
{
    if (size < 0) size = 0;

    while ( ui->polygonWidget->count() > size)
        removeEntry();

    while ( ui->polygonWidget->count() < size)
        addEntry();

}
*/

/**/

void FixtureGeometryControlWidget::getParameters(FlatPolygon& params) const
{
    params.frame.p1 = Vector3dd(
                ui->originXSpinBox->value(),
                ui->originYSpinBox->value(),
                ui->originZSpinBox->value()
            );
    params.frame.e1 = Vector3dd(
                ui->ort1XSpinBox->value(),
                ui->ort1YSpinBox->value(),
                ui->ort1ZSpinBox->value()
            );

    params.frame.e2 = Vector3dd(
                ui->ort2XSpinBox->value(),
                ui->ort2YSpinBox->value(),
                ui->ort2ZSpinBox->value()
            );

    params.polygon.resize(ui->polygonWidget->rowCount());
    for (size_t i = 0; i < ui->polygonWidget->rowCount(); i++)
    {
        QDoubleSpinBox *xWidget = static_cast<QDoubleSpinBox *>(ui->polygonWidget->cellWidget(i,1));
        QDoubleSpinBox *yWidget = static_cast<QDoubleSpinBox *>(ui->polygonWidget->cellWidget(i,2));
        params.polygon[i] = Vector2dd(xWidget->value(), yWidget->value());
    }

}

FlatPolygon *FixtureGeometryControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/
    FlatPolygon *result = new FlatPolygon();
    getParameters(*result);
    return result;
}

void FixtureGeometryControlWidget::setParameters(const FlatPolygon &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);

    ui->originXSpinBox->setValue(input.frame.p1.x());
    ui->originYSpinBox->setValue(input.frame.p1.y());
    ui->originZSpinBox->setValue(input.frame.p1.z());

    ui->ort1XSpinBox->setValue(input.frame.e1.x());
    ui->ort1YSpinBox->setValue(input.frame.e1.y());
    ui->ort1ZSpinBox->setValue(input.frame.e1.z());

    ui->ort2XSpinBox->setValue(input.frame.e2.x());
    ui->ort2YSpinBox->setValue(input.frame.e2.y());
    ui->ort2ZSpinBox->setValue(input.frame.e2.z());

    ui->polygonWidget->clear();
    for (size_t i = 0; i < input.polygon.size(); i++)
    {
        ui->polygonWidget->insertRow(i);
        ui->polygonWidget->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
        {
            QDoubleSpinBox *widget = new QDoubleSpinBox;
            widget->show();
            widget->setMaximum( 9999999);
            widget->setMinimum(-9999999);
            widget->setDecimals(5);
            widget->setValue(input.polygon[i].x());
            connect(widget, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
            ui->polygonWidget->setCellWidget(i, 1, widget);
        }
        {
            QDoubleSpinBox *widget = new QDoubleSpinBox;
            widget->show();
            widget->setMaximum( 9999999);
            widget->setMinimum(-9999999);
            widget->setDecimals(5);
            widget->setValue(input.polygon[i].y());
            connect(widget, SIGNAL(valueChanged(double)), this, SLOT(paramsChangedInUI()));
            ui->polygonWidget->setCellWidget(i, 2, widget);
        }
    }
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void FixtureGeometryControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    FlatPolygon *inputCasted = static_cast<FlatPolygon *>(input);
    setParameters(*inputCasted);
}

void FixtureGeometryControlWidget::loadParamWidget(WidgetLoader &)
{
    SYNC_PRINT(("FixtureGeometryControlWidget::loadParamWidget : Not yet implemented\n"));
}

void FixtureGeometryControlWidget::saveParamWidget(WidgetSaver &)
{
    SYNC_PRINT(("FixtureGeometryControlWidget::saveParamWidget : Not yet implemented\n"));
}

void FixtureGeometryControlWidget::setFixtureGeometry(FixtureSceneGeometry *geometry)
{
    mGeometry = geometry;
    ui->infoLabel->setText(QString("Related points :%1").arg(geometry->relatedPoints.size()));

    double sum = 0;
    double sumRepr = 0;

    Plane3d plane = geometry->frame.toPlane();

    for (size_t i = 0; i < geometry->relatedPoints.size(); i++)
    {
        SceneFeaturePoint *point = geometry->relatedPoints[i];
        Vector3dd position = point->position;
        Vector3dd repr     = point->reprojectedPosition;

        sum     += plane.distanceTo(position);
        sumRepr += plane.distanceTo(repr);
    }
}
