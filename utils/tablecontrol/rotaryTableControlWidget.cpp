#include "rotaryTableControlWidget.h"
#include "ui_tableControlWidget.h"

#include "mesh3DScene.h"

#include "rotaryTableMeshModel.h"

RotaryTableControlWidget::RotaryTableControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TableControlWidget)
{
    ui->setupUi(this);
    ui->widget->setCollapseTree(true);

    connect(ui->widgetPitch, SIGNAL(valueChanged(double)), this, SLOT(updateState()));
    connect(ui->widgetYaw  , SIGNAL(valueChanged(double)), this, SLOT(updateState()));
    connect(ui->widgetRoll , SIGNAL(valueChanged(double)), this, SLOT(updateState()));

    connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(tableCellClicked(int, int)));

    scene = QSharedPointer<Mesh3DScene>(new Mesh3DScene());
    scene->switchColor(true);
    ui->widget->setNewScenePointer(scene);

    loadCommands();
    updateState();
}

RotaryTableControlWidget::~RotaryTableControlWidget()
{
    delete ui;
}

void RotaryTableControlWidget::loadCommands()
{
    JSONGetter getter("commands.json");
    getter.visit(positions, "commands");

    updateTable();
}

void RotaryTableControlWidget::updateTable()
{
    QTableWidget *widget = ui->tableWidget;
    widget->clear();

    widget->setColumnCount(COLUMN_NUMBER);

    for (size_t count = 0; count < positions.size(); count++)
    {
        int newRow = widget->rowCount();
        widget->setRowCount(newRow + 1);

        widget->setItem(newRow, COLUMN_AXIS_1, new QTableWidgetItem(QString("%1").arg(positions[count].yaw())));
        widget->setItem(newRow, COLUMN_AXIS_2, new QTableWidgetItem(QString("%1").arg(positions[count].pitch())));
        widget->setItem(newRow, COLUMN_AXIS_3, new QTableWidgetItem(QString("%1").arg(positions[count].roll())));

    }
}

void RotaryTableControlWidget::updateState()
{
    CameraLocationAngles loc(
        ui->widgetYaw->value(),
        ui->widgetPitch->value(),
        ui->widgetRoll->value());

    scene->clear();
    scene->currentTransform = Matrix44::Scale(20);
    scene->add(RotaryTableMeshModel::getMesh(loc), true);
    ui->widget->updateHelperObjects();
}


void RotaryTableControlWidget::tableCellClicked(int row, int column)
{
    if (row >= positions.size())
        return;

    ui->widgetYaw->setValue(positions[row].yaw());
    ui->widgetPitch->setValue(positions[row].pitch());
    ui->widgetRoll->setValue(positions[row].roll());

}

