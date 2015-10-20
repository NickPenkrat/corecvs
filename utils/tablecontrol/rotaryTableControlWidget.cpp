#include <QFileDialog>

#include "rotaryTableControlWidget.h"
#include "ui_rotaryTableControlWidget.h"

#include "mesh3DScene.h"

#include "rotaryTableMeshModel.h"

RotaryTableControlWidget::RotaryTableControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RotaryTableControlWidget)
{
    ui->setupUi(this);
    ui->widget->setCollapseTree(true);

    connect(ui->widgetPitch, SIGNAL(valueChanged(double)), this, SLOT(updateState()));
    connect(ui->widgetYaw  , SIGNAL(valueChanged(double)), this, SLOT(updateState()));
    connect(ui->widgetRoll , SIGNAL(valueChanged(double)), this, SLOT(updateState()));

    connect(ui->tableWidget, SIGNAL(cellClicked      (int,int)), this, SLOT(tableCellClicked     (int, int)));
    connect(ui->tableWidget, SIGNAL(cellDoubleClicked(int,int)), this, SLOT(tableCellDoubleClicked(int,int)));

    connect(ui->generatePushButton , SIGNAL(released()), this, SLOT(generate()));
    connect(ui->loadPushButton , SIGNAL(released()), this, SLOT(load()));
    connect(ui->savePushButton , SIGNAL(released()), this, SLOT(save()));

    connect(ui->executePushButton   , SIGNAL(released()), this, SLOT(execute()));
    connect(ui->executeIncPushButton, SIGNAL(released()), this, SLOT(executeAndIncrement()));

    connect(&generator, SIGNAL(newListGenerated(vector<CameraLocationAngles>)),
            this, SLOT(newList(vector<CameraLocationAngles>)));

    scene = QSharedPointer<Mesh3DScene>(new Mesh3DScene());
    scene->switchColor(true);
    ui->widget->setNewScenePointer(scene);

    selected = 0;
    loadCommands("commands.json");
    updateState();
}

RotaryTableControlWidget::~RotaryTableControlWidget()
{
    delete ui;
}

void RotaryTableControlWidget::loadCommands(QString filename)
{
    JSONGetter getter(filename);
    getter.visit(positions, "commands");

    for (auto &triple : positions) {
        triple = triple.toRad();
    }

    updateTable();
}

void RotaryTableControlWidget::saveCommands(QString filename)
{
    JSONSetter setter(filename);
    auto copy = positions;
    for (auto &triple: copy) {
        triple = triple.toDeg();
    }
    setter.visit(positions, "commands");
}

void RotaryTableControlWidget::updateTable()
{
    QTableWidget *widget = ui->tableWidget;
    widget->setRowCount(0);

    widget->setColumnCount(COLUMN_NUMBER);

    for (size_t count = 0; count < positions.size(); count++)
    {
        int newRow = widget->rowCount();
        widget->setRowCount(newRow + 1);


        QTableWidgetItem *item1 = new QTableWidgetItem(QString("%1°").arg(radToDeg(positions[count].yaw())));
        QTableWidgetItem *item2 = new QTableWidgetItem(QString("%1°").arg(radToDeg(positions[count].pitch())));
        QTableWidgetItem *item3 = new QTableWidgetItem(QString("%1°").arg(radToDeg(positions[count].roll())));

        if (count == selected)
        {
            item1->setBackgroundColor(Qt::red);
            item2->setBackgroundColor(Qt::red);
            item3->setBackgroundColor(Qt::red);
        }

        widget->setItem(newRow, COLUMN_AXIS_1, item1);
        widget->setItem(newRow, COLUMN_AXIS_2, item2);
        widget->setItem(newRow, COLUMN_AXIS_3, item3);


    }
}

void RotaryTableControlWidget::execute()
{
    qDebug("Move to:");
    qDebug() << QString ("O: %1").arg(radToDeg(ui->widgetYaw  ->value()));
    qDebug() << QString ("M: %1").arg(radToDeg(ui->widgetPitch->value()));
    qDebug() << QString ("I: %1").arg(radToDeg(ui->widgetRoll ->value()));
}

void RotaryTableControlWidget::executeAndIncrement()
{
    execute();
    selected++;
    selected = selected % positions.size();
    selectedToCurrent();
    updateState();
    updateTable();
}

void RotaryTableControlWidget::save()
{
    QString fileName = QFileDialog::getSaveFileName(
      this,
      tr("Save Rotation script"),
      ".",
      tr("Roations (*.json)"));
    if (!fileName.isEmpty())
    {
        saveCommands(fileName);
    }
}

void RotaryTableControlWidget::load()
{
    QString fileName = QFileDialog::getOpenFileName(
      this,
      tr("Load Rotation script"),
      ".",
      tr("Roations (*.json)"));
    if (!fileName.isEmpty())
    {
        loadCommands(fileName);
    }
}

void RotaryTableControlWidget::generate()
{
    generator.show();
    generator.raise();
}

void RotaryTableControlWidget::newList(const vector<CameraLocationAngles> &input)
{
    positions = input;
    updateTable();
}


void RotaryTableControlWidget::updateState()
{
    CameraLocationAngles loc(
        ui->widgetYaw->value(),
        ui->widgetPitch->value(),
        ui->widgetRoll->value());

    scene->clear();
    scene->currentTransform = Matrix44::Scale(1.0 / 4.0);
    scene->add(RotaryTableMeshModel::getMesh(loc), true);
    ui->widget->updateHelperObjects();
}

void RotaryTableControlWidget::selectedToCurrent()
{
    if (selected >= positions.size())
        return;
    ui->widgetYaw  ->setValue(positions[selected].yaw  ());
    ui->widgetPitch->setValue(positions[selected].pitch());
    ui->widgetRoll ->setValue(positions[selected].roll ());
}

void RotaryTableControlWidget::tableCellClicked(int row, int column)
{
    if (row >= positions.size())
        return;

    ui->widgetYaw  ->setValue(positions[row].yaw  ());
    ui->widgetPitch->setValue(positions[row].pitch());
    ui->widgetRoll ->setValue(positions[row].roll ());
}

void RotaryTableControlWidget::tableCellDoubleClicked(int row, int column)
{
    if (row >= positions.size())
        return;

    selected = row;
    selectedToCurrent();
    updateTable();
}


