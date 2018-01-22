#include "rotaryTableControlWidget.h"
#include "rotaryTableMeshModel.h"
#include "mesh3DScene.h"
#include "core/utils/log.h"
#include <QFileDialog>

#include "ui_rotaryTableControlWidget.h"

//#define PORT_NAME "COM1"

RotaryTableControlWidget::RotaryTableControlWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::RotaryTableControlWidget)
  //, mPort(PORT_NAME)
{
    ui->setupUi(this);
    ui->widget->setCollapseTree(true);

    connect(ui->widgetPitch        , SIGNAL(valueChanged(double)), this, SLOT(updateState()));
    connect(ui->widgetYaw          , SIGNAL(valueChanged(double)), this, SLOT(updateState()));
    connect(ui->widgetRoll         , SIGNAL(valueChanged(double)), this, SLOT(updateState()));

    connect(ui->tableWidget        , SIGNAL(cellClicked      (int,int)), this, SLOT(tableCellClicked     (int, int)));
    connect(ui->tableWidget        , SIGNAL(cellDoubleClicked(int,int)), this, SLOT(tableCellDoubleClicked(int,int)));

    connect(ui->generatePushButton , SIGNAL(released()), this, SLOT(generate()));
    connect(ui->loadPushButton     , SIGNAL(released()), this, SLOT(load()));
    connect(ui->savePushButton     , SIGNAL(released()), this, SLOT(save()));

    connect(ui->executePushButton   , SIGNAL(released()), this, SLOT(execute()));
    connect(ui->executeIncPushButton, SIGNAL(released()), this, SLOT(executeAndIncrement()));
    connect(ui->executeAllPushButton, SIGNAL(released()), this, SLOT(executeAll()));

    connect(&generator, SIGNAL(newListGenerated(vector<CameraLocationAngles>)),
            this, SLOT(newList(vector<CameraLocationAngles>)));

    scene = QSharedPointer<Mesh3DScene>(new Mesh3DScene());
    scene->switchColor(true);
    ui->widget->setNewScenePointer(scene);

    selected = 0;
    loadCommands("commands.json");
    selectedToCurrent();
    updateState();

    //if (!mPort.isOpen())
    //{
    //    mPort.open(QIODevice::ReadWrite);
    //    if (!mPort.setBaudRate(QSerialPort::Baud115200)) {
    //        L_ERROR_P("RotaryTable port - setBaudRate");
    //    }
    //    if (!mPort.setDataBits(QSerialPort::Data8)) {
    //        L_ERROR_P("RotaryTable port - setDataBits");
    //    }
    //    if (!mPort.setParity(QSerialPort::NoParity)) {
    //        L_ERROR_P("RotaryTable port - setParity");
    //    }
    //    if (!mPort.setStopBits(QSerialPort::OneStop)) {
    //        L_ERROR_P("RotaryTable port - setStopBits");
    //    }
    //    if (!mPort.setFlowControl(QSerialPort::NoFlowControl)) {
    //        L_ERROR_P("RotaryTable port - setFlowControl");
    //    }

    //    if (mPort.isOpen())
    //    {
    //        mPort.write("MODE = NONINTERPOLATED\r\nWAIT 1000\r\n");
    //    }
    //}
    //L_INFO_P("Rotary table port <%s> opened: %s", PORT_NAME, mPort.isOpen() ? "ok" : "err");
}

RotaryTableControlWidget::~RotaryTableControlWidget()
{
    delete ui;
    //mPort.close();
}

void RotaryTableControlWidget::loadCommands(QString filename)
{
    QFile file(filename);
    if (!file.exists())
    {
        qDebug() << "RotaryTableControlWidget::loadCommands(" << filename << "): no such file";
    } else {
        JSONGetter getter(filename);
        getter.visit(positions, "commands");

        for (auto &triple : positions) {
            triple = triple.toRad();
        }
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
    setter.visit(copy, "commands");
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

        // QString("%1°").arg(radToDeg(positions[count].yaw()))  // uses 4-5 digits after comma, which were not nice
        char buf[32];
#define TOSTR(dval) (snprintf2buf(buf, "%.3f°", (float)(dval)), buf)

        QTableWidgetItem *item1 = new QTableWidgetItem(QString(TOSTR(radToDeg(positions[count].yaw()))));
        QTableWidgetItem *item2 = new QTableWidgetItem(QString(TOSTR(radToDeg(positions[count].pitch()))));
        QTableWidgetItem *item3 = new QTableWidgetItem(QString(TOSTR(radToDeg(positions[count].roll()))));

#undef TOSTR

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
    double y = radToDeg(ui->widgetYaw->value());
    double p = radToDeg(ui->widgetPitch->value());
    double r = radToDeg(ui->widgetRoll->value());

    int iy = roundSign(y);
    int ip = roundSign(p);
    int ir = roundSign(r);

    //L_INFO_P("moving to [O:%.3f, I:%.3f, M:%.3f]", y, r, p);

    //L_INFO_P("The command to send:<%s>", cmd);
    //
    // TODO: implement for the table accroding to the doc!
    //if (mPort.isOpen())
    //{
    //    qint64 res = mPort.write(cmd);
    //    L_INFO_P("The command is sent, res = %d", (int)res);
    //}

    QString path = getPathScript(iy, ir, ip);

    std::ostringstream cmds;
    cmds << "AX_O "       << iy << std::endl
         << "AX_I "       << ir << std::endl
         << "AX_M "       << ip << std::endl
         << "WAIT 20000"        << std::endl;

    std::ofstream f(QSTR_DATA_PTR(path));
    f << "MODE=NONINTERPOLATED" << std::endl << cmds.str() << "END" << std::endl;
    f.close();

    L_INFO_P("<%s> saved", QSTR_DATA_PTR(path));

    addCmdsToScriptAll(cmds.str());
}

void RotaryTableControlWidget::executeAndIncrement()
{
    execute();
    if (positions.size() > 0)
    {
        selected = (selected + 1) % positions.size();
        selectedToCurrent();
    }
    updateState();
    updateTable();
}

void RotaryTableControlWidget::executeAll()
{
    selected = 0;
    selectedToCurrent();
    updateState();
    updateTable();

    QFile::remove(getPathScriptAll());

    do {
        executeAndIncrement();
    } while (selected != 0);

    addCmdsToScriptAll("END");

    L_INFO_P("<%s> created", QSTR_DATA_PTR(getPathScriptAll()));
}

void RotaryTableControlWidget::save()
{
    QString fileName = QFileDialog::getSaveFileName(this
        , tr("Save Rotation script")
        , "."
        , tr("Rotations (*.json)"));

    if (!fileName.isEmpty())
    {
        saveCommands(fileName);
    }
}

void RotaryTableControlWidget::load()
{
    QString fileName = QFileDialog::getOpenFileName(this
        , tr("Load Rotation script")
        , "."
        , tr("Rotations (*.json)"));

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

void RotaryTableControlWidget::tableCellClicked(int row, int /*column*/)
{
    if (row >= (int)positions.size())
        return;

    ui->widgetYaw  ->setValue(positions[row].yaw  ());
    ui->widgetPitch->setValue(positions[row].pitch());
    ui->widgetRoll ->setValue(positions[row].roll ());
}

void RotaryTableControlWidget::tableCellDoubleClicked(int row, int /*column*/)
{
    if (row >= (int)positions.size())
        return;

    selected = row;
    selectedToCurrent();
    updateTable();
}

QString RotaryTableControlWidget::getPathScriptAll()
{
    return (mScriptsPath.isEmpty() ? "." : mScriptsPath) + PATH_SEPARATOR + "_autoplay_rot_all.txt";
}

QString RotaryTableControlWidget::getPathScript(int iy, int ir, int ip)
{
    char filename[256];
    snprintf2buf(filename, "_autoplay_rot_O=%03d_I=%03d_M=%02d.txt", iy, ir, ip);

    return (mScriptsPath.isEmpty() ? "." : mScriptsPath) + PATH_SEPARATOR + filename;
}

void RotaryTableControlWidget::addCmdsToScriptAll(const string &s)
{
    bool exist = QFile::exists(getPathScriptAll());

    std::ofstream f(QSTR_DATA_PTR(getPathScriptAll()), std::ios_base::app);

    if (!exist) {
        f << "MODE=NONINTERPOLATED" << std::endl << std::endl;
    }
    f << s.c_str() << std::endl;
    f.close();
}
