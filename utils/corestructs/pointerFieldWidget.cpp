#include "pointerFieldWidget.h"
#include "ui_pointerFieldWidget.h"

#include "g12Image.h"
#include "rgb24Buffer.h"
#include "fixtureCamera.h"
#include "bufferFactory.h"
#include "fixtureScene.h"

#include "jsonGetter.h"
#ifdef WITH_JSONMODERN
#include "jsonModernReader.h"
#endif
#ifdef WITH_RAPIDJSON
#include "rapidJSONReader.h"
#endif

#include <string>

#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>

using namespace corecvs;
using namespace std;

PointerFieldWidget::PointerFieldWidget(const corecvs::PointerField *field, QWidget *parent) :
    QWidget(parent),
    fieldReflection(field),
    ui(new Ui::PointerFieldWidget)
{
    ui->setupUi(this);

    if (!fieldReflection->isInputPin()) {
        ui->loadPushButton->setEnabled(false);
        ui->loadPushButton->hide();
    }

    if (!fieldReflection->isOuputPin()) {
        ui->savePushButton->setEnabled(false);
        ui->savePushButton->hide();
    }


    /* Ok so far this is how we roll. This should be pluggable */
    if (std::string(fieldReflection->targetClass) == "corecvs::RGB24Buffer" )
    {
        connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadRGB24Buffer()));
        connect(ui->showPushButton, SIGNAL(released()), this, SLOT(showRGB24Buffer()));
    }

    if (std::string(fieldReflection->targetClass) == "corecvs::G12Buffer" )
    {
        connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadG12Buffer()));
        connect(ui->showPushButton, SIGNAL(released()), this, SLOT(showG12Buffer()));
    }


    if (std::string(fieldReflection->targetClass) == "corecvs::FixtureScene" )
    {
        connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadFixtureScene()));
        connect(ui->showPushButton, SIGNAL(released()), this, SLOT(showFixtureScene()));
        connect(ui->savePushButton, SIGNAL(released()), this, SLOT(saveFixtureScene()));

    }

    if (std::string(fieldReflection->targetClass) == "corecvs::FixtureCamera" )
    {
        //connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadRGB24Buffer()));
        connect(ui->showPushButton, SIGNAL(released()), this, SLOT(showFixtureCamera()));
        connect(ui->savePushButton, SIGNAL(released()), this, SLOT(saveFixtureCamera()));

    }



}

PointerFieldWidget::~PointerFieldWidget()
{
    delete ui;
}

void PointerFieldWidget::setValue(void *value)
{
   rawPointer = value;
   if (value == NULL) {
       ui->isNull->setText("NULL");
   } else {
       if (std::string(fieldReflection->targetClass) == "corecvs::RGB24Buffer" )
       {
           RGB24Buffer *buffer = static_cast<RGB24Buffer *>(rawPointer);
           ui->isNull->setText(QString("Image [%1x%2]").arg(buffer->w).arg(buffer->h));
           return;
       }
       if (std::string(fieldReflection->targetClass) == "corecvs::FixtureCamera" )
       {
           corecvs::FixtureCamera *camera = static_cast<corecvs::FixtureCamera *>(rawPointer);
           ui->isNull->setText(QString("Camera [%1]").arg(QString::fromStdString(camera->nameId)));
           return;
       }
       ui->isNull->setText("Set");
   }
}

void *PointerFieldWidget::getValue()
{
    return rawPointer;
}

void PointerFieldWidget::loadRGB24Buffer()
{
    if (std::string(fieldReflection->targetClass) != "corecvs::RGB24Buffer" )
        return;

    QString filename = QFileDialog::getOpenFileName(
                this,
                "Choose an file name",
                ".",
                "Text (*.bmp *.jpeg *.png)"
                );

    /*TODO: This is a design flaw around this desctruction */
    corecvs::RGB24Buffer *buffer = static_cast<corecvs::RGB24Buffer *>(rawPointer);
    delete_safe(buffer);
    rawPointer = BufferFactory::getInstance()->loadRGB24Bitmap(filename.toStdString());
    setValue(rawPointer);
}

void PointerFieldWidget::showRGB24Buffer()
{
    if (std::string(fieldReflection->targetClass) != "corecvs::RGB24Buffer" || rawPointer == NULL)
        return;
    if (image == NULL)
        image = new AdvancedImageWidget;

    RGB24Buffer *buffer = static_cast<RGB24Buffer *>(rawPointer);
    image->setImage(QSharedPointer<QImage>(new RGB24Image(buffer)));
    image->show();
    image->raise();
}

void PointerFieldWidget::loadG12Buffer()
{}

void PointerFieldWidget::showG12Buffer()
{}

void PointerFieldWidget::loadFixtureScene()
{
    if (std::string(fieldReflection->targetClass) != "corecvs::FixtureScene")
    {
        SYNC_PRINT(("PointerFieldWidget::loadFixtureScene(): internal type error"));
        return;
    }

    QString filename = QFileDialog::getOpenFileName(
                this,
                "Choose an file name",
                ".",
                "Scene (*.json)"
                );

    corecvs::FixtureScene *pointer = static_cast<corecvs::FixtureScene *>(rawPointer);
    delete_safe(pointer);
    pointer = new corecvs::FixtureScene;

#ifdef WITH_JSONMODERN
    typedef JSONModernReader JSONReader;
#elif defined(WITH_RAPIDJSON)
    typedef RapidJSONReader  JSONReader;
#else
    typedef JSONGetter       JSONReader;
#endif
    {
        JSONReader getter(QSTR_DATA_PTR(filename));
        if (!getter.hasError()) {
            getter.visit(*pointer, "scene");
        }
        else {
            QMessageBox::information(this, "Unable to parse json", "Unable to parse json. See log for details");
        }
    }

    rawPointer = pointer;
    setValue(rawPointer);

  /*rawPointer = BufferFactory::getInstance()->loadRGB24Bitmap(filename.toStdString());
    setValue(rawPointer);*/
}

void PointerFieldWidget::showFixtureScene()
{
    if (std::string(fieldReflection->targetClass) != "corecvs::FixtureScene" || rawPointer == NULL )
    {
        cout << "Nothing to show" << endl;
        return;
    }
    corecvs::FixtureScene *scene = static_cast<corecvs::FixtureScene *>(rawPointer);
    scene->dumpInfo();

}

void PointerFieldWidget::showFixtureCamera()
{
    if (std::string(fieldReflection->targetClass) != "corecvs::FixtureCamera" || rawPointer == NULL )
        return;
    if (cameraControl == NULL)
        cameraControl = new CameraModelParametersControlWidget;

    corecvs::FixtureCamera *camera = static_cast<corecvs::FixtureCamera *>(rawPointer);

    cameraControl->setParameters(*camera);
    cameraControl->show();
    cameraControl->raise();
}

void PointerFieldWidget::saveFixtureCamera()
{}
