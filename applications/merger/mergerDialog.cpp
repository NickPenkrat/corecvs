/**
 * \file mergerDialog.cpp
 * \brief Implements frame recording dialog based on BaseHostDialog
 *
 * \date Sep 17, 2010
 * \author Sergey Levi
 */

#include "mergerDialog.h"

#include <stdio.h>
#include <QLayout>
#include <QDir>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <foldableWidget.h>


#include "parametersMapper/parametersMapperMerger.h"

#include "sceneShaded.h"
#include "mesh3DScene.h"

#include <core/camerafixture/cameraFixture.h>

// FIXME
#include "ui_cameraModelParametersControlWidget.h"


MergerDialog::MergerDialog()
    : BaseHostDialog(),
      mIsRecording(false),
      mMergerControlWidget(NULL)
{
//    this->resize(this->width(), this->height() - 65);

    {
        mMainScene = QSharedPointer<FixtureScene>(new FixtureScene);

        JSONGetter getter("topview.json");

        if (!getter.hasError())
        {
            getter.visit(*mMainScene.data(), "scene");
            mMainScene->dumpInfo();
            SYNC_PRINT(("MergerDialog::MergerDialog(): koefs %d\n", mMainScene->fixtures()[0]->cameras[0]->distortion.mKoeff.size()));

        } else {

            CameraFixture *body = mMainScene->createCameraFixture();
            body->setLocation(Affine3DQ::Identity());
            body->name = "Car Body";

            int DEFAULT_H = 480;
            int DEFAULT_W = 640;
            Vector2dd size(DEFAULT_W, DEFAULT_H);

            PinholeCameraIntrinsics pinhole1(size, degToRad(60));
            PinholeCameraIntrinsics pinhole2(size, degToRad(60));
            PinholeCameraIntrinsics pinhole3(size, degToRad(60));
            PinholeCameraIntrinsics pinhole4(size, degToRad(60));

            FixtureCamera *frontCam = mMainScene->createCamera(); mMainScene->addCameraToFixture(frontCam, body);
            FixtureCamera *rightCam = mMainScene->createCamera(); mMainScene->addCameraToFixture(rightCam, body);
            FixtureCamera *backCam  = mMainScene->createCamera(); mMainScene->addCameraToFixture(backCam , body);
            FixtureCamera *leftCam  = mMainScene->createCamera(); mMainScene->addCameraToFixture(leftCam , body);

            frontCam->intrinsics.reset(pinhole1.clone()); // frontCam->distortion = inverted.mParams;
            rightCam->intrinsics.reset(pinhole2.clone()); // rightCam->distortion = inverted.mParams;
            backCam ->intrinsics.reset(pinhole3.clone()); // backCam ->distortion = inverted.mParams;
            leftCam ->intrinsics.reset(pinhole4.clone()); // leftCam ->distortion = inverted.mParams;

            frontCam->nameId = "Front";
            rightCam->nameId = "Right";
            backCam ->nameId = "Back";
            leftCam ->nameId = "Left";
        }

    }


}

MergerDialog::~MergerDialog()
{
    {
        SYNC_PRINT(("MergerDialog::~MergerDialog(): koefs %d\n", mMainScene->fixtures()[0]->cameras[0]->distortion.mKoeff.size()));
        JSONSetter saver("topview.json");        
        saver.visit(*mMainScene.data(), "scene");
        SYNC_PRINT(("MergerDialog::~MergerDialog(): koefs %d\n", mMainScene->fixtures()[0]->cameras[0]->distortion.mKoeff.size()));
    }

    terminateCalculator();

    cleanupEventQueue();
}

void MergerDialog::initParameterWidgets()
{
    BaseHostDialog::initParameterWidgets();

    mMergerControlWidget = new MergerControlWidget(this, true, UI_NAME_RECORDER);
    FoldableWidget *w = new FoldableWidget(this, "Merger", mMergerControlWidget);
    dockWidget()->layout()->addWidget(w);
    mSaveableWidgets.push_back(mMergerControlWidget);

    CameraFixture *carBody = mMainScene->fixtures().front();

    for (int i = 0; i < 4; i++)
    {
        mCams[i] = new CameraModelParametersControlWidget();
        //mCams[i]->ui->distortionFrame->hide();
        //mCams[i]->ui->projectionFrame->hide();
        mCams[i]->ui->projectionTypeFrame->hide();


        FoldableWidget *wc = new FoldableWidget(this, QString("cam %1").arg(carBody->cameras[i]->nameId.c_str()), mCams[i]);
        dockWidget()->layout()->addWidget(wc);

        //mSaveableWidgets.push_back(mCams[i]);

        mCams[i]->setParameters(*carBody->cameras[i]);
        QObject::connect(mCams[i], SIGNAL(paramsChanged()), this, SLOT(virtualCamsChanged()));

    }
}

void MergerDialog::initCommon()
{
    BaseHostDialog::initCommon();
    createAdditionalWindows();
}


CamerasConfigParameters * MergerDialog::getAdditionalParams() const
{
    CamerasConfigParameters  *fp = new CamerasConfigParameters ();

    fp->setInputsN((CamerasConfigParameters::CaptureDevicesNum) 4);

    fp->setRectifierData(mRectifierData);

    fp->setDistortionTransform(mDistortionTransform);

    return fp;
}


void MergerDialog::createCalculator()
{
    ParametersMapperMerger *mapper = new ParametersMapperMerger();

    mapper->setMergerControlWidget(mMergerControlWidget);
    mapper->setBaseParametersControlWidget(mBaseControlWidget);
    mapper->setPresentationParametersControlWidget(mPresentationControlWidget);

    connect(mapper, SIGNAL(mergerParamsChanged(QSharedPointer<Merger>))
            , this, SLOT(mergerControlParametersChanged(QSharedPointer<Merger>)));

  /*  connect(mapper, SIGNAL(baseParametersParamsChanged(QSharedPointer<BaseParameters>))
            , this, SLOT(baseControlParametersChanged(QSharedPointer<Base>)));*/

    MergerThread *mergerThread = new MergerThread();
    mCalculator = mergerThread;


    connect(mapper, SIGNAL(baseParametersParamsChanged(QSharedPointer<BaseParameters>))
            , mergerThread
            , SLOT(baseControlParametersChanged(QSharedPointer<BaseParameters>)));
    connect(mapper, SIGNAL(mergerParamsChanged(QSharedPointer<Merger>))
            , mergerThread
            , SLOT(mergerControlParametersChanged(QSharedPointer<Merger>)));
    connect(mapper, SIGNAL(presentationParametersParamsChanged(QSharedPointer<PresentationParameters>))
            , mergerThread
            , SLOT(presentationControlParametersChanged(QSharedPointer<PresentationParameters>)));


   qRegisterMetaType<QSharedPointer<FixtureScene>>("QSharedPointer<FixtureScene>");

    connect(this, SIGNAL(newCarScene(QSharedPointer<FixtureScene>))
            , mergerThread
            , SLOT(sceneParametersChanged(QSharedPointer<FixtureScene>)));

    emit newCarScene(mMainScene);

    connect(mMergerControlWidget, SIGNAL(saveRemap(QString)),
      static_cast<MergerThread*>(mCalculator), SLOT(saveRemap(QString)));

    mapper->paramsChanged();

    mParamsMapper = mapper;

    connect(mCalculator, SIGNAL(errorMessage(QString)),
            this,        SLOT  (errorMessage(QString)), Qt::BlockingQueuedConnection);

}

void MergerDialog::createAdditionalWindows()
{
    qDebug("MergerDialog::createAdditionalWindows()");

    mAdditionalFeed = dynamic_cast<AdvancedImageWidget *>(createAdditionalWindow("Additional feed", imageWindow, QIcon(":/new/subwindows/film_add.png")));
    mUnwarpedFeed   = dynamic_cast<AdvancedImageWidget *>(createAdditionalWindow("Unwarped   feed", imageWindow, QIcon(":/new/subwindows/film_add.png")));

    m3DView = dynamic_cast<CloudViewDialog *>(createAdditionalWindow("3D", oglWindow, QIcon(":/new/subwindows/flood_it.png")));
}



void MergerDialog::mergerControlParametersChanged(QSharedPointer<Merger> params)
{
    if (!params)
        return;

    mMergerControlParams = params;
}

void MergerDialog::connectFinishedRecalculation()
{
    MergerThread *calculator = dynamic_cast<MergerThread *>(mCalculator);

    if (calculator)
    {
        connect(calculator, SIGNAL(processingFinished(AbstractOutputData *)),
                this, SLOT(finishedRecalculation(AbstractOutputData *)), Qt::QueuedConnection);
        play();
    }
}

void MergerDialog::processResult()
{

    for (unsigned i = 0; i < eventList.size(); i++)
    {
        MergerOutputData *fod = dynamic_cast<MergerOutputData*>(eventList[i]);
        if (fod == NULL)
            break;

        mStatsDialog.addStats(fod->stats);

//        fod->mMainImage.print();

        if (i == eventList.size() - 1) {


            QSharedPointer<QImage> inputImages = QSharedPointer<QImage>(new QImage(fod->mMainImage.width(), fod->mMainImage.height(),  QImage::Format_RGB32));
            fod->mMainImage.drawImage(inputImages.data());
            mAdditionalFeed->setImage(inputImages);

            if (fod->mainOutput) {
                mImage = QSharedPointer<QImage>(new RGB24Image(fod->mainOutput));
            }
            if (fod->unwarpOutput) {
                mUnwarpedFeed->setImage(QSharedPointer<QImage>(new RGB24Image(fod->unwarpOutput)));
            }

            if (fod->visualisation != NULL)
            {

                QSharedPointer<Mesh3DScene> mScene = QSharedPointer<Mesh3DScene>(new Mesh3DScene());
                mScene->switchColor(true);
                mScene->add(*fod->visualisation, true);
                //fod->visualisation = NULL; /* transftering ownership */
                m3DView->setNewScenePointer(mScene, CloudViewDialog::MAIN_SCENE);

            }
        }

        delete fod;
    }
    updateWidgets();
}

void MergerDialog::errorMessage(QString message)
{
    QMessageBox msgBox;
    msgBox.setText(message);
    msgBox.exec();
}

void MergerDialog::virtualCamsChanged()
{
    SYNC_PRINT(("MergerDialog::virtualCamsChanged(): called"));
    for (int i = 0; i < 4; i++)
    {
        CameraFixture *carBody = mMainScene->fixtures().front();

        CameraModel model;
        mCams[i]->getParameters(model);
        carBody->cameras[i]->copyModelFrom(model);
    }
    emit newCarScene(mMainScene);
}

void MergerDialog::showHistogram()
{
}

void MergerDialog::show3DHistogram()
{
}

void MergerDialog::showRectificationDialog()
{
}

void MergerDialog::doRectify()
{
}

void MergerDialog::resetRectification()
{
}
