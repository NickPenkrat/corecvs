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

#include "parametersMapper/parametersMapperMerger.h"

#include "sceneShaded.h"
#include "mesh3DScene.h"


MergerDialog::MergerDialog()
    : BaseHostDialog(),
      mIsRecording(false),
      mMergerControlWidget(NULL)
{
//    this->resize(this->width(), this->height() - 65);

}

MergerDialog::~MergerDialog()
{
    terminateCalculator();

    cleanupEventQueue();
}

void MergerDialog::initParameterWidgets()
{
    BaseHostDialog::initParameterWidgets();

    mMergerControlWidget = new MergerControlWidget(this, true, UI_NAME_RECORDER);
    dockWidget()->layout()->addWidget(mMergerControlWidget);
    mSaveableWidgets.push_back(mMergerControlWidget);

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

    mCalculator = new MergerThread();

    connect(mapper, SIGNAL(baseParametersParamsChanged(QSharedPointer<BaseParameters>))
            , static_cast<MergerThread*>(mCalculator)
            , SLOT(baseControlParametersChanged(QSharedPointer<BaseParameters>)));
    connect(mapper, SIGNAL(mergerParamsChanged(QSharedPointer<Merger>))
            , static_cast<MergerThread*>(mCalculator)
            , SLOT(mergerControlParametersChanged(QSharedPointer<Merger>)));
    connect(mapper, SIGNAL(presentationParametersParamsChanged(QSharedPointer<PresentationParameters>))
            , static_cast<MergerThread*>(mCalculator)
            , SLOT(presentationControlParametersChanged(QSharedPointer<PresentationParameters>)));

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
