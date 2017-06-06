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


    //connect(mMergerControlWidget->ui()->choosePathButton, SIGNAL(clicked()), this, SLOT(openPathSelectDialog()));
    //connect(mMergerControlWidget->ui()->recStartButton, SIGNAL(clicked()), this, SLOT(toggleRecording()));

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

    mapper->paramsChanged();

    mParamsMapper = mapper;

    connect(this, SIGNAL(recordingTriggered()), mCalculator, SLOT(toggleRecording()), Qt::QueuedConnection);

    //connect(mMergerControlWidget->ui()->recRestartButton, SIGNAL(released()), mCalculator, SLOT(resetRecording()), Qt::QueuedConnection);
    //connect(mMergerControlWidget->ui()->recPauseButton, SIGNAL(released()), mCalculator, SLOT(toggleRecording()), Qt::QueuedConnection);

    connect(mCalculator, SIGNAL(recordingStateChanged(MergerThread::RecordingState)), this,
            SLOT(recordingStateChanged(MergerThread::RecordingState)), Qt::QueuedConnection);

    connect(mCalculator, SIGNAL(errorMessage(QString)),
            this,        SLOT  (errorMessage(QString)), Qt::BlockingQueuedConnection);

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
            mImage = QSharedPointer<QImage>(new QImage(fod->mMainImage.width(), fod->mMainImage.height(),  QImage::Format_RGB32));
            fod->mMainImage.drawImage(mImage.data());
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
