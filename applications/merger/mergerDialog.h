#ifndef _RECORDER_DIALOG_H_
#define _RECORDER_DIALOG_H_

/**
 * \file mergerDialog.h
 * \brief Defines a merger dialog class derived from BaseHostDialog
 *
 * \date Sep 17, 2010
 * \author Sergey Levi
 */

#include <QString>

#include "core/utils/global.h"

#include "baseHostDialog.h"
#include "mergerThread.h"
#include "mergerControlWidget.h"
#include "cloudViewDialog.h"

#include "cameraModelParametersControlWidget.h"

#define UI_NAME_RECORDER "merger"

class MergerDialog : public BaseHostDialog
{
    Q_OBJECT

public:

    MergerDialog();
    ~MergerDialog();


    virtual void initParameterWidgets() override;
    virtual void initCommon() override;
    virtual void createCalculator() override;
    virtual void connectFinishedRecalculation();

    virtual CamerasConfigParameters *getAdditionalParams() const override;
signals:
    void recordingTriggered();
    void recordingReset();

    void newCarScene(QSharedPointer<FixtureScene> scene);

public slots:
    void mergerControlParametersChanged(QSharedPointer<Merger> params);

    virtual void processResult();

    void errorMessage(QString message);

    /* */
    void virtualCamsChanged();

private:
    bool mIsRecording;

    MergerControlWidget *mMergerControlWidget;

    CameraModelParametersControlWidget* mCams[4];

    QSharedPointer<Merger> mMergerControlParams;

    /* These slots are for silencing error, created due current to misdesign */
public slots:
    void showHistogram();
    void show3DHistogram();
    void showRectificationDialog();
    void doRectify();
    void resetRectification();

public:

    /* We should protect FixtureScene by Mutex... etc... nobody cares. */
    QSharedPointer<FixtureScene> mMainScene;


    AdvancedImageWidget *mAdditionalFeed = NULL;
    AdvancedImageWidget *mUnwarpedFeed   = NULL;
    CloudViewDialog *m3DView             = NULL;

    virtual void createAdditionalWindows();
};

#endif // _RECORDER_DIALOG_H_
