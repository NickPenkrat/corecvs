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

#include "global.h"

#include "baseHostDialog.h"
#include "mergerThread.h"
#include "mergerControlWidget.h"

#define UI_NAME_RECORDER "merger"

class MergerDialog : public BaseHostDialog
{
    Q_OBJECT

public:

    MergerDialog();
    ~MergerDialog();


    virtual void initParameterWidgets();
    virtual void createCalculator();
    virtual void connectFinishedRecalculation();

signals:
    void recordingTriggered();
    void recordingReset();

public slots:
    void mergerControlParametersChanged(QSharedPointer<Merger> params);

    virtual void processResult();

    void errorMessage(QString message);
private:
    bool mIsRecording;

    MergerControlWidget *mMergerControlWidget;

    QSharedPointer<Merger> mMergerControlParams;

    /* These slots are for silencing error, created due current to misdesign */
public slots:
    void showHistogram();
    void show3DHistogram();
    void showRectificationDialog();
    void doRectify();
    void resetRectification();

};

#endif // _RECORDER_DIALOG_H_
