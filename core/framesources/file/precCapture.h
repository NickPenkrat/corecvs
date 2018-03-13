#pragma once
/**
 * \file precCapture.h
 * \brief Add Comment Here
 *
 * \date Jan 9, 2015
 * \author alexander
 */
#include <QtCore/QThread>
#include <QtCore/QMutex>

#include "core/framesources/file/imageFileCaptureInterface.h"
#include "abstractFileCaptureSpinThread.h"
#include "abstractFileCapture.h"

class FilePreciseCapture : public AbstractFileCapture
{
public:
    FilePreciseCapture(const std::string &params, bool isVerbose = true, bool isRGB = true);

    virtual FramePair getFrame();

    void increaseCurrentCount() { mCurrentCount++; }

private:

    class FileSpinThread : public AbstractFileCaptureSpinThread
    {
    public:
        FileSpinThread(AbstractFileCapture *pInterface, int delay, ImageCaptureInterface::FramePair &framePair);
        virtual bool grabFramePair() override;
    };

    int mCurrentCount;
};
