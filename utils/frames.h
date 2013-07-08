#pragma once

/**
 * \file frames.h
 * \brief Add Comment Here
 *
 * \date Mar 24, 2010
 * \author alexander
 */

#include <stdint.h>

#include "global.h"

#include "g12Buffer.h"
#include "imageCaptureInterface.h"

class Frames
{
public:
    enum FrameSourceId {
        LEFT_FRAME,
        DEFAULT_FRAME = LEFT_FRAME,
        RIGHT_FRAME,
        MAX_INPUTS_NUMBER
    };


    G12Buffer *currentFrames[MAX_INPUTS_NUMBER];
    int frameCount;

    Frames();
   ~Frames();

    void fetchNewFrames(ImageCaptureInterface *input);

    G12Buffer* getCurrentFrame (FrameSourceId id)   { return currentFrames[id]; }

    /// Swaps the frame sources so, for example, left camera becomes right,
    /// and right camera becomes left. Useful when cameras are plugged in wrong order.
    void swapFrameSources(bool shouldSwap);

    uint64_t timestamp() const                      { return mTimestamp; }

    int64_t  desyncTime() const                     { return mDesyncTime; }

    uint64_t startProcessTimestamp() const          { return mStartProcessTimestamp; }

private:
    bool     mSwapped;
    uint64_t mTimestamp;
    int64_t  mDesyncTime;
    uint64_t mStartProcessTimestamp;

    /* Not allowed to call*/
    Frames(const Frames &);
    Frames& operator=(const Frames&);
};

/* EOF */
