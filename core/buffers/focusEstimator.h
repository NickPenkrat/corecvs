#pragma once
/**
 * \file focusEstimator.h
 * \brief a header for focusEstimator.cpp
 *
 * \ingroup cppcorefiles
 * \date Sept 21, 2015
 * \author alexander
 */

namespace corecvs {

class FocusEstimator
{
public:
    /** Returns focus score that's integral over the image
     */
    static int getScore(unsigned char* data, int h, int w, int stride, int method = 0);

#if defined(RGB24Buffer) || defined(CRGB24BUFFER_H_)
    static int getScore(RGB24Buffer* buffer)
    {
        if (buffer == NULL)
            return 0;

        G8Buffer *gray = buffer->getChannel(ImageChannel::GRAY);

        int score = getScore(gray->data, gray->h, gray->w, gray->stride);

        delete_safe(gray);

        return score;
    }
#endif

};

} //namespace corecvs
