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
};

} //namespace corecvs
