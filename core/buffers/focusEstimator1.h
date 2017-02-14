#ifndef FOCUSESTIMATOR1_H
#define FOCUSESTIMATOR1_H

#include <vector>

#include "rgb24Buffer.h"
#include "chessBoardDetector.h"
#include "generated/focusEstimationParameters.h"
#include "generated/focusEstimationResult.h"
#include "calculationStats.h"

namespace corecvs {


class FocusEstimator1
{
public:
    FocusEstimator1();

    /* Forms the state of the estimator. Pointer data should remain valid untill operator() is called and exited */
    void setInputImage(RGB24Buffer* buffer);
    void setBoardInfo(const std::vector<BoardCornersType> &boards);
    void setRoi(const Rectangled &rectangle);
    void setParameters(const FocusEstimationParameters &params);
    void setStatistics(Statistics *stats);

    void operator ()();

    void reset(void);

    FocusEstimationResult getResult();
    RGB24Buffer *getDebug(); /**< Debug is debug. Could be reallocated on reset and operator() */

    ~FocusEstimator1();

private:
    RGB24Buffer                    *mPrevious = NULL;  /* Owned */
    RGB24Buffer                    *mDebug = NULL;     /* Owned */

    FocusEstimationResult           mResult;

    Rectangled                      mROI = Rectangled::Empty();
    RGB24Buffer                    *mCurrent = NULL;    /* Not owned */
    Statistics                     *mStats    = NULL;

    std::vector<BoardCornersType>   mBoards;

public:
    FocusEstimationParameters       mParams;

};

} // namespace corecvs

#endif // FOCUSESTIMATOR1_H
