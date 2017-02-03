#include "focusEstimator1.h"
#include "focusEstimator.h"
#include "ellipticalApproximation.h"


#include "abstractPainter.h"
#include "polygonPointIterator.h"

namespace corecvs {

FocusEstimator1::FocusEstimator1()
{

}

void FocusEstimator1::setInputImage(RGB24Buffer *buffer)
{
    mCurrent = buffer;
    if (mROI.isEmpty()) {
        mROI = Rectangled(0, 0, mCurrent->w, mCurrent->h);
    }
}

void FocusEstimator1::setBoardInfo(const std::vector<BoardCornersType> &boards)
{
    mBoards = boards;
}

void FocusEstimator1::setRoi(const Rectangled &rectangle)
{
    mROI = rectangle;
}

void FocusEstimator1::setParameters(const FocusEstimationParameters &params)
{
    this->mParams = params;
}

void FocusEstimator1::setStatistics(Statistics *stats)
{
    mStats = stats;
}

void FocusEstimator1::operator ()()
{
    if (mCurrent == NULL)
    {
        SYNC_PRINT(("FocusEstimator1::operator ()(): There is no input\n"));
        return;
    }

    if (mParams.produceDebug())
    {
        delete_safe(mDebug);
        mDebug = new RGB24Buffer(mCurrent);
    }

    AbstractPainter<RGB24Buffer> painter(mDebug); /*it is safe to construct it for NULL. Not safe to use however*/

    EllipticalApproximation1d approxCenterW;
    EllipticalApproximation1d approxCenterB;


    Statistics::startInterval(mStats);

    FocusEstimator::Result res = FocusEstimator::calc(mCurrent, mROI.left(), mROI.top(), mROI.right(), mROI.bottom());
    mResult.setScore   (res.fullScore);
    mResult.setScoreROI(res.score);

    Statistics::resetInterval(mStats, "Old Style Score");


    if (!mBoards.empty())
    {
        for (size_t b = 0; b < mBoards.size(); b++)
        {
            BoardCornersType &board = mBoards[b];
            cout << "Board "  << board.size() << " " << board.front().size() << endl;

            if (mDebug != NULL)
            {
                for (size_t i = 0; i < board.size(); i++ )
                {
                    for (size_t j = 0; j < board[i].size(); j++ )
                    {
                        //input->draw(board[i][j], RGBColor::Blue());
                        painter.drawCircle(board[i][j], 10.0, RGBColor::Blue());
                    }
                }
            }

            for (size_t i = 0; i < board.size() - 1; i++ )
            {
                for (size_t j = 0; j < board[i].size() - 1; j++ )
                {
                    bool isWhite = !((i + j) % 2);


                    Vector2dd p = board[i][j];
                    Vector2dd v1 = board[i + 1][j    ] - board[i][j];
                    Vector2dd v2 = board[i    ][j + 1] - board[i][j];

                    Polygon junction;
                    double jc = mParams.junctionCoef();
                    junction.push_back(p - v1 * jc - v2 * jc);
                    junction.push_back(p + v1 * jc - v2 * jc);
                    junction.push_back(p + v1 * jc + v2 * jc);
                    junction.push_back(p - v1 * jc + v2 * jc);

                    Polygon center;
                    double scenter  = (1.0 - mParams.centerCoef()) / 2;
                    double ecenter  = (1.0 - scenter);


                    center.push_back(p + v1 * scenter + v2 * scenter);
                    center.push_back(p + v1 * scenter + v2 * ecenter);
                    center.push_back(p + v1 * ecenter + v2 * ecenter);
                    center.push_back(p + v1 * ecenter + v2 * scenter);

                    if (mDebug != NULL) {
                        painter.drawPolygon(junction, RGBColor::Yellow());
                        painter.drawPolygon(center  , isWhite ? RGBColor::Amber() : RGBColor::Gray());
                    }

                    /* Check the uniform*/

                    PolygonPointIterator it(center);
                    for (Vector2d<int> runner : it)
                    {
                        if (!mCurrent->isValidCoord(runner))
                            continue;

                        if (isWhite)
                            approxCenterW.addPoint(mCurrent->element(runner).yd());
                        else
                            approxCenterB.addPoint(mCurrent->element(runner).yd());

                         if (mDebug != NULL) {
                             mDebug->element(runner).blendWith(RGBColor::Green());
                         }
                    }



                }
            }


        }
    }

    mResult.setBnoise(approxCenterB.getRadius());
    mResult.setWnoise(approxCenterW.getRadius());


    Statistics::endInterval(mStats, "Computing Stats");



}

void FocusEstimator1::reset()
{
    mROI = Rectangled::Empty();
    mResult = FocusEstimationResult();
    mParams = FocusEstimationParameters();
    delete_safe (mPrevious);
    delete_safe (mDebug);
}

FocusEstimationResult FocusEstimator1::getResult()
{
    return mResult;
}

RGB24Buffer *FocusEstimator1::getDebug()
{
    return mDebug;
}


FocusEstimator1::~FocusEstimator1()
{
    reset();
}

} // namespace corecvs
