#include "focusEstimator1.h"
#include "focusEstimator.h"
#include "ellipticalApproximation.h"
#include "abstractPainter.h"
#include "polygonPointIterator.h"

namespace corecvs {

FocusEstimator1::FocusEstimator1()
{}

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
    Statistics::enterContext(mStats, "FocusEstimator1");

    if (mParams.produceDebug())
    {
        delete_safe(mDebug);
        mDebug = new RGB24Buffer(mCurrent);
    }

    AbstractPainter<RGB24Buffer> painter(mDebug); /*it is safe to construct it for NULL. Not safe to use however*/

    EllipticalApproximation1d approxCenterW;
    EllipticalApproximation1d approxCenterB;
    EllipticalApproximation1d approxCenterBouter;

    EllipticalApproximation1d approxSharpness;

    Statistics::startInterval(mStats);

    if ( mParams.computeOldStats())
    {
        FocusEstimator::Result res = FocusEstimator::calc(mCurrent, mROI.left(), mROI.top(), mROI.right(), mROI.bottom());
        mResult.setScore   (res.fullScore);
        mResult.setScoreROI(res.score);
    }

    Statistics::resetInterval(mStats, "Old Style Score");


    if (!mBoards.empty())
    {
        mResult.setBoardProcessed(true);
        double pixelSize = numeric_limits<double>::max();
        //double pixelSizeW = numeric_limits<double>::max();

        for (size_t b = 0; b < mBoards.size(); b++)
        {
            BoardCornersType &board = mBoards[b];
            int boardH = (int)board.size();
            int boardW = (boardH != 0) ? (int)board.front().size() : 0;
            cout << "FocusEstimator1 \tBoard_HxW: "  << boardH << " " << boardW << endl;

            for (int i = 0; i < (int)board.size() - 1; i++)
            {
                for (int j = 0; j < (int)board[i].size() - 1; j++)
                {
                    EllipticalApproximation1d approxCenterBinner;

                    bool isWhite = !((i + j) % 2);

                    Vector2dd p = board[i][j];
                    Vector2dd v1 = board[i + 1][j    ] - board[i][j];
                    Vector2dd v2 = board[i    ][j + 1] - board[i][j];

                    double jc = mParams.junctionCoef();

                    Polygon junction;
                    Vector2dd v1j = v1 * jc;
                    Vector2dd v2j = v2 * jc;

                    junction.push_back(p - v1j - v2j);
                    junction.push_back(p + v1j - v2j);
                    junction.push_back(p + v1j + v2j);
                    junction.push_back(p - v1j + v2j);

                    double pixelCurSize = std::min(v1j.l2Metric(), v2j.l2Metric());
                    pixelSize = std::min(pixelCurSize, pixelSize);

                    Polygon center;
                    double scenter  = (1.0 - mParams.centerCoef()) / 2;
                    double ecenter  = (1.0 - scenter);

                    center.push_back(p + v1 * scenter + v2 * scenter);
                    center.push_back(p + v1 * scenter + v2 * ecenter);
                    center.push_back(p + v1 * ecenter + v2 * ecenter);
                    center.push_back(p + v1 * ecenter + v2 * scenter);

                    if (mDebug != NULL) {
                        painter.drawPolygon(junction, RGBColor::Yellow());
                        painter.drawPolygon(center, isWhite ? RGBColor::Amber() : RGBColor::Gray());
                    }

                    /* Check the uniform*/

                    PolygonPointIterator it(center);
                    for (Vector2d<int> runner : it)
                    {
                        if (!mCurrent->isValidCoord(runner))
                            continue;

                        if (isWhite) {
                            approxCenterW.addPoint(mCurrent->element(runner).yd());
                        } else {
                            approxCenterB.addPoint(mCurrent->element(runner).yd());
                            approxCenterBinner.addPoint(mCurrent->element(runner).yd());
                        }

                        if (mDebug != NULL) {
                            mDebug->element(runner).blendWith(RGBColor::Green());
                        }
                    }

                    approxCenterBouter.addPoint(approxCenterBinner.getRadius());
                }
            }

            /*if (mDebug != NULL)
            {
                for (size_t i = 0; i < board.size(); i++ )
                {
                    for (size_t j = 0; j < board[i].size(); j++ )
                    {
                        //input->draw(board[i][j], RGBColor::Blue());
                        painter.drawCircle(board[i][j], pixelSize, RGBColor::Blue());
                    }
                }
            }*/

            if (mDebug != NULL)
            {
                cout << "FocusEstimator1 \tpixelSize is :" << pixelSize << endl;
            }

            for (size_t i = 0; i < board.size(); i++ )
            {
                for (size_t j = 0; j < board[i].size(); j++ )
                {
                    Circle2d workArea(board[i][j].x(), board[i][j].y(), pixelSize);

                    if (mDebug != NULL)
                    {
                        //cout << "Processing cross:" << i << " " << j << " " << workArea << endl;
                        // painter.drawCircle(board[i][j], pixelSize, RGBColor::Blue());
                    }

                    CircleSpanIterator inner(workArea);
                    while (inner.hasValue())
                    {
                        // cout << "!\n";
                        HLineSpanInt span = inner.getSpan();
                        if (span.y() < 1 || span.y() + 1 >= mCurrent->h)
                            continue;

                        for (int s1 = std::max(span.x1, 1); s1 < std::min(span.x2, mCurrent->w - 1); s1++ )
                        {
                            if (mDebug != NULL) {
                                mDebug->element(span.y(), s1) = RGBColor::Cyan();
                            }
                            //cout << "#";

                            double sobelH = mCurrent->element(span.y()    , s1 - 1).yd() - mCurrent->element(span.y()    , s1 + 1).yd();
                            double sobelV = mCurrent->element(span.y() - 1, s1    ).yd() - mCurrent->element(span.y() + 1, s1    ).yd();

                            double edgeScore = sobelH * sobelH + sobelV * sobelV;
                            if (edgeScore > mParams.edgeThreshold()) {
                                approxSharpness.addPoint(edgeScore);
                            }
                        }
                        inner.step();
                    }
                }
            }
        } // Cycle over boards
    } // If Boards

    //mResult.setBnoise(approxCenterB.getRadius());
    mResult.setBnoise(approxCenterBouter.getMean());
    SYNC_PRINT(("FocusEstimator1::FocusEstimator1() : old = %lf new = %lf\n", approxCenterB.getRadius(), approxCenterBouter.getRadius()));

    mResult.setWnoise(approxCenterW.getRadius());
    mResult.setSharpness(approxSharpness.getMean());

    Statistics::endInterval(mStats, "Overall computation");
    Statistics::leaveContext(mStats);
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
