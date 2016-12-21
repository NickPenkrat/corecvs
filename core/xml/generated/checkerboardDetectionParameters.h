#ifndef CHECKERBOARD_DETECTION_PARAMETERS_H_
#define CHECKERBOARD_DETECTION_PARAMETERS_H_
/**
 * \file checkerboardDetectionParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "reflection.h"
#include "defaultSetter.h"
#include "printerVisitor.h"

/*
 *  Embed includes.
 */
/*
 *  Additional includes for Composite Types.
 */

// using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

// namespace corecvs {
// }
/*
 *  Additional includes for enum section.
 */
#include "checkerboardDetectionAlgorithm.h"
#include "imageChannel.h"

/**
 * \brief Checkerboard Detection Parameters 
 * Checkerboard Detection Parameters 
 **/
class CheckerboardDetectionParameters : public corecvs::BaseReflection<CheckerboardDetectionParameters>
{
public:
    enum FieldId {
        ESTIMATE_UNDISTORTED_FROM_DISTORTED_ID,
        USE_UNDISTORTION_ID,
        ALGORITHM_ID,
        CHANNEL_ID,
        CELLSIZEHOR_ID,
        CELLSIZEVERT_ID,
        CLEANEXISTING_ID,
        PRECISEDIAMETER_ID,
        ITERATIONCOUNT_ID,
        MINACCURACY_ID,
        PARTIALBOARD_ID,
        FASTBOARDSPEEDUP_ID,
        DRAW_SGFS_ON_BOARDS_ID,
        SKIP_UNDISTORTED_WITH_NO_DISTORTED_BOARD_ID,
        CHECKERBOARD_DETECTION_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Estimate undistorted from distorted 
     * Estimate undistorted from distorted 
     */
    bool mEstimateUndistortedFromDistorted;

    /** 
     * \brief Use Undistortion 
     * Use Undistortion 
     */
    bool mUseUndistortion;

    /** 
     * \brief Algorithm 
     * Algorithm 
     */
    int mAlgorithm;

    /** 
     * \brief Channel 
     * Channel 
     */
    int mChannel;

    /** 
     * \brief cellSizeHor 
     * cellSizeHor 
     */
    double mCellSizeHor;

    /** 
     * \brief cellSizeVert 
     * cellSizeVert 
     */
    double mCellSizeVert;

    /** 
     * \brief cleanExisting 
     * cleanExisting 
     */
    bool mCleanExisting;

    /** 
     * \brief preciseDiameter 
     * preciseDiameter 
     */
    int mPreciseDiameter;

    /** 
     * \brief iterationCount 
     * iterationCount 
     */
    int mIterationCount;

    /** 
     * \brief minAccuracy 
     * minAccuracy 
     */
    double mMinAccuracy;

    /** 
     * \brief partialBoard 
     * partialBoard 
     */
    bool mPartialBoard;

    /** 
     * \brief fastBoardSpeedup 
     * fastBoardSpeedup 
     */
    bool mFastBoardSpeedup;

    /** 
     * \brief Draw SGFs on boards 
     * Draw SGFs on boards 
     */
    bool mDrawSGFsOnBoards;

    /** 
     * \brief Skip undistorted with no distorted board 
     * Skip undistorted with no distorted board 
     */
    bool mSkipUndistortedWithNoDistortedBoard;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool estimateUndistortedFromDistorted() const
    {
        return mEstimateUndistortedFromDistorted;
    }

    bool useUndistortion() const
    {
        return mUseUndistortion;
    }

    CheckerboardDetectionAlgorithm::CheckerboardDetectionAlgorithm algorithm() const
    {
        return static_cast<CheckerboardDetectionAlgorithm::CheckerboardDetectionAlgorithm>(mAlgorithm);
    }

    ImageChannel::ImageChannel channel() const
    {
        return static_cast<ImageChannel::ImageChannel>(mChannel);
    }

    double cellSizeHor() const
    {
        return mCellSizeHor;
    }

    double cellSizeVert() const
    {
        return mCellSizeVert;
    }

    bool cleanExisting() const
    {
        return mCleanExisting;
    }

    int preciseDiameter() const
    {
        return mPreciseDiameter;
    }

    int iterationCount() const
    {
        return mIterationCount;
    }

    double minAccuracy() const
    {
        return mMinAccuracy;
    }

    bool partialBoard() const
    {
        return mPartialBoard;
    }

    bool fastBoardSpeedup() const
    {
        return mFastBoardSpeedup;
    }

    bool drawSGFsOnBoards() const
    {
        return mDrawSGFsOnBoards;
    }

    bool skipUndistortedWithNoDistortedBoard() const
    {
        return mSkipUndistortedWithNoDistortedBoard;
    }

    /* Section with setters */
    void setEstimateUndistortedFromDistorted(bool estimateUndistortedFromDistorted)
    {
        mEstimateUndistortedFromDistorted = estimateUndistortedFromDistorted;
    }

    void setUseUndistortion(bool useUndistortion)
    {
        mUseUndistortion = useUndistortion;
    }

    void setAlgorithm(CheckerboardDetectionAlgorithm::CheckerboardDetectionAlgorithm algorithm)
    {
        mAlgorithm = algorithm;
    }

    void setChannel(ImageChannel::ImageChannel channel)
    {
        mChannel = channel;
    }

    void setCellSizeHor(double cellSizeHor)
    {
        mCellSizeHor = cellSizeHor;
    }

    void setCellSizeVert(double cellSizeVert)
    {
        mCellSizeVert = cellSizeVert;
    }

    void setCleanExisting(bool cleanExisting)
    {
        mCleanExisting = cleanExisting;
    }

    void setPreciseDiameter(int preciseDiameter)
    {
        mPreciseDiameter = preciseDiameter;
    }

    void setIterationCount(int iterationCount)
    {
        mIterationCount = iterationCount;
    }

    void setMinAccuracy(double minAccuracy)
    {
        mMinAccuracy = minAccuracy;
    }

    void setPartialBoard(bool partialBoard)
    {
        mPartialBoard = partialBoard;
    }

    void setFastBoardSpeedup(bool fastBoardSpeedup)
    {
        mFastBoardSpeedup = fastBoardSpeedup;
    }

    void setDrawSGFsOnBoards(bool drawSGFsOnBoards)
    {
        mDrawSGFsOnBoards = drawSGFsOnBoards;
    }

    void setSkipUndistortedWithNoDistortedBoard(bool skipUndistortedWithNoDistortedBoard)
    {
        mSkipUndistortedWithNoDistortedBoard = skipUndistortedWithNoDistortedBoard;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mEstimateUndistortedFromDistorted, static_cast<const corecvs::BoolField *>(fields()[ESTIMATE_UNDISTORTED_FROM_DISTORTED_ID]));
        visitor.visit(mUseUndistortion,           static_cast<const corecvs::BoolField *>(fields()[USE_UNDISTORTION_ID]));
        visitor.visit((int &)mAlgorithm,          static_cast<const corecvs::EnumField *>(fields()[ALGORITHM_ID]));
        visitor.visit((int &)mChannel,            static_cast<const corecvs::EnumField *>(fields()[CHANNEL_ID]));
        visitor.visit(mCellSizeHor,               static_cast<const corecvs::DoubleField *>(fields()[CELLSIZEHOR_ID]));
        visitor.visit(mCellSizeVert,              static_cast<const corecvs::DoubleField *>(fields()[CELLSIZEVERT_ID]));
        visitor.visit(mCleanExisting,             static_cast<const corecvs::BoolField *>(fields()[CLEANEXISTING_ID]));
        visitor.visit(mPreciseDiameter,           static_cast<const corecvs::IntField *>(fields()[PRECISEDIAMETER_ID]));
        visitor.visit(mIterationCount,            static_cast<const corecvs::IntField *>(fields()[ITERATIONCOUNT_ID]));
        visitor.visit(mMinAccuracy,               static_cast<const corecvs::DoubleField *>(fields()[MINACCURACY_ID]));
        visitor.visit(mPartialBoard,              static_cast<const corecvs::BoolField *>(fields()[PARTIALBOARD_ID]));
        visitor.visit(mFastBoardSpeedup,          static_cast<const corecvs::BoolField *>(fields()[FASTBOARDSPEEDUP_ID]));
        visitor.visit(mDrawSGFsOnBoards,          static_cast<const corecvs::BoolField *>(fields()[DRAW_SGFS_ON_BOARDS_ID]));
        visitor.visit(mSkipUndistortedWithNoDistortedBoard, static_cast<const corecvs::BoolField *>(fields()[SKIP_UNDISTORTED_WITH_NO_DISTORTED_BOARD_ID]));
    }

    CheckerboardDetectionParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    CheckerboardDetectionParameters(
          bool estimateUndistortedFromDistorted
        , bool useUndistortion
        , CheckerboardDetectionAlgorithm::CheckerboardDetectionAlgorithm algorithm
        , ImageChannel::ImageChannel channel
        , double cellSizeHor
        , double cellSizeVert
        , bool cleanExisting
        , int preciseDiameter
        , int iterationCount
        , double minAccuracy
        , bool partialBoard
        , bool fastBoardSpeedup
        , bool drawSGFsOnBoards
        , bool skipUndistortedWithNoDistortedBoard
    )
    {
        mEstimateUndistortedFromDistorted = estimateUndistortedFromDistorted;
        mUseUndistortion = useUndistortion;
        mAlgorithm = algorithm;
        mChannel = channel;
        mCellSizeHor = cellSizeHor;
        mCellSizeVert = cellSizeVert;
        mCleanExisting = cleanExisting;
        mPreciseDiameter = preciseDiameter;
        mIterationCount = iterationCount;
        mMinAccuracy = minAccuracy;
        mPartialBoard = partialBoard;
        mFastBoardSpeedup = fastBoardSpeedup;
        mDrawSGFsOnBoards = drawSGFsOnBoards;
        mSkipUndistortedWithNoDistortedBoard = skipUndistortedWithNoDistortedBoard;
    }

    friend std::ostream& operator << (std::ostream &out, CheckerboardDetectionParameters &toSave)
    {
        corecvs::PrinterVisitor printer(out);
        toSave.accept<corecvs::PrinterVisitor>(printer);
        return out;
    }

    void print ()
    {
        std::cout << *this;
    }
};
#endif  //CHECKERBOARD_DETECTION_PARAMETERS_H_
