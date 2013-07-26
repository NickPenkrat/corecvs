#ifndef OPENCV_BM_PARAMETERS_H_
#define OPENCV_BM_PARAMETERS_H_
/**
 * \file openCVBMParameters.h
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

using namespace core3vi;

/*
 *  Additional includes for Pointer Types.
 */

namespace core3vi {
}
/*
 *  Additional includes for enum section.
 */

/**
 * \brief OpenCV BM Parameters Class 
 * OpenCV BM Parameters Class 
 **/
class OpenCVBMParameters : public BaseReflection<OpenCVBMParameters>
{
public:
    enum FieldId {
        BLOCK_SIZE_ID,
        DISPARITY_SEARCH_ID,
        PREFILTERCAP_ID,
        MINDISPARITY_ID,
        TEXTURETHRESHOLD_ID,
        UNIQUENESSRATIO_ID,
        SPECKLEWINDOWSIZE_ID,
        SPECKLERANGE_ID,
        DISP12MAXDIFF_ID,
        OPENCV_BM_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Block Size 
     * Block Size 
     */
    int mBlockSize;

    /** 
     * \brief Disparity search 
     * Disparity search 
     */
    int mDisparitySearch;

    /** 
     * \brief preFilterCap 
     * preFilterCap 
     */
    int mPreFilterCap;

    /** 
     * \brief minDisparity 
     * minDisparity 
     */
    int mMinDisparity;

    /** 
     * \brief textureThreshold 
     * textureThreshold 
     */
    int mTextureThreshold;

    /** 
     * \brief uniquenessRatio 
     * uniquenessRatio 
     */
    int mUniquenessRatio;

    /** 
     * \brief speckleWindowSize 
     * speckleWindowSize 
     */
    int mSpeckleWindowSize;

    /** 
     * \brief speckleRange 
     * speckleRange 
     */
    int mSpeckleRange;

    /** 
     * \brief disp12MaxDiff 
     * disp12MaxDiff 
     */
    int mDisp12MaxDiff;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    int blockSize() const
    {
        return mBlockSize;
    }

    int disparitySearch() const
    {
        return mDisparitySearch;
    }

    int preFilterCap() const
    {
        return mPreFilterCap;
    }

    int minDisparity() const
    {
        return mMinDisparity;
    }

    int textureThreshold() const
    {
        return mTextureThreshold;
    }

    int uniquenessRatio() const
    {
        return mUniquenessRatio;
    }

    int speckleWindowSize() const
    {
        return mSpeckleWindowSize;
    }

    int speckleRange() const
    {
        return mSpeckleRange;
    }

    int disp12MaxDiff() const
    {
        return mDisp12MaxDiff;
    }

    /* Section with setters */
    void setBlockSize(int blockSize)
    {
        mBlockSize = blockSize;
    }

    void setDisparitySearch(int disparitySearch)
    {
        mDisparitySearch = disparitySearch;
    }

    void setPreFilterCap(int preFilterCap)
    {
        mPreFilterCap = preFilterCap;
    }

    void setMinDisparity(int minDisparity)
    {
        mMinDisparity = minDisparity;
    }

    void setTextureThreshold(int textureThreshold)
    {
        mTextureThreshold = textureThreshold;
    }

    void setUniquenessRatio(int uniquenessRatio)
    {
        mUniquenessRatio = uniquenessRatio;
    }

    void setSpeckleWindowSize(int speckleWindowSize)
    {
        mSpeckleWindowSize = speckleWindowSize;
    }

    void setSpeckleRange(int speckleRange)
    {
        mSpeckleRange = speckleRange;
    }

    void setDisp12MaxDiff(int disp12MaxDiff)
    {
        mDisp12MaxDiff = disp12MaxDiff;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mBlockSize,                 static_cast<const IntField *>     (fields()[BLOCK_SIZE_ID]));
        visitor.visit(mDisparitySearch,           static_cast<const IntField *>     (fields()[DISPARITY_SEARCH_ID]));
        visitor.visit(mPreFilterCap,              static_cast<const IntField *>     (fields()[PREFILTERCAP_ID]));
        visitor.visit(mMinDisparity,              static_cast<const IntField *>     (fields()[MINDISPARITY_ID]));
        visitor.visit(mTextureThreshold,          static_cast<const IntField *>     (fields()[TEXTURETHRESHOLD_ID]));
        visitor.visit(mUniquenessRatio,           static_cast<const IntField *>     (fields()[UNIQUENESSRATIO_ID]));
        visitor.visit(mSpeckleWindowSize,         static_cast<const IntField *>     (fields()[SPECKLEWINDOWSIZE_ID]));
        visitor.visit(mSpeckleRange,              static_cast<const IntField *>     (fields()[SPECKLERANGE_ID]));
        visitor.visit(mDisp12MaxDiff,             static_cast<const IntField *>     (fields()[DISP12MAXDIFF_ID]));
    }

    OpenCVBMParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    OpenCVBMParameters(
          int blockSize
        , int disparitySearch
        , int preFilterCap
        , int minDisparity
        , int textureThreshold
        , int uniquenessRatio
        , int speckleWindowSize
        , int speckleRange
        , int disp12MaxDiff
    )
    {
        mBlockSize = blockSize;
        mDisparitySearch = disparitySearch;
        mPreFilterCap = preFilterCap;
        mMinDisparity = minDisparity;
        mTextureThreshold = textureThreshold;
        mUniquenessRatio = uniquenessRatio;
        mSpeckleWindowSize = speckleWindowSize;
        mSpeckleRange = speckleRange;
        mDisp12MaxDiff = disp12MaxDiff;
    }

    friend ostream& operator << (ostream &out, OpenCVBMParameters &toSave)
    {
        PrinterVisitor printer(out);
        toSave.accept<PrinterVisitor>(printer);
        return out;
    }

    void print ()
    {
        cout << *this;
    }
};
#endif  //OPENCV_BM_PARAMETERS_H_