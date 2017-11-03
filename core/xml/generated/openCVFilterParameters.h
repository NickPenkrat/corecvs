#ifndef OPENCV_FILTER_PARAMETERS_H_
#define OPENCV_FILTER_PARAMETERS_H_
/**
 * \file openCVFilterParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "core/reflection/reflection.h"
#include "core/reflection/defaultSetter.h"
#include "core/reflection/printerVisitor.h"

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
#include "core/xml/generated/openCVBinaryFilterType.h"

/**
 * \brief OpenCV Filter Parameters 
 * OpenCV Filter Parameters 
 **/
class OpenCVFilterParameters : public corecvs::BaseReflection<OpenCVFilterParameters>
{
public:
    enum FieldId {
        OPENCVFILTER_ID,
        PARAM1_ID,
        PARAM2_ID,
        OPENCV_FILTER_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief OpenCVFilter 
     * OpenCVFilter 
     */
    int mOpenCVFilter;

    /** 
     * \brief Param1 
     * Param1 
     */
    int mParam1;

    /** 
     * \brief Param2 
     * Param2 
     */
    int mParam2;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    OpenCVBinaryFilterType::OpenCVBinaryFilterType openCVFilter() const
    {
        return static_cast<OpenCVBinaryFilterType::OpenCVBinaryFilterType>(mOpenCVFilter);
    }

    int param1() const
    {
        return mParam1;
    }

    int param2() const
    {
        return mParam2;
    }

    /* Section with setters */
    void setOpenCVFilter(OpenCVBinaryFilterType::OpenCVBinaryFilterType openCVFilter)
    {
        mOpenCVFilter = openCVFilter;
    }

    void setParam1(int param1)
    {
        mParam1 = param1;
    }

    void setParam2(int param2)
    {
        mParam2 = param2;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit((int &)mOpenCVFilter,       static_cast<const corecvs::EnumField *>(fields()[OPENCVFILTER_ID]));
        visitor.visit(mParam1,                    static_cast<const corecvs::IntField *>(fields()[PARAM1_ID]));
        visitor.visit(mParam2,                    static_cast<const corecvs::IntField *>(fields()[PARAM2_ID]));
    }

    OpenCVFilterParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    OpenCVFilterParameters(
          OpenCVBinaryFilterType::OpenCVBinaryFilterType openCVFilter
        , int param1
        , int param2
    )
    {
        mOpenCVFilter = openCVFilter;
        mParam1 = param1;
        mParam2 = param2;
    }

    friend std::ostream& operator << (std::ostream &out, OpenCVFilterParameters &toSave)
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
#endif  //OPENCV_FILTER_PARAMETERS_H_
