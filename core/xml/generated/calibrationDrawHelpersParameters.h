#ifndef CALIBRATION_DRAW_HELPERS_PARAMETERS_H_
#define CALIBRATION_DRAW_HELPERS_PARAMETERS_H_
/**
 * \file calibrationDrawHelpersParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 * Generated from calibration.xml
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

/**
 * \brief Calibration Draw Helpers Parameters 
 * We have two OpenGL backends to draw. Old is without shaders, new is with shaders 
 **/
class CalibrationDrawHelpersParameters : public corecvs::BaseReflection<CalibrationDrawHelpersParameters>
{
public:
    enum FieldId {
        USE_OLD_BACKEND_ID,
        SCALE_FOR_CAMERAS_ID,
        GRID_STEP_FOR_CAMERAS_ID,
        PRINTNAMES_ID,
        BILLBOARDNAMES_ID,
        PREFER_REPROJECTED_ID,
        FORCE_KNOWN_ID,
        PRIVATECOLOR_ID,
        LARGEPOINTS_ID,
        DRAWFIXTURECAMS_ID,
        SOLIDCAMERAS_ID,
        DRAWOBSERVATIONS_ID,
        DRAWTRUELINES_ID,
        PROJECTION_RAY_LENGTH_ID,
        DRAWRAYS_ID,
        CALIBRATION_DRAW_HELPERS_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Use Old Backend 
     * We have two OpenGL backends to draw. Old is without shaders, new is with shaders 
     */
    bool mUseOldBackend;

    /** 
     * \brief Scale For Cameras 
     * Scale For Cameras 
     */
    double mScaleForCameras;

    /** 
     * \brief Grid Step For Cameras 
     * Grid Step For Cameras 
     */
    int mGridStepForCameras;

    /** 
     * \brief printNames 
     * printNames 
     */
    bool mPrintNames;

    /** 
     * \brief billboardNames 
     * billboardNames 
     */
    bool mBillboardNames;

    /** 
     * \brief Prefer Reprojected 
     * Prefer Reprojected 
     */
    bool mPreferReprojected;

    /** 
     * \brief Force Known 
     * Force Known 
     */
    bool mForceKnown;

    /** 
     * \brief privateColor 
     * privateColor 
     */
    bool mPrivateColor;

    /** 
     * \brief largePoints 
     * largePoints 
     */
    bool mLargePoints;

    /** 
     * \brief drawFixtureCams 
     * drawFixtureCams 
     */
    bool mDrawFixtureCams;

    /** 
     * \brief solidCameras 
     * solidCameras 
     */
    bool mSolidCameras;

    /** 
     * \brief drawObservations 
     * drawObservations 
     */
    bool mDrawObservations;

    /** 
     * \brief drawTrueLines 
     * drawTrueLines 
     */
    bool mDrawTrueLines;

    /** 
     * \brief Projection Ray Length 
     * Projection Ray Length 
     */
    double mProjectionRayLength;

    /** 
     * \brief drawRays 
     * drawRays 
     */
    bool mDrawRays;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool useOldBackend() const
    {
        return mUseOldBackend;
    }

    double scaleForCameras() const
    {
        return mScaleForCameras;
    }

    int gridStepForCameras() const
    {
        return mGridStepForCameras;
    }

    bool printNames() const
    {
        return mPrintNames;
    }

    bool billboardNames() const
    {
        return mBillboardNames;
    }

    bool preferReprojected() const
    {
        return mPreferReprojected;
    }

    bool forceKnown() const
    {
        return mForceKnown;
    }

    bool privateColor() const
    {
        return mPrivateColor;
    }

    bool largePoints() const
    {
        return mLargePoints;
    }

    bool drawFixtureCams() const
    {
        return mDrawFixtureCams;
    }

    bool solidCameras() const
    {
        return mSolidCameras;
    }

    bool drawObservations() const
    {
        return mDrawObservations;
    }

    bool drawTrueLines() const
    {
        return mDrawTrueLines;
    }

    double projectionRayLength() const
    {
        return mProjectionRayLength;
    }

    bool drawRays() const
    {
        return mDrawRays;
    }

    /* Section with setters */
    void setUseOldBackend(bool useOldBackend)
    {
        mUseOldBackend = useOldBackend;
    }

    void setScaleForCameras(double scaleForCameras)
    {
        mScaleForCameras = scaleForCameras;
    }

    void setGridStepForCameras(int gridStepForCameras)
    {
        mGridStepForCameras = gridStepForCameras;
    }

    void setPrintNames(bool printNames)
    {
        mPrintNames = printNames;
    }

    void setBillboardNames(bool billboardNames)
    {
        mBillboardNames = billboardNames;
    }

    void setPreferReprojected(bool preferReprojected)
    {
        mPreferReprojected = preferReprojected;
    }

    void setForceKnown(bool forceKnown)
    {
        mForceKnown = forceKnown;
    }

    void setPrivateColor(bool privateColor)
    {
        mPrivateColor = privateColor;
    }

    void setLargePoints(bool largePoints)
    {
        mLargePoints = largePoints;
    }

    void setDrawFixtureCams(bool drawFixtureCams)
    {
        mDrawFixtureCams = drawFixtureCams;
    }

    void setSolidCameras(bool solidCameras)
    {
        mSolidCameras = solidCameras;
    }

    void setDrawObservations(bool drawObservations)
    {
        mDrawObservations = drawObservations;
    }

    void setDrawTrueLines(bool drawTrueLines)
    {
        mDrawTrueLines = drawTrueLines;
    }

    void setProjectionRayLength(double projectionRayLength)
    {
        mProjectionRayLength = projectionRayLength;
    }

    void setDrawRays(bool drawRays)
    {
        mDrawRays = drawRays;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mUseOldBackend,             static_cast<const corecvs::BoolField *>(fields()[USE_OLD_BACKEND_ID]));
        visitor.visit(mScaleForCameras,           static_cast<const corecvs::DoubleField *>(fields()[SCALE_FOR_CAMERAS_ID]));
        visitor.visit(mGridStepForCameras,        static_cast<const corecvs::IntField *>(fields()[GRID_STEP_FOR_CAMERAS_ID]));
        visitor.visit(mPrintNames,                static_cast<const corecvs::BoolField *>(fields()[PRINTNAMES_ID]));
        visitor.visit(mBillboardNames,            static_cast<const corecvs::BoolField *>(fields()[BILLBOARDNAMES_ID]));
        visitor.visit(mPreferReprojected,         static_cast<const corecvs::BoolField *>(fields()[PREFER_REPROJECTED_ID]));
        visitor.visit(mForceKnown,                static_cast<const corecvs::BoolField *>(fields()[FORCE_KNOWN_ID]));
        visitor.visit(mPrivateColor,              static_cast<const corecvs::BoolField *>(fields()[PRIVATECOLOR_ID]));
        visitor.visit(mLargePoints,               static_cast<const corecvs::BoolField *>(fields()[LARGEPOINTS_ID]));
        visitor.visit(mDrawFixtureCams,           static_cast<const corecvs::BoolField *>(fields()[DRAWFIXTURECAMS_ID]));
        visitor.visit(mSolidCameras,              static_cast<const corecvs::BoolField *>(fields()[SOLIDCAMERAS_ID]));
        visitor.visit(mDrawObservations,          static_cast<const corecvs::BoolField *>(fields()[DRAWOBSERVATIONS_ID]));
        visitor.visit(mDrawTrueLines,             static_cast<const corecvs::BoolField *>(fields()[DRAWTRUELINES_ID]));
        visitor.visit(mProjectionRayLength,       static_cast<const corecvs::DoubleField *>(fields()[PROJECTION_RAY_LENGTH_ID]));
        visitor.visit(mDrawRays,                  static_cast<const corecvs::BoolField *>(fields()[DRAWRAYS_ID]));
    }

    CalibrationDrawHelpersParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    CalibrationDrawHelpersParameters(
          bool useOldBackend
        , double scaleForCameras
        , int gridStepForCameras
        , bool printNames
        , bool billboardNames
        , bool preferReprojected
        , bool forceKnown
        , bool privateColor
        , bool largePoints
        , bool drawFixtureCams
        , bool solidCameras
        , bool drawObservations
        , bool drawTrueLines
        , double projectionRayLength
        , bool drawRays
    )
    {
        mUseOldBackend = useOldBackend;
        mScaleForCameras = scaleForCameras;
        mGridStepForCameras = gridStepForCameras;
        mPrintNames = printNames;
        mBillboardNames = billboardNames;
        mPreferReprojected = preferReprojected;
        mForceKnown = forceKnown;
        mPrivateColor = privateColor;
        mLargePoints = largePoints;
        mDrawFixtureCams = drawFixtureCams;
        mSolidCameras = solidCameras;
        mDrawObservations = drawObservations;
        mDrawTrueLines = drawTrueLines;
        mProjectionRayLength = projectionRayLength;
        mDrawRays = drawRays;
    }

    bool operator ==(const CalibrationDrawHelpersParameters &other) const 
    {
        if ( !(this->mUseOldBackend == other.mUseOldBackend)) return false;
        if ( !(this->mScaleForCameras == other.mScaleForCameras)) return false;
        if ( !(this->mGridStepForCameras == other.mGridStepForCameras)) return false;
        if ( !(this->mPrintNames == other.mPrintNames)) return false;
        if ( !(this->mBillboardNames == other.mBillboardNames)) return false;
        if ( !(this->mPreferReprojected == other.mPreferReprojected)) return false;
        if ( !(this->mForceKnown == other.mForceKnown)) return false;
        if ( !(this->mPrivateColor == other.mPrivateColor)) return false;
        if ( !(this->mLargePoints == other.mLargePoints)) return false;
        if ( !(this->mDrawFixtureCams == other.mDrawFixtureCams)) return false;
        if ( !(this->mSolidCameras == other.mSolidCameras)) return false;
        if ( !(this->mDrawObservations == other.mDrawObservations)) return false;
        if ( !(this->mDrawTrueLines == other.mDrawTrueLines)) return false;
        if ( !(this->mProjectionRayLength == other.mProjectionRayLength)) return false;
        if ( !(this->mDrawRays == other.mDrawRays)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, CalibrationDrawHelpersParameters &toSave)
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
#endif  //CALIBRATION_DRAW_HELPERS_PARAMETERS_H_
