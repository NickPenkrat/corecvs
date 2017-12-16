#ifndef RECTIFY_PARAMETERS_H_
#define RECTIFY_PARAMETERS_H_
/**
 * \file rectifyParameters.h
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
#include "matchingMethodType.h"
#include "estimationMethodType.h"
#include "optimizationMethodType.h"

/**
 * \brief Rectify Parameters 
 * Rectify Parameters 
 **/
class RectifyParameters : public corecvs::BaseReflection<RectifyParameters>
{
public:
    enum FieldId {
        MATCHINGMETHOD_ID,
        HESSIANTHRESHOLD_ID,
        OCTAVES_ID,
        OCTAVELAYERS_ID,
        EXTENDED_ID,
        FILTERMINIMUMLENGTH_ID,
        USEKLT_ID,
        COMPUTEESSENTIAL_ID,
        PRIORFOCAL_ID,
        PRIORFOCAL2_ID,
        BASELINELENGTH_ID,
        FOVANGLE_ID,
        ESTIMATIONMETHOD_ID,
        NORMALISE_ID,
        RANSACITERATIONS_ID,
        RANSACTESTSIZE_ID,
        RANSACTHRESHOLD_ID,
        BASELINEX_ID,
        BASELINEY_ID,
        BASELINEZ_ID,
        ITERATIVEMETHOD_ID,
        ITERATIVEITERATIONS_ID,
        ITERATIVEINITIALSIGMA_ID,
        ITERATIVEFACTORSIGMA_ID,
        MANUALX_ID,
        MANUALY_ID,
        MANUALZ_ID,
        MANUALPITCH_ID,
        MANUALYAW_ID,
        MANUALROLL_ID,
        ZDIRX_ID,
        ZDIRY_ID,
        ZDIRZ_ID,
        AUTOZ_ID,
        AUTOSHIFT_ID,
        PRESHIFT_ID,
        GUESSSHIFTTHRESHOLD_ID,
        RECTIFY_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief matchingMethod 
     * matchingMethod 
     */
    int mMatchingMethod;

    /** 
     * \brief hessianThreshold 
     * hessianThreshold 
     */
    double mHessianThreshold;

    /** 
     * \brief octaves 
     * octaves 
     */
    int mOctaves;

    /** 
     * \brief octaveLayers 
     * octaveLayers 
     */
    int mOctaveLayers;

    /** 
     * \brief extended 
     * extended 
     */
    bool mExtended;

    /** 
     * \brief filterMinimumLength 
     * filterMinimumLength 
     */
    double mFilterMinimumLength;

    /** 
     * \brief useKLT 
     * useKLT 
     */
    bool mUseKLT;

    /** 
     * \brief computeEssential 
     * computeEssential 
     */
    bool mComputeEssential;

    /** 
     * \brief priorFocal 
     * priorFocal 
     */
    double mPriorFocal;

    /** 
     * \brief priorFocal2 
     * priorFocal2 
     */
    double mPriorFocal2;

    /** 
     * \brief baselineLength 
     * baselineLength 
     */
    double mBaselineLength;

    /** 
     * \brief fovAngle 
     * fovAngle 
     */
    double mFovAngle;

    /** 
     * \brief estimationMethod 
     * estimationMethod 
     */
    int mEstimationMethod;

    /** 
     * \brief normalise 
     * normalise 
     */
    bool mNormalise;

    /** 
     * \brief ransacIterations 
     * ransacIterations 
     */
    int mRansacIterations;

    /** 
     * \brief ransacTestSize 
     * ransacTestSize 
     */
    int mRansacTestSize;

    /** 
     * \brief ransacThreshold 
     * ransacThreshold 
     */
    double mRansacThreshold;

    /** 
     * \brief baselineX 
     * baselineX 
     */
    double mBaselineX;

    /** 
     * \brief baselineY 
     * baselineY 
     */
    double mBaselineY;

    /** 
     * \brief baselineZ 
     * baselineZ 
     */
    double mBaselineZ;

    /** 
     * \brief iterativeMethod 
     * iterativeMethod 
     */
    int mIterativeMethod;

    /** 
     * \brief iterativeIterations 
     * iterativeIterations 
     */
    int mIterativeIterations;

    /** 
     * \brief iterativeInitialSigma 
     * iterativeInitialSigma 
     */
    double mIterativeInitialSigma;

    /** 
     * \brief iterativeFactorSigma 
     * iterativeFactorSigma 
     */
    double mIterativeFactorSigma;

    /** 
     * \brief manualX 
     * manualX 
     */
    double mManualX;

    /** 
     * \brief manualY 
     * manualY 
     */
    double mManualY;

    /** 
     * \brief manualZ 
     * manualZ 
     */
    double mManualZ;

    /** 
     * \brief manualPitch 
     * manualPitch 
     */
    double mManualPitch;

    /** 
     * \brief manualYaw 
     * manualYaw 
     */
    double mManualYaw;

    /** 
     * \brief manualRoll 
     * manualRoll 
     */
    double mManualRoll;

    /** 
     * \brief zdirX 
     * zdirX 
     */
    double mZdirX;

    /** 
     * \brief zdirY 
     * zdirY 
     */
    double mZdirY;

    /** 
     * \brief zdirZ 
     * zdirZ 
     */
    double mZdirZ;

    /** 
     * \brief autoZ 
     * autoZ 
     */
    bool mAutoZ;

    /** 
     * \brief autoShift 
     * autoShift 
     */
    bool mAutoShift;

    /** 
     * \brief preShift 
     * preShift 
     */
    int mPreShift;

    /** 
     * \brief guessShiftThreshold 
     * guessShiftThreshold 
     */
    int mGuessShiftThreshold;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    MatchingMethodType::MatchingMethodType matchingMethod() const
    {
        return static_cast<MatchingMethodType::MatchingMethodType>(mMatchingMethod);
    }

    double hessianThreshold() const
    {
        return mHessianThreshold;
    }

    int octaves() const
    {
        return mOctaves;
    }

    int octaveLayers() const
    {
        return mOctaveLayers;
    }

    bool extended() const
    {
        return mExtended;
    }

    double filterMinimumLength() const
    {
        return mFilterMinimumLength;
    }

    bool useKLT() const
    {
        return mUseKLT;
    }

    bool computeEssential() const
    {
        return mComputeEssential;
    }

    double priorFocal() const
    {
        return mPriorFocal;
    }

    double priorFocal2() const
    {
        return mPriorFocal2;
    }

    double baselineLength() const
    {
        return mBaselineLength;
    }

    double fovAngle() const
    {
        return mFovAngle;
    }

    EstimationMethodType::EstimationMethodType estimationMethod() const
    {
        return static_cast<EstimationMethodType::EstimationMethodType>(mEstimationMethod);
    }

    bool normalise() const
    {
        return mNormalise;
    }

    int ransacIterations() const
    {
        return mRansacIterations;
    }

    int ransacTestSize() const
    {
        return mRansacTestSize;
    }

    double ransacThreshold() const
    {
        return mRansacThreshold;
    }

    double baselineX() const
    {
        return mBaselineX;
    }

    double baselineY() const
    {
        return mBaselineY;
    }

    double baselineZ() const
    {
        return mBaselineZ;
    }

    OptimizationMethodType::OptimizationMethodType iterativeMethod() const
    {
        return static_cast<OptimizationMethodType::OptimizationMethodType>(mIterativeMethod);
    }

    int iterativeIterations() const
    {
        return mIterativeIterations;
    }

    double iterativeInitialSigma() const
    {
        return mIterativeInitialSigma;
    }

    double iterativeFactorSigma() const
    {
        return mIterativeFactorSigma;
    }

    double manualX() const
    {
        return mManualX;
    }

    double manualY() const
    {
        return mManualY;
    }

    double manualZ() const
    {
        return mManualZ;
    }

    double manualPitch() const
    {
        return mManualPitch;
    }

    double manualYaw() const
    {
        return mManualYaw;
    }

    double manualRoll() const
    {
        return mManualRoll;
    }

    double zdirX() const
    {
        return mZdirX;
    }

    double zdirY() const
    {
        return mZdirY;
    }

    double zdirZ() const
    {
        return mZdirZ;
    }

    bool autoZ() const
    {
        return mAutoZ;
    }

    bool autoShift() const
    {
        return mAutoShift;
    }

    int preShift() const
    {
        return mPreShift;
    }

    int guessShiftThreshold() const
    {
        return mGuessShiftThreshold;
    }

    /* Section with setters */
    void setMatchingMethod(MatchingMethodType::MatchingMethodType matchingMethod)
    {
        mMatchingMethod = matchingMethod;
    }

    void setHessianThreshold(double hessianThreshold)
    {
        mHessianThreshold = hessianThreshold;
    }

    void setOctaves(int octaves)
    {
        mOctaves = octaves;
    }

    void setOctaveLayers(int octaveLayers)
    {
        mOctaveLayers = octaveLayers;
    }

    void setExtended(bool extended)
    {
        mExtended = extended;
    }

    void setFilterMinimumLength(double filterMinimumLength)
    {
        mFilterMinimumLength = filterMinimumLength;
    }

    void setUseKLT(bool useKLT)
    {
        mUseKLT = useKLT;
    }

    void setComputeEssential(bool computeEssential)
    {
        mComputeEssential = computeEssential;
    }

    void setPriorFocal(double priorFocal)
    {
        mPriorFocal = priorFocal;
    }

    void setPriorFocal2(double priorFocal2)
    {
        mPriorFocal2 = priorFocal2;
    }

    void setBaselineLength(double baselineLength)
    {
        mBaselineLength = baselineLength;
    }

    void setFovAngle(double fovAngle)
    {
        mFovAngle = fovAngle;
    }

    void setEstimationMethod(EstimationMethodType::EstimationMethodType estimationMethod)
    {
        mEstimationMethod = estimationMethod;
    }

    void setNormalise(bool normalise)
    {
        mNormalise = normalise;
    }

    void setRansacIterations(int ransacIterations)
    {
        mRansacIterations = ransacIterations;
    }

    void setRansacTestSize(int ransacTestSize)
    {
        mRansacTestSize = ransacTestSize;
    }

    void setRansacThreshold(double ransacThreshold)
    {
        mRansacThreshold = ransacThreshold;
    }

    void setBaselineX(double baselineX)
    {
        mBaselineX = baselineX;
    }

    void setBaselineY(double baselineY)
    {
        mBaselineY = baselineY;
    }

    void setBaselineZ(double baselineZ)
    {
        mBaselineZ = baselineZ;
    }

    void setIterativeMethod(OptimizationMethodType::OptimizationMethodType iterativeMethod)
    {
        mIterativeMethod = iterativeMethod;
    }

    void setIterativeIterations(int iterativeIterations)
    {
        mIterativeIterations = iterativeIterations;
    }

    void setIterativeInitialSigma(double iterativeInitialSigma)
    {
        mIterativeInitialSigma = iterativeInitialSigma;
    }

    void setIterativeFactorSigma(double iterativeFactorSigma)
    {
        mIterativeFactorSigma = iterativeFactorSigma;
    }

    void setManualX(double manualX)
    {
        mManualX = manualX;
    }

    void setManualY(double manualY)
    {
        mManualY = manualY;
    }

    void setManualZ(double manualZ)
    {
        mManualZ = manualZ;
    }

    void setManualPitch(double manualPitch)
    {
        mManualPitch = manualPitch;
    }

    void setManualYaw(double manualYaw)
    {
        mManualYaw = manualYaw;
    }

    void setManualRoll(double manualRoll)
    {
        mManualRoll = manualRoll;
    }

    void setZdirX(double zdirX)
    {
        mZdirX = zdirX;
    }

    void setZdirY(double zdirY)
    {
        mZdirY = zdirY;
    }

    void setZdirZ(double zdirZ)
    {
        mZdirZ = zdirZ;
    }

    void setAutoZ(bool autoZ)
    {
        mAutoZ = autoZ;
    }

    void setAutoShift(bool autoShift)
    {
        mAutoShift = autoShift;
    }

    void setPreShift(int preShift)
    {
        mPreShift = preShift;
    }

    void setGuessShiftThreshold(int guessShiftThreshold)
    {
        mGuessShiftThreshold = guessShiftThreshold;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit((int &)mMatchingMethod,     static_cast<const corecvs::EnumField *>(fields()[MATCHINGMETHOD_ID]));
        visitor.visit(mHessianThreshold,          static_cast<const corecvs::DoubleField *>(fields()[HESSIANTHRESHOLD_ID]));
        visitor.visit(mOctaves,                   static_cast<const corecvs::IntField *>(fields()[OCTAVES_ID]));
        visitor.visit(mOctaveLayers,              static_cast<const corecvs::IntField *>(fields()[OCTAVELAYERS_ID]));
        visitor.visit(mExtended,                  static_cast<const corecvs::BoolField *>(fields()[EXTENDED_ID]));
        visitor.visit(mFilterMinimumLength,       static_cast<const corecvs::DoubleField *>(fields()[FILTERMINIMUMLENGTH_ID]));
        visitor.visit(mUseKLT,                    static_cast<const corecvs::BoolField *>(fields()[USEKLT_ID]));
        visitor.visit(mComputeEssential,          static_cast<const corecvs::BoolField *>(fields()[COMPUTEESSENTIAL_ID]));
        visitor.visit(mPriorFocal,                static_cast<const corecvs::DoubleField *>(fields()[PRIORFOCAL_ID]));
        visitor.visit(mPriorFocal2,               static_cast<const corecvs::DoubleField *>(fields()[PRIORFOCAL2_ID]));
        visitor.visit(mBaselineLength,            static_cast<const corecvs::DoubleField *>(fields()[BASELINELENGTH_ID]));
        visitor.visit(mFovAngle,                  static_cast<const corecvs::DoubleField *>(fields()[FOVANGLE_ID]));
        visitor.visit((int &)mEstimationMethod,   static_cast<const corecvs::EnumField *>(fields()[ESTIMATIONMETHOD_ID]));
        visitor.visit(mNormalise,                 static_cast<const corecvs::BoolField *>(fields()[NORMALISE_ID]));
        visitor.visit(mRansacIterations,          static_cast<const corecvs::IntField *>(fields()[RANSACITERATIONS_ID]));
        visitor.visit(mRansacTestSize,            static_cast<const corecvs::IntField *>(fields()[RANSACTESTSIZE_ID]));
        visitor.visit(mRansacThreshold,           static_cast<const corecvs::DoubleField *>(fields()[RANSACTHRESHOLD_ID]));
        visitor.visit(mBaselineX,                 static_cast<const corecvs::DoubleField *>(fields()[BASELINEX_ID]));
        visitor.visit(mBaselineY,                 static_cast<const corecvs::DoubleField *>(fields()[BASELINEY_ID]));
        visitor.visit(mBaselineZ,                 static_cast<const corecvs::DoubleField *>(fields()[BASELINEZ_ID]));
        visitor.visit((int &)mIterativeMethod,    static_cast<const corecvs::EnumField *>(fields()[ITERATIVEMETHOD_ID]));
        visitor.visit(mIterativeIterations,       static_cast<const corecvs::IntField *>(fields()[ITERATIVEITERATIONS_ID]));
        visitor.visit(mIterativeInitialSigma,     static_cast<const corecvs::DoubleField *>(fields()[ITERATIVEINITIALSIGMA_ID]));
        visitor.visit(mIterativeFactorSigma,      static_cast<const corecvs::DoubleField *>(fields()[ITERATIVEFACTORSIGMA_ID]));
        visitor.visit(mManualX,                   static_cast<const corecvs::DoubleField *>(fields()[MANUALX_ID]));
        visitor.visit(mManualY,                   static_cast<const corecvs::DoubleField *>(fields()[MANUALY_ID]));
        visitor.visit(mManualZ,                   static_cast<const corecvs::DoubleField *>(fields()[MANUALZ_ID]));
        visitor.visit(mManualPitch,               static_cast<const corecvs::DoubleField *>(fields()[MANUALPITCH_ID]));
        visitor.visit(mManualYaw,                 static_cast<const corecvs::DoubleField *>(fields()[MANUALYAW_ID]));
        visitor.visit(mManualRoll,                static_cast<const corecvs::DoubleField *>(fields()[MANUALROLL_ID]));
        visitor.visit(mZdirX,                     static_cast<const corecvs::DoubleField *>(fields()[ZDIRX_ID]));
        visitor.visit(mZdirY,                     static_cast<const corecvs::DoubleField *>(fields()[ZDIRY_ID]));
        visitor.visit(mZdirZ,                     static_cast<const corecvs::DoubleField *>(fields()[ZDIRZ_ID]));
        visitor.visit(mAutoZ,                     static_cast<const corecvs::BoolField *>(fields()[AUTOZ_ID]));
        visitor.visit(mAutoShift,                 static_cast<const corecvs::BoolField *>(fields()[AUTOSHIFT_ID]));
        visitor.visit(mPreShift,                  static_cast<const corecvs::IntField *>(fields()[PRESHIFT_ID]));
        visitor.visit(mGuessShiftThreshold,       static_cast<const corecvs::IntField *>(fields()[GUESSSHIFTTHRESHOLD_ID]));
    }

    RectifyParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    RectifyParameters(
          MatchingMethodType::MatchingMethodType matchingMethod
        , double hessianThreshold
        , int octaves
        , int octaveLayers
        , bool extended
        , double filterMinimumLength
        , bool useKLT
        , bool computeEssential
        , double priorFocal
        , double priorFocal2
        , double baselineLength
        , double fovAngle
        , EstimationMethodType::EstimationMethodType estimationMethod
        , bool normalise
        , int ransacIterations
        , int ransacTestSize
        , double ransacThreshold
        , double baselineX
        , double baselineY
        , double baselineZ
        , OptimizationMethodType::OptimizationMethodType iterativeMethod
        , int iterativeIterations
        , double iterativeInitialSigma
        , double iterativeFactorSigma
        , double manualX
        , double manualY
        , double manualZ
        , double manualPitch
        , double manualYaw
        , double manualRoll
        , double zdirX
        , double zdirY
        , double zdirZ
        , bool autoZ
        , bool autoShift
        , int preShift
        , int guessShiftThreshold
    )
    {
        mMatchingMethod = matchingMethod;
        mHessianThreshold = hessianThreshold;
        mOctaves = octaves;
        mOctaveLayers = octaveLayers;
        mExtended = extended;
        mFilterMinimumLength = filterMinimumLength;
        mUseKLT = useKLT;
        mComputeEssential = computeEssential;
        mPriorFocal = priorFocal;
        mPriorFocal2 = priorFocal2;
        mBaselineLength = baselineLength;
        mFovAngle = fovAngle;
        mEstimationMethod = estimationMethod;
        mNormalise = normalise;
        mRansacIterations = ransacIterations;
        mRansacTestSize = ransacTestSize;
        mRansacThreshold = ransacThreshold;
        mBaselineX = baselineX;
        mBaselineY = baselineY;
        mBaselineZ = baselineZ;
        mIterativeMethod = iterativeMethod;
        mIterativeIterations = iterativeIterations;
        mIterativeInitialSigma = iterativeInitialSigma;
        mIterativeFactorSigma = iterativeFactorSigma;
        mManualX = manualX;
        mManualY = manualY;
        mManualZ = manualZ;
        mManualPitch = manualPitch;
        mManualYaw = manualYaw;
        mManualRoll = manualRoll;
        mZdirX = zdirX;
        mZdirY = zdirY;
        mZdirZ = zdirZ;
        mAutoZ = autoZ;
        mAutoShift = autoShift;
        mPreShift = preShift;
        mGuessShiftThreshold = guessShiftThreshold;
    }

    bool operator ==(const RectifyParameters &other) const 
    {
        if ( !(this->mMatchingMethod == other.mMatchingMethod)) return false;
        if ( !(this->mHessianThreshold == other.mHessianThreshold)) return false;
        if ( !(this->mOctaves == other.mOctaves)) return false;
        if ( !(this->mOctaveLayers == other.mOctaveLayers)) return false;
        if ( !(this->mExtended == other.mExtended)) return false;
        if ( !(this->mFilterMinimumLength == other.mFilterMinimumLength)) return false;
        if ( !(this->mUseKLT == other.mUseKLT)) return false;
        if ( !(this->mComputeEssential == other.mComputeEssential)) return false;
        if ( !(this->mPriorFocal == other.mPriorFocal)) return false;
        if ( !(this->mPriorFocal2 == other.mPriorFocal2)) return false;
        if ( !(this->mBaselineLength == other.mBaselineLength)) return false;
        if ( !(this->mFovAngle == other.mFovAngle)) return false;
        if ( !(this->mEstimationMethod == other.mEstimationMethod)) return false;
        if ( !(this->mNormalise == other.mNormalise)) return false;
        if ( !(this->mRansacIterations == other.mRansacIterations)) return false;
        if ( !(this->mRansacTestSize == other.mRansacTestSize)) return false;
        if ( !(this->mRansacThreshold == other.mRansacThreshold)) return false;
        if ( !(this->mBaselineX == other.mBaselineX)) return false;
        if ( !(this->mBaselineY == other.mBaselineY)) return false;
        if ( !(this->mBaselineZ == other.mBaselineZ)) return false;
        if ( !(this->mIterativeMethod == other.mIterativeMethod)) return false;
        if ( !(this->mIterativeIterations == other.mIterativeIterations)) return false;
        if ( !(this->mIterativeInitialSigma == other.mIterativeInitialSigma)) return false;
        if ( !(this->mIterativeFactorSigma == other.mIterativeFactorSigma)) return false;
        if ( !(this->mManualX == other.mManualX)) return false;
        if ( !(this->mManualY == other.mManualY)) return false;
        if ( !(this->mManualZ == other.mManualZ)) return false;
        if ( !(this->mManualPitch == other.mManualPitch)) return false;
        if ( !(this->mManualYaw == other.mManualYaw)) return false;
        if ( !(this->mManualRoll == other.mManualRoll)) return false;
        if ( !(this->mZdirX == other.mZdirX)) return false;
        if ( !(this->mZdirY == other.mZdirY)) return false;
        if ( !(this->mZdirZ == other.mZdirZ)) return false;
        if ( !(this->mAutoZ == other.mAutoZ)) return false;
        if ( !(this->mAutoShift == other.mAutoShift)) return false;
        if ( !(this->mPreShift == other.mPreShift)) return false;
        if ( !(this->mGuessShiftThreshold == other.mGuessShiftThreshold)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, RectifyParameters &toSave)
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
#endif  //RECTIFY_PARAMETERS_H_
