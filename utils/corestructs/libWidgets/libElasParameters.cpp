/**
 * \file libElasParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "libElasParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace core3vi {
template<>
Reflection BaseReflection<LibElasParameters>::reflection = Reflection();
template<>
int BaseReflection<LibElasParameters>::dummy = LibElasParameters::staticInit();
} // namespace core3vi 

SUPPRESS_OFFSET_WARNING_BEGIN

int LibElasParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "LibElas Parameters",
        "LibElas Parameters. Supported only in internal builds, due to LIBELAS GPL license",
        ""
    );
     

    fields().push_back(
        new IntField
        (
          LibElasParameters::DISP_MIN_ID,
          offsetof(LibElasParameters, mDispMin),
          0,
          "Disp Min",
          "Disp Min",
          "min disparity"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::DISP_MAX_ID,
          offsetof(LibElasParameters, mDispMax),
          255,
          "Disp Max",
          "Disp Max",
          "max disparity"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LibElasParameters::SUPPORT_THRESHOLD_ID,
          offsetof(LibElasParameters, mSupportThreshold),
          0.85,
          "Support Threshold",
          "Support Threshold",
          "maximum uniqueness ratio best vs second best support match"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::SUPPORT_TEXTURE_ID,
          offsetof(LibElasParameters, mSupportTexture),
          10,
          "Support Texture",
          "Support Texture",
          "min texture for support points"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::CANDIDATE_STEPSIZE_ID,
          offsetof(LibElasParameters, mCandidateStepsize),
          5,
          "Candidate Stepsize",
          "Candidate Stepsize",
          "step size of regular grid on which support points are matched"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::INCON_WINDOW_SIZE_ID,
          offsetof(LibElasParameters, mInconWindowSize),
          5,
          "Incon Window Size",
          "Incon Window Size",
          "window size of inconsistent support point check"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::INCON_THRESHOLD_ID,
          offsetof(LibElasParameters, mInconThreshold),
          5,
          "Incon Threshold",
          "Incon Threshold",
          "disparity similarity threshold for support point to be considered consistent"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::INCON_MIN_SUPPORT_ID,
          offsetof(LibElasParameters, mInconMinSupport),
          5,
          "Incon Min Support",
          "Incon Min Support",
          "minimum number of consistent support points"
        )
    );
    fields().push_back(
        new BoolField
        (
          LibElasParameters::ADD_CORNERS_ID,
          offsetof(LibElasParameters, mAddCorners),
          false,
          "Add Corners",
          "Add Corners",
          "add support points at image corners with nearest neighbor disparities"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::GRID_SIZE_ID,
          offsetof(LibElasParameters, mGridSize),
          20,
          "Grid Size",
          "Grid Size",
          "size of neighborhood for additional support point extrapolation"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LibElasParameters::BETA_ID,
          offsetof(LibElasParameters, mBeta),
          0.02,
          "beta",
          "beta",
          "image likelihood parameter"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LibElasParameters::GAMMA_ID,
          offsetof(LibElasParameters, mGamma),
          3,
          "gamma",
          "gamma",
          "prior constant"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LibElasParameters::SIGMA_ID,
          offsetof(LibElasParameters, mSigma),
          1,
          "sigma",
          "sigma",
          "prior sigma"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LibElasParameters::S_RADIUS_ID,
          offsetof(LibElasParameters, mSRadius),
          2,
          "S Radius",
          "S Radius",
          "prior sigma radius"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::MATCH_TEXTURE_ID,
          offsetof(LibElasParameters, mMatchTexture),
          1,
          "Match Texture",
          "Match Texture",
          "min texture for dense matching"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::L_R_THRESHOLD_ID,
          offsetof(LibElasParameters, mLRThreshold),
          2,
          "L R Threshold",
          "L R Threshold",
          "disparity threshold for left right consistency check"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LibElasParameters::SPECKLE_SIM_THRESHOLD_ID,
          offsetof(LibElasParameters, mSpeckleSimThreshold),
          1,
          "Speckle Sim Threshold",
          "Speckle Sim Threshold",
          "similarity threshold for speckle segmentation"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::SPECKLE_SIZE_ID,
          offsetof(LibElasParameters, mSpeckleSize),
          200,
          "Speckle Size",
          "Speckle Size",
          "maximal size of a speckle. Small speckles get removed"
        )
    );
    fields().push_back(
        new IntField
        (
          LibElasParameters::IPOL_GAP_WIDTH_ID,
          offsetof(LibElasParameters, mIpolGapWidth),
          3,
          "Ipol Gap Width",
          "Ipol Gap Width",
          "interpolate small gaps"
        )
    );
    fields().push_back(
        new BoolField
        (
          LibElasParameters::FILTER_MEDIAN_ID,
          offsetof(LibElasParameters, mFilterMedian),
          false,
          "Filter Median",
          "Filter Median",
          "optional median filter approximated"
        )
    );
    fields().push_back(
        new BoolField
        (
          LibElasParameters::FILTER_ADAPTIVE_MEAN_ID,
          offsetof(LibElasParameters, mFilterAdaptiveMean),
          true,
          "Filter Adaptive Mean",
          "Filter Adaptive Mean",
          "optional adaptive mean filter approximated"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


