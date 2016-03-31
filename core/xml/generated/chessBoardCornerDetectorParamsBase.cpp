/**
 * \file chessBoardCornerDetectorParamsBase.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "chessBoardCornerDetectorParamsBase.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<ChessBoardCornerDetectorParamsBase>::reflection = Reflection();
template<>
int BaseReflection<ChessBoardCornerDetectorParamsBase>::dummy = ChessBoardCornerDetectorParamsBase::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int ChessBoardCornerDetectorParamsBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Chess Board Corner Detector Params Base",
        "Chess Board Corner Detector Params Base",
        ""
    );
     

    fields().push_back(
        new BoolField
        (
          ChessBoardCornerDetectorParamsBase::PRODUCEDEBUG_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mProduceDebug),
          false,
          "produceDebug",
          "produceDebug",
          "produceDebug"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::GRADIENTCROSSWIDTH_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mGradientCrossWidth),
          3,
          "gradientCrossWidth",
          "gradientCrossWidth",
          "Width of cross for corner gradient-score"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::SECTORSIZEDEG_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mSectorSizeDeg),
          90,
          "sectorSizeDeg",
          "sectorSizeDeg",
          "Sector size in deg"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardCornerDetectorParamsBase::HISTOGRAMBINS_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mHistogramBins),
          50,
          "histogramBins",
          "histogramBins",
          "Number of bins for computing edge direction histogram",
          true,
         0,
         999999
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::MINANGLEDEG_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mMinAngleDeg),
          30,
          "minAngleDeg",
          "minAngleDeg",
          "Minimal angle between edges in deg"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardCornerDetectorParamsBase::NEIGHBORHOOD_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mNeighborhood),
          25,
          "neighborhood",
          "neighborhood",
          "Typical radius for estimating edge-related data and refinig corner positions",
          true,
         0,
         999999
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::GRADTHRESHOLD_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mGradThreshold),
          0.1,
          "gradThreshold",
          "gradThreshold",
          "Gradient magnitude threshold"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::ORIENTATIONINLIERTHRESHOLD_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mOrientationInlierThreshold),
          0.25,
          "orientationInlierThreshold",
          "orientationInlierThreshold",
          "Gradient orientation inlier threshold"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::INLIERDISTANCETHRESHOLD_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mInlierDistanceThreshold),
          5,
          "inlierDistanceThreshold",
          "inlierDistanceThreshold",
          "Threshold for distance to edge"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::UPDATETHRESHOLD_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mUpdateThreshold),
          4,
          "updateThreshold",
          "updateThreshold",
          "Threshold for maximal corner-position update"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::SCORETHRESHOLD_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mScoreThreshold),
          0,
          "scoreThreshold",
          "scoreThreshold",
          "Threshold for final score"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardCornerDetectorParamsBase::NROUNDS_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mNRounds),
          3,
          "nRounds",
          "nRounds",
          "Number of orientation/position refinement rounds",
          true,
         0,
         999999
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardCornerDetectorParamsBase::MEANSHIFTBANDWIDTH_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mMeanshiftBandwidth),
          1,
          "meanshiftBandwidth",
          "meanshiftBandwidth",
          "Meanshift smoothing stdev"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardCornerDetectorParamsBase::NMSLOCALITY_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mNmsLocality),
          20,
          "nmsLocality",
          "nmsLocality",
          "Non Minimal Supresstion locality threshold",
          true,
         0,
         999999
        )
    );
    double mPatternRadius_dv[] = {4,8,12};
    fields().push_back(
        new DoubleVectorField
        (
          ChessBoardCornerDetectorParamsBase::PATTERN_RADIUS_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mPatternRadius),
          vector<double>(mPatternRadius_dv, mPatternRadius_dv + 3),
          3,
          "Pattern Radius",
          "Pattern Radius",
          "Pattern Radius"
        )
    );
    double mPatternStartAngleDeg_dv[] = {0,45};
    fields().push_back(
        new DoubleVectorField
        (
          ChessBoardCornerDetectorParamsBase::PATTERNSTARTANGLEDEG_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mPatternStartAngleDeg),
          vector<double>(mPatternStartAngleDeg_dv, mPatternStartAngleDeg_dv + 2),
          2,
          "patternStartAngleDeg",
          "patternStartAngleDeg",
          "patternStartAngleDeg"
        )
    );
    double mCornerScores_dv[] = {4,8,12};
    fields().push_back(
        new DoubleVectorField
        (
          ChessBoardCornerDetectorParamsBase::CORNERSCORES_ID,
          offsetof(ChessBoardCornerDetectorParamsBase, mCornerScores),
          vector<double>(mCornerScores_dv, mCornerScores_dv + 3),
          3,
          "cornerScores",
          "cornerScores",
          "cornerScores"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


