/**
 * \file chessBoardAssemblerParamsBase.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "chessBoardAssemblerParamsBase.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<ChessBoardAssemblerParamsBase>::reflection = Reflection();
template<>
int BaseReflection<ChessBoardAssemblerParamsBase>::dummy = ChessBoardAssemblerParamsBase::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int ChessBoardAssemblerParamsBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Chess Board Assembler Params Base",
        "Chess Board Assembler Params Base",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          ChessBoardAssemblerParamsBase::SEEDTHRESHOLD_ID,
          offsetof(ChessBoardAssemblerParamsBase, mSeedThreshold),
          0.3,
          "seedThreshold",
          "seedThreshold",
          "Threshold for seed grid non-regularity"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardAssemblerParamsBase::SEEDTGPENALTY_ID,
          offsetof(ChessBoardAssemblerParamsBase, mSeedTgPenalty),
          5,
          "seedTgPenalty",
          "seedTgPenalty",
          "Factor for orthogonal error in seed estimation"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardAssemblerParamsBase::CONSERVATIVITY_ID,
          offsetof(ChessBoardAssemblerParamsBase, mConservativity),
          0.9,
          "conservativity",
          "conservativity",
          "Factor of conservativity in next row prediction (should be lower to high-distorted boards)"
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardAssemblerParamsBase::COSTTHRESHOLD_ID,
          offsetof(ChessBoardAssemblerParamsBase, mCostThreshold),
          -10,
          "costThreshold",
          "costThreshold",
          " Maximal cost for real board",
          true,
         -999999,
         999999
        )
    );
    fields().push_back(
        new DoubleField
        (
          ChessBoardAssemblerParamsBase::MINSEEDDISTANCE_ID,
          offsetof(ChessBoardAssemblerParamsBase, mMinSeedDistance),
          15,
          "minSeedDistance",
          "minSeedDistance",
          "Minimal seed distance"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardAssemblerParamsBase::HYPOTHESISDIMENSIONS_ID,
          offsetof(ChessBoardAssemblerParamsBase, mHypothesisDimensions),
          1,
          "hypothesisDimensions",
          "hypothesisDimensions",
          "Hypothesis type: consider only hypothesis that fits specified number of dims"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardAssemblerParamsBase::HYPOTHESISDIMFIRST_ID,
          offsetof(ChessBoardAssemblerParamsBase, mHypothesisDimFirst),
          18,
          "hypothesisDimFirst",
          "hypothesisDimFirst",
          "hypothesisDimFirst"
        )
    );
    fields().push_back(
        new IntField
        (
          ChessBoardAssemblerParamsBase::HYPOTHESISDIMSECOND_ID,
          offsetof(ChessBoardAssemblerParamsBase, mHypothesisDimSecond),
          11,
          "hypothesisDimSecond",
          "hypothesisDimSecond",
          "hypothesisDimSecond"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


