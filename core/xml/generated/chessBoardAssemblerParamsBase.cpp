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


using namespace corecvs;

int ChessBoardAssemblerParamsBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Chess Board Assembler Params Base",
        "Chess Board Assembler Params Base",
        ""
    );

     getReflection()->objectSize = sizeof(ChessBoardAssemblerParamsBase);
     

    DoubleField* field0 = new DoubleField
        (
          ChessBoardAssemblerParamsBase::SEEDTHRESHOLD_ID,
          offsetof(ChessBoardAssemblerParamsBase, mSeedThreshold),
          0.3,
          "seedThreshold",
          "seedThreshold",
          "Threshold for seed grid non-regularity"
        );
    field0->widgetHint=BaseField::SPIN_BOX;
    field0->precision=2;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          ChessBoardAssemblerParamsBase::SEEDTGPENALTY_ID,
          offsetof(ChessBoardAssemblerParamsBase, mSeedTgPenalty),
          5,
          "seedTgPenalty",
          "seedTgPenalty",
          "Factor for orthogonal error in seed estimation"
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    DoubleField* field2 = new DoubleField
        (
          ChessBoardAssemblerParamsBase::CONSERVATIVITY_ID,
          offsetof(ChessBoardAssemblerParamsBase, mConservativity),
          0.9,
          "conservativity",
          "conservativity",
          "Factor of conservativity in next row prediction (should be lower to high-distorted boards)"
        );
    field2->widgetHint=BaseField::SPIN_BOX;
    field2->precision=2;
    fields().push_back(field2);
    /*  */ 
    DoubleField* field3 = new DoubleField
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
        );
    field3->widgetHint=BaseField::SPIN_BOX;
    field3->precision=2;
    fields().push_back(field3);
    /*  */ 
    DoubleField* field4 = new DoubleField
        (
          ChessBoardAssemblerParamsBase::MINSEEDDISTANCE_ID,
          offsetof(ChessBoardAssemblerParamsBase, mMinSeedDistance),
          15,
          "minSeedDistance",
          "minSeedDistance",
          "Minimal seed distance"
        );
    field4->widgetHint=BaseField::SPIN_BOX;
    field4->precision=2;
    fields().push_back(field4);
    /*  */ 
    IntField* field5 = new IntField
        (
          ChessBoardAssemblerParamsBase::HYPOTHESISDIMENSIONS_ID,
          offsetof(ChessBoardAssemblerParamsBase, mHypothesisDimensions),
          1,
          "hypothesisDimensions",
          "hypothesisDimensions",
          "Hypothesis type: consider only hypothesis that fits specified number of dims"
        );
    fields().push_back(field5);
    /*  */ 
    BoolField* field6 = new BoolField
        (
          ChessBoardAssemblerParamsBase::KDTREE_ID,
          offsetof(ChessBoardAssemblerParamsBase, mKdtree),
          false,
          "kdtree",
          "kdtree",
          "Use k-d tree for greedy expansion"
        );
    field6->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field6);
    /*  */ 
    IntField* field7 = new IntField
        (
          ChessBoardAssemblerParamsBase::HYPOTHESISDIMFIRST_ID,
          offsetof(ChessBoardAssemblerParamsBase, mHypothesisDimFirst),
          18,
          "hypothesisDimFirst",
          "hypothesisDimFirst",
          "hypothesisDimFirst"
        );
    fields().push_back(field7);
    /*  */ 
    IntField* field8 = new IntField
        (
          ChessBoardAssemblerParamsBase::HYPOTHESISDIMSECOND_ID,
          offsetof(ChessBoardAssemblerParamsBase, mHypothesisDimSecond),
          11,
          "hypothesisDimSecond",
          "hypothesisDimSecond",
          "hypothesisDimSecond"
        );
    fields().push_back(field8);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Chess Board Assembler Params Base")]= &reflection;
   return 0;
}
int ChessBoardAssemblerParamsBase::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


