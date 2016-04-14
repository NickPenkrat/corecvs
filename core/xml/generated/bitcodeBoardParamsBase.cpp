/**
 * \file bitcodeBoardParamsBase.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "bitcodeBoardParamsBase.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<BitcodeBoardParamsBase>::reflection = Reflection();
template<>
int BaseReflection<BitcodeBoardParamsBase>::dummy = BitcodeBoardParamsBase::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int BitcodeBoardParamsBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Bitcode Board Params Base",
        "Bitcode Board Params Base",
        ""
    );
     

    fields().push_back(
        new BoolField
        (
          BitcodeBoardParamsBase::VERTICAL_ID,
          offsetof(BitcodeBoardParamsBase, mVertical),
          true,
          "vertical",
          "vertical",
          "Orientation"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::CELLSIZE_ID,
          offsetof(BitcodeBoardParamsBase, mCellSize),
          256,
          "cellSize",
          "cellSize",
          "Chess cells size in pixels"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::BLACKCOLOR_ID,
          offsetof(BitcodeBoardParamsBase, mBlackColor),
          0,
          "blackColor",
          "blackColor",
          "shades of gray for black chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::WHITECOLOR_ID,
          offsetof(BitcodeBoardParamsBase, mWhiteColor),
          255,
          "whiteColor",
          "whiteColor",
          "shades of gray for white chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::IDENTSIZE_ID,
          offsetof(BitcodeBoardParamsBase, mIdentSize),
          1,
          "identSize",
          "identSize",
          "white ident size around chessboard in chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::BOARDHEIGHT_ID,
          offsetof(BitcodeBoardParamsBase, mBoardHeight),
          4,
          "boardHeight",
          "boardHeight",
          "size of the chessboard in chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::BOARDWIDTH_ID,
          offsetof(BitcodeBoardParamsBase, mBoardWidth),
          4,
          "boardWidth",
          "boardWidth",
          "size of the chessboard in chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::CODEWIDTH_ID,
          offsetof(BitcodeBoardParamsBase, mCodeWidth),
          4,
          "codeWidth",
          "codeWidth",
          "width and height of the bitcode in chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::CODEHEIGHT_ID,
          offsetof(BitcodeBoardParamsBase, mCodeHeight),
          2,
          "codeHeight",
          "codeHeight",
          "width and height of the bitcode in chesses"
        )
    );
    fields().push_back(
        new IntField
        (
          BitcodeBoardParamsBase::BITCODEIDENTSIZE_ID,
          offsetof(BitcodeBoardParamsBase, mBitcodeIdentSize),
          1,
          "bitcodeIdentSize",
          "bitcodeIdentSize",
          "ident between chessboard and bitcode in chesses"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


