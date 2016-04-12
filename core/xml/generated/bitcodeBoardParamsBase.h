#ifndef BITCODE_BOARD_PARAMS_BASE_H_
#define BITCODE_BOARD_PARAMS_BASE_H_
/**
 * \file bitcodeBoardParamsBase.h
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

using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

namespace corecvs {
}
/*
 *  Additional includes for enum section.
 */

/**
 * \brief Bitcode Board Params Base 
 * Bitcode Board Params Base 
 **/
class BitcodeBoardParamsBase : public BaseReflection<BitcodeBoardParamsBase>
{
public:
    enum FieldId {
        VERTICAL_ID,
        CELLSIZE_ID,
        BLACKCOLOR_ID,
        WHITECOLOR_ID,
        IDENTSIZE_ID,
        BOARDHEIGHT_ID,
        BOARDWIDTH_ID,
        CODEWIDTH_ID,
        CODEHEIGHT_ID,
        BITCODEIDENTSIZE_ID,
        BITCODECONFIDENCE_ID,
        BITCODE_BOARD_PARAMS_BASE_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief vertical 
     * Orientation 
     */
    bool mVertical;

    /** 
     * \brief cellSize 
     * Chess cells size in pixels 
     */
    int mCellSize;

    /** 
     * \brief blackColor 
     * shades of gray for black chesses 
     */
    int mBlackColor;

    /** 
     * \brief whiteColor 
     * shades of gray for white chesses 
     */
    int mWhiteColor;

    /** 
     * \brief identSize 
     * white ident size around chessboard in chesses 
     */
    double mIdentSize;

    /** 
     * \brief boardHeight 
     * size of the chessboard in chesses 
     */
    int mBoardHeight;

    /** 
     * \brief boardWidth 
     * size of the chessboard in chesses 
     */
    int mBoardWidth;

    /** 
     * \brief codeWidth 
     * width and height of the bitcode in chesses 
     */
    int mCodeWidth;

    /** 
     * \brief codeHeight 
     * width and height of the bitcode in chesses 
     */
    int mCodeHeight;

    /** 
     * \brief bitcodeIdentSize 
     * ident between chessboard and bitcode in chesses 
     */
    double mBitcodeIdentSize;

    /** 
     * \brief bitcodeConfidence 
     * Area in which the stats are collected during detection 
     */
    double mBitcodeConfidence;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool vertical() const
    {
        return mVertical;
    }

    int cellSize() const
    {
        return mCellSize;
    }

    int blackColor() const
    {
        return mBlackColor;
    }

    int whiteColor() const
    {
        return mWhiteColor;
    }

    double identSize() const
    {
        return mIdentSize;
    }

    int boardHeight() const
    {
        return mBoardHeight;
    }

    int boardWidth() const
    {
        return mBoardWidth;
    }

    int codeWidth() const
    {
        return mCodeWidth;
    }

    int codeHeight() const
    {
        return mCodeHeight;
    }

    double bitcodeIdentSize() const
    {
        return mBitcodeIdentSize;
    }

    double bitcodeConfidence() const
    {
        return mBitcodeConfidence;
    }

    /* Section with setters */
    void setVertical(bool vertical)
    {
        mVertical = vertical;
    }

    void setCellSize(int cellSize)
    {
        mCellSize = cellSize;
    }

    void setBlackColor(int blackColor)
    {
        mBlackColor = blackColor;
    }

    void setWhiteColor(int whiteColor)
    {
        mWhiteColor = whiteColor;
    }

    void setIdentSize(double identSize)
    {
        mIdentSize = identSize;
    }

    void setBoardHeight(int boardHeight)
    {
        mBoardHeight = boardHeight;
    }

    void setBoardWidth(int boardWidth)
    {
        mBoardWidth = boardWidth;
    }

    void setCodeWidth(int codeWidth)
    {
        mCodeWidth = codeWidth;
    }

    void setCodeHeight(int codeHeight)
    {
        mCodeHeight = codeHeight;
    }

    void setBitcodeIdentSize(double bitcodeIdentSize)
    {
        mBitcodeIdentSize = bitcodeIdentSize;
    }

    void setBitcodeConfidence(double bitcodeConfidence)
    {
        mBitcodeConfidence = bitcodeConfidence;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mVertical,                  static_cast<const BoolField *>    (fields()[VERTICAL_ID]));
        visitor.visit(mCellSize,                  static_cast<const IntField *>     (fields()[CELLSIZE_ID]));
        visitor.visit(mBlackColor,                static_cast<const IntField *>     (fields()[BLACKCOLOR_ID]));
        visitor.visit(mWhiteColor,                static_cast<const IntField *>     (fields()[WHITECOLOR_ID]));
        visitor.visit(mIdentSize,                 static_cast<const DoubleField *>  (fields()[IDENTSIZE_ID]));
        visitor.visit(mBoardHeight,               static_cast<const IntField *>     (fields()[BOARDHEIGHT_ID]));
        visitor.visit(mBoardWidth,                static_cast<const IntField *>     (fields()[BOARDWIDTH_ID]));
        visitor.visit(mCodeWidth,                 static_cast<const IntField *>     (fields()[CODEWIDTH_ID]));
        visitor.visit(mCodeHeight,                static_cast<const IntField *>     (fields()[CODEHEIGHT_ID]));
        visitor.visit(mBitcodeIdentSize,          static_cast<const DoubleField *>  (fields()[BITCODEIDENTSIZE_ID]));
        visitor.visit(mBitcodeConfidence,         static_cast<const DoubleField *>  (fields()[BITCODECONFIDENCE_ID]));
    }

    BitcodeBoardParamsBase()
    {
        DefaultSetter setter;
        accept(setter);
    }

    BitcodeBoardParamsBase(
          bool vertical
        , int cellSize
        , int blackColor
        , int whiteColor
        , double identSize
        , int boardHeight
        , int boardWidth
        , int codeWidth
        , int codeHeight
        , double bitcodeIdentSize
        , double bitcodeConfidence
    )
    {
        mVertical = vertical;
        mCellSize = cellSize;
        mBlackColor = blackColor;
        mWhiteColor = whiteColor;
        mIdentSize = identSize;
        mBoardHeight = boardHeight;
        mBoardWidth = boardWidth;
        mCodeWidth = codeWidth;
        mCodeHeight = codeHeight;
        mBitcodeIdentSize = bitcodeIdentSize;
        mBitcodeConfidence = bitcodeConfidence;
    }

    friend ostream& operator << (ostream &out, BitcodeBoardParamsBase &toSave)
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
#endif  //BITCODE_BOARD_PARAMS_BASE_H_
