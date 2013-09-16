#ifndef DRAW_3D_PARAMETERS_H_
#define DRAW_3D_PARAMETERS_H_
/**
 * \file draw3dParameters.h
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
#include "rgbColorParameters.h"

using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

namespace corecvs {
}
/*
 *  Additional includes for enum section.
 */
#include "draw3dStyle.h"
#include "draw3dTextureGen.h"

/**
 * \brief draw 3d Parameters 
 * draw 3d Parameters 
 **/
class Draw3dParameters : public BaseReflection<Draw3dParameters>
{
public:
    enum FieldId {
        STYLE_ID,
        COLOR_ID,
        SECONDARY_COLOR_ID,
        TEXTURE_CORRODINATES_ID,
        TEXTURE_ALPHA_ID,
        TEXTURE_SCALE_ID,
        DECAL_MATRIX_TYPE_ID,
        DECAL_LEFT_CAM_ID,
        DECAL_LEFT_ALPHA_ID,
        DECAL_RIGHT_CAM_ID,
        DECAL_RIGHT_ALPHA_ID,
        DRAW_3D_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief style 
     * style 
     */
    int mStyle;

    /** 
     * \brief Color 
     * Color 
     */
    RgbColorParameters mColor;

    /** 
     * \brief Secondary Color 
     * Secondary Color 
     */
    RgbColorParameters mSecondaryColor;

    /** 
     * \brief Texture Corrodinates 
     * Texture Corrodinates 
     */
    int mTextureCorrodinates;

    /** 
     * \brief Texture Alpha 
     * Texture Alpha 
     */
    int mTextureAlpha;

    /** 
     * \brief Texture Scale 
     * Texture Scale 
     */
    double mTextureScale;

    /** 
     * \brief Decal Matrix Type 
     * Decal Matrix Type 
     */
    int mDecalMatrixType;

    /** 
     * \brief Decal Left Cam 
     * Decal Left Cam 
     */
    bool mDecalLeftCam;

    /** 
     * \brief Decal Left Alpha 
     * Decal Left Alpha 
     */
    int mDecalLeftAlpha;

    /** 
     * \brief Decal Right Cam 
     * Decal Right Cam 
     */
    bool mDecalRightCam;

    /** 
     * \brief Decal Right Alpha 
     * Decal Right Alpha 
     */
    int mDecalRightAlpha;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    Draw3dStyle::Draw3dStyle style() const
    {
        return static_cast<Draw3dStyle::Draw3dStyle>(mStyle);
    }

    RgbColorParameters color() const
    {
        return mColor;
    }

    RgbColorParameters secondaryColor() const
    {
        return mSecondaryColor;
    }

    Draw3dTextureGen::Draw3dTextureGen textureCorrodinates() const
    {
        return static_cast<Draw3dTextureGen::Draw3dTextureGen>(mTextureCorrodinates);
    }

    int textureAlpha() const
    {
        return mTextureAlpha;
    }

    double textureScale() const
    {
        return mTextureScale;
    }

    int decalMatrixType() const
    {
        return mDecalMatrixType;
    }

    bool decalLeftCam() const
    {
        return mDecalLeftCam;
    }

    int decalLeftAlpha() const
    {
        return mDecalLeftAlpha;
    }

    bool decalRightCam() const
    {
        return mDecalRightCam;
    }

    int decalRightAlpha() const
    {
        return mDecalRightAlpha;
    }

    /* Section with setters */
    void setStyle(Draw3dStyle::Draw3dStyle style)
    {
        mStyle = style;
    }

    void setColor(RgbColorParameters const &color)
    {
        mColor = color;
    }

    void setSecondaryColor(RgbColorParameters const &secondaryColor)
    {
        mSecondaryColor = secondaryColor;
    }

    void setTextureCorrodinates(Draw3dTextureGen::Draw3dTextureGen textureCorrodinates)
    {
        mTextureCorrodinates = textureCorrodinates;
    }

    void setTextureAlpha(int textureAlpha)
    {
        mTextureAlpha = textureAlpha;
    }

    void setTextureScale(double textureScale)
    {
        mTextureScale = textureScale;
    }

    void setDecalMatrixType(int decalMatrixType)
    {
        mDecalMatrixType = decalMatrixType;
    }

    void setDecalLeftCam(bool decalLeftCam)
    {
        mDecalLeftCam = decalLeftCam;
    }

    void setDecalLeftAlpha(int decalLeftAlpha)
    {
        mDecalLeftAlpha = decalLeftAlpha;
    }

    void setDecalRightCam(bool decalRightCam)
    {
        mDecalRightCam = decalRightCam;
    }

    void setDecalRightAlpha(int decalRightAlpha)
    {
        mDecalRightAlpha = decalRightAlpha;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit((int &)mStyle,              static_cast<const EnumField *>    (fields()[STYLE_ID]));
        visitor.visit(mColor,                     static_cast<const CompositeField *>(fields()[COLOR_ID]));
        visitor.visit(mSecondaryColor,            static_cast<const CompositeField *>(fields()[SECONDARY_COLOR_ID]));
        visitor.visit((int &)mTextureCorrodinates, static_cast<const EnumField *>    (fields()[TEXTURE_CORRODINATES_ID]));
        visitor.visit(mTextureAlpha,              static_cast<const IntField *>     (fields()[TEXTURE_ALPHA_ID]));
        visitor.visit(mTextureScale,              static_cast<const DoubleField *>  (fields()[TEXTURE_SCALE_ID]));
        visitor.visit(mDecalMatrixType,           static_cast<const IntField *>     (fields()[DECAL_MATRIX_TYPE_ID]));
        visitor.visit(mDecalLeftCam,              static_cast<const BoolField *>    (fields()[DECAL_LEFT_CAM_ID]));
        visitor.visit(mDecalLeftAlpha,            static_cast<const IntField *>     (fields()[DECAL_LEFT_ALPHA_ID]));
        visitor.visit(mDecalRightCam,             static_cast<const BoolField *>    (fields()[DECAL_RIGHT_CAM_ID]));
        visitor.visit(mDecalRightAlpha,           static_cast<const IntField *>     (fields()[DECAL_RIGHT_ALPHA_ID]));
    }

    Draw3dParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    Draw3dParameters(
          Draw3dStyle::Draw3dStyle style
        , RgbColorParameters color
        , RgbColorParameters secondaryColor
        , Draw3dTextureGen::Draw3dTextureGen textureCorrodinates
        , int textureAlpha
        , double textureScale
        , int decalMatrixType
        , bool decalLeftCam
        , int decalLeftAlpha
        , bool decalRightCam
        , int decalRightAlpha
    )
    {
        mStyle = style;
        mColor = color;
        mSecondaryColor = secondaryColor;
        mTextureCorrodinates = textureCorrodinates;
        mTextureAlpha = textureAlpha;
        mTextureScale = textureScale;
        mDecalMatrixType = decalMatrixType;
        mDecalLeftCam = decalLeftCam;
        mDecalLeftAlpha = decalLeftAlpha;
        mDecalRightCam = decalRightCam;
        mDecalRightAlpha = decalRightAlpha;
    }

    friend ostream& operator << (ostream &out, Draw3dParameters &toSave)
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
#endif  //DRAW_3D_PARAMETERS_H_
