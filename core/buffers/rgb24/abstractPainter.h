#pragma once
/**
 * \file abstractPainter.h
 *
 * \date Nov 22, 2012
 **/

#include <stdint.h>

#include "global.h"
#include "hardcodeFont.h"
#include "hersheyVectorFont.h"

namespace core3vi {

template<class TargetBuffer>
class AbstractPainter
{
	TargetBuffer *mTarget;
public:
	typedef typename TargetBuffer::InternalElementType ElementType;

	AbstractPainter(TargetBuffer *target) :
		mTarget(target)
	{}

    /**
     *  This function draws the char with the hardcoded font
     *  Not all chars are supported
     *
     *  Note: this method must be implemented right here, otherwise msvc couldn't understand correctly usage of "ElementType".
     **/
    void drawChar  (uint16_t x, uint16_t y, uint8_t theChar, ElementType color, int scale = 2)
    {
        uint8_t *char_ptr = NULL;
        if (theChar == ' ')
            return;
        if (theChar >='0' && theChar <= '9')
            char_ptr = HardcodeFont::num_glyphs + HardcodeFont::GLYPH_HEIGHT * (theChar - '0');
        if (theChar >='a' && theChar <= 'z')
            char_ptr = HardcodeFont::alpha_glyphs + HardcodeFont::GLYPH_HEIGHT * (theChar - 'a');
        if (theChar >='A' && theChar <= 'Z')
            char_ptr = HardcodeFont::alpha_glyphs + HardcodeFont::GLYPH_HEIGHT * (theChar - 'A');

        if (theChar == '-')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 0;
        if (theChar == '%')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 1;
        if (theChar == '.')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 2;
        if (theChar == ':')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 3;
        if (theChar == '!')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 4;
        if (theChar == '?')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 5;
        if (theChar == '$')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 6;
        if (theChar == '\"')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 7;
        if (theChar == '\'')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 8;
        if (theChar == '_')
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 9;

        if (char_ptr == NULL)
            char_ptr = HardcodeFont::symbols_glyphs + HardcodeFont::GLYPH_HEIGHT * 10;

        for (int i = 0; i < HardcodeFont::GLYPH_HEIGHT; i++)
        {
            for (int j = 0; j < HardcodeFont::GLYPH_WIDTH; j++)
            {
                if (char_ptr[i] & (1 << (HardcodeFont::GLYPH_WIDTH - j - 1)))
                {
                    int rx = x + scale * j;
                    int ry = y + scale * i;

                    for (int dy = 0; dy < scale; dy++)
                    {
                        for (int dx = 0; dx < scale; dx++)
                        {
                            if (mTarget->isValidCoord(ry + dy, rx + dx))
                        	    mTarget->element(ry + dy, rx + dx) = color;
                        }
                    }

                }
            }
        }
    }

    void drawFormat(uint16_t x, uint16_t y, ElementType color, int scale, const char *format, ...)
    {
        char str[1024];
        va_list marker;
        va_start(marker, format);
        vsnprintf(str, CORE_COUNT_OF(str), format, marker);

        int i = 0;
        while (str[i] != 0)
        {
            drawChar(x + i * 6 * scale, y, str[i], color, scale);
            i++;
        }
        va_end(marker);
    }

    int  drawGlyph(int16_t dx, int16_t dy, uint8_t theChar, ElementType color, int scale)
    {
        int id = 0;
        if (theChar >= ' ' || theChar <= '~')
        {
            id = theChar - ' ';
        }

        HersheyVectorFont::Glyph *glyph = &HersheyVectorFont::data[id];
        int oldx = -1;
        int oldy = -1;
        //printf("(%d)\n", glyph->len);

        for (int i = 0; i < glyph->len; i++)
        {
            int x = glyph->data[i * 2];
            int y = glyph->data[i * 2 + 1];

          //printf("[%d %d][%d %d]\n", x,y, oldx, oldy);
            if (x != -1 && y != -1 && oldx != -1 && oldy != -1)
            {
                mTarget->drawLine(dx + oldx * scale, dy - oldy * scale, dx + x * scale, dy - y * scale, color);
            }
            oldx = x;
            oldy = y;
        }
        return glyph->width * scale;
    }


    void drawFormatVector(int16_t x, int16_t y, ElementType color, int scale, const char *format, ...)
    {
        char str[1024];
        va_list marker;
        va_start(marker, format);
        vsnprintf(str, CORE_COUNT_OF(str), format, marker);

        int i = 0;
        int dx = 0;
        while (str[i] != 0)
        {
            dx += drawGlyph(x + dx, y, str[i++], color, scale);
        }
        va_end(marker);
    }

	virtual ~AbstractPainter() {}
};

/* TODO: try msvc to feed this...
template<class TargetBuffer>
void AbstractPainter<TargetBuffer>::drawFormatVector(int16_t x, int16_t y, AbstractPainter<TargetBuffer>::ElementType color, int scale, const char *format, ...)
{
    char str[1024];
    va_list marker;
    va_start(marker, format);
    vsnprintf(str, CORE_COUNT_OF(str), format, marker);

    int i = 0;
    int dx = 0;
    while (str[i] != 0)
    {
        dx += drawGlyph(x + dx, y, str[i++], color, scale);
    }
    va_end(marker);
}
*/


} /* namespace core3vi */

/* EOF */
