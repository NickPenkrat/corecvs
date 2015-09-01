/**
 * \file aLowCodec.cpp
 * \brief Add Comment Here
 *
 * \date Sep 1, 2015
 * \author sfedorenko
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "global.h"

#include "aLowCodec.h"

/** The coding rules are taken from the document "AR1820HS_DS_G.pdf", page 93
 */
inline uint8_t code12to8(uint16_t value)
{
  /*if (value <   32) return value; */                      // 0 0 0 0 0 0 0 a b c d e => 000abcde
    if (value <   64) return value;                         // 0 0 0 0 0 0 1 a b c d e => 001abcde
    if (value <  128) return ((value >> 1) & 0x1f) | 0x40;  // 0 0 0 0 0 1 a b c d e X => 010abcde
    if (value <  256) return ((value >> 2) & 0x1f) | 0x60;  // 0 0 0 0 1 a b c d e X X => 011abcde
    if (value <  512) return ((value >> 3) & 0x1f) | 0x80;  // 0 0 0 1 a b c d e X X X => 100abcde
    if (value < 1024) return ((value >> 4) & 0x1f) | 0xA0;  // 0 0 1 a b c d e X X X X => 101abcde
    if (value < 2048) return ((value >> 5) & 0x1f) | 0xC0;  // 0 1 a b c d e X X X X X => 110abcde
    if (value < 4096) return ((value >> 6) & 0x1f) | 0xE0;  // 1 a b c d e X X X X X X => 111abcde
    ASSERT_FAIL("code12to8: value is out of range");
}

/** The decoding rules are taken from the document "AR1820HS_DS_G.pdf", page 94
*/
inline uint16_t decode8to12(uint8_t value)
{
    switch (value & 0xE0) {
        case 0x00:  return value;                              // 000abcde => 0 0 0 0 0 0 0 a b c d e
        case 0x20:  return value;                              // 001abcde => 0 0 0 0 0 0 1 a b c d e
        case 0x40:  return ((value & 0x1f) << 1) | 0x041;      // 010abcde => 0 0 0 0 0 1 a b c d e 1
        case 0x60:  return ((value & 0x1f) << 2) | 0x082;      // 011abcde => 0 0 0 0 1 a b c d e 1 0
        case 0x80:  return ((value & 0x1f) << 3) | 0x104;      // 100abcde => 0 0 0 1 a b c d e 1 0 0
        case 0xA0:  return ((value & 0x1f) << 4) | 0x208;      // 101abcde => 0 0 1 a b c d e 1 0 0 0
        case 0xC0:  return ((value & 0x1f) << 5) | 0x410;      // 110abcde => 0 1 a b c d e 1 0 0 0 0
        case 0xE0:  return ((value & 0x1f) << 6) | 0x820;      // 111abcde => 1 a b c d e 1 0 0 0 0 0
    }
    ASSERT_FAIL("decode8to12: value is out of range");
}

RGB24Buffer *ALowCodec::code(const RGB24Buffer *rgb24buffer)
{
    RGB24Buffer *toReturn = new RGB24Buffer((RGB24Buffer *)rgb24buffer);

    toReturn->binaryOperationInPlace(*rgb24buffer, [](const RGBColor &/*a*/, const RGBColor &rgb) {
        uint8_t r = rgb.r();
        uint8_t g = rgb.g();
        uint8_t b = rgb.b();
        r = code12to8((uint16_t)r << 4);
        g = code12to8((uint16_t)g << 4);
        b = code12to8((uint16_t)b << 4);
        return RGBColor(r, g, b);
    });

    return toReturn;
}

RGB24Buffer *ALowCodec::decode(const RGB24Buffer *rgb24buffer)
{
    //int w = 2592;
    //int h = 1944;
    //RGB24Buffer *toReturn = new RGB24Buffer(h, w, false);

    RGB24Buffer *toReturn = new RGB24Buffer((RGB24Buffer *)rgb24buffer);

    toReturn->binaryOperationInPlace(*rgb24buffer, [](const RGBColor &/*a*/, const RGBColor &rgb) {
        uint8_t r = rgb.r();
        uint8_t g = rgb.g();
        uint8_t b = rgb.b();
        r = decode8to12(r) >> 4;
        g = decode8to12(g) >> 4;
        b = decode8to12(b) >> 4;
        return RGBColor(r, g, b);
    });

    return toReturn;
}
