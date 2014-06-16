#ifndef OUTPUT_STYLE_H_
#define OUTPUT_STYLE_H_
/**
 * \file outputStyle.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

/**
 * Helper namespace to hide OutputStyle enum from global context.
 */

namespace OutputStyle {

/** 
 * \brief Output Style 
 * Output Style 
 */
enum OutputStyle {
    /** 
     * \brief right Frame 
     * Right Frame 
     */
    RIGHT_FRAME = 0,
    /** 
     * \brief left Frame 
     * Left Frame 
     */
    LEFT_FRAME = 1,
    /** 
     * \brief sidebyside stereo 
     * Side-by-side stereo 
     */
    SIDEBYSIDE_STEREO = 2,
    /** 
     * \brief anaglyph RG 
     * Red-Green Anaglyph 
     */
    ANAGLYPH_RG = 3,
    /** 
     * \brief anaglyph RC 
     * Red-Cyan Anaglyph 
     */
    ANAGLYPH_RC = 4,
    /** 
     * \brief blend 
     * Blend 
     */
    BLEND = 5,
    /** 
     * \brief none 
     * None 
     */
    NONE = 6,
    /** 
     * \brief Last virtual option to run cycles to
     */
    OUTPUT_STYLE_LAST
};


static inline const char *getName(const OutputStyle &value)
{
    switch (value) 
    {
     case RIGHT_FRAME : return "RIGHT_FRAME"; break ;
     case LEFT_FRAME : return "LEFT_FRAME"; break ;
     case SIDEBYSIDE_STEREO : return "SIDEBYSIDE_STEREO"; break ;
     case ANAGLYPH_RG : return "ANAGLYPH_RG"; break ;
     case ANAGLYPH_RC : return "ANAGLYPH_RC"; break ;
     case BLEND : return "BLEND"; break ;
     case NONE : return "NONE"; break ;
     default : return "Not in range"; break ;
     
    }
    return "Not in range";
}

} //namespace OutputStyle

#endif  //OUTPUT_STYLE_H_
