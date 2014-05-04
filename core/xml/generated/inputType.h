#ifndef INPUT_TYPE_H_
#define INPUT_TYPE_H_
/**
 * \file inputType.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

/**
 * Helper namespace to hide InputType enum from global context.
 */

namespace InputType {

/** 
 * \brief Input Type 
 * Input Type 
 */
enum InputType {
    /** 
     * \brief Left Frame 
     * Left Frame 
     */
    LEFT_FRAME = 0,
    /** 
     * \brief Right Frame 
     * Right Frame 
     */
    RIGHT_FRAME = 1,
    /** 
     * \brief Last virtual option to run cycles to
     */
    INPUT_TYPE_LAST
};


static inline const char *getName(const InputType &value)
{
    switch (value) 
    {
     case LEFT_FRAME : return "LEFT_FRAME"; break ;
     case RIGHT_FRAME : return "RIGHT_FRAME"; break ;
     default : return "Not in range"; break ;
     
    }
    return "Not in range";
}

} //namespace InputType

#endif  //INPUT_TYPE_H_
