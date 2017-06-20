/* ===========================================================================
* MAGNA Electronics - C O N F I D E N T I A L
*  This document in its entirety is CONFIDENTIAL and may not be disclosed,
*  disseminated or distributed to parties outside MAGNA Electronics
*  without written permission from MAGNA Electronics.
* ===========================================================================
* SHORT:   spherical_lut.h
* DESIGN:
* DESCRIPTION:
*   This file holds the declarations of the tables for spherical distortion correction.
*
* ORIGINAL AUTHOR: Werner Kozek
* OTHER AUTHORS: Alexander Pimenov
* CREATED:         Jan 17, 2011
* LAST MODIFIED:   Jan 17, 2011
*
* ======================================================================== */

#ifndef SPHERICAL_LUT_H_
#define SPHERICAL_LUT_H_

#ifdef __cplusplus
    extern "C" {
#endif


#define LUT_LEN 48
#define LUT_LEN_HD 90

extern double WarpToUnwarpLUT[LUT_LEN][2];
extern double UnwarpToWarpLUT[LUT_LEN][2];

extern double WarpToUnwarpLUT_HD[LUT_LEN_HD][2];
extern double UnwarpToWarpLUT_HD[LUT_LEN_HD][2];

#ifdef __cplusplus
    } // extern "C"
#endif


#endif /* SPHERICAL_LUT_H_ */
