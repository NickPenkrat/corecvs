#ifndef RECONSTRUCTIONFUNCTOR
#define RECONSTRUCTIONFUNCTOR

#include "typesafeBitmaskEnums.h"

namespace corecvs
{
enum class PhotostationPlacerOptimizationType
{
    NON_DEGENERATE_ORIENTATIONS = 1, // Orientations of all cameras except first
    DEGENERATE_ORIENTATIONS = 2,     // Orientation of first camera
    NON_DEGENERATE_TRANSLATIONS = 4, // Translations of all cameras except first (TODO: Clarify if for noncentral camera we would like to fix "scale")
    DEGENERATE_TRANSLATIONS = 8,     // Translation of first camera
    FOCALS = 16,                     // Camera focals in multicamera
    PRINCIPALS = 32,                 // Camera principals in multicamera
    POINTS = 64,                     // 3D points
    TUNE_GPS = 128                   // Allow shifting of GPS-initialized cameras
};
enum class PhotostationPlacerOptimizationErrorType
{
    REPROJECTION,                    // Reprojection error
    ANGULAR,                         // Angular error
    CROSS_PRODUCT,                    // Cross product error
    RAY_DIFF
};

/*
 * Parameters are stored in the following way (section is omitted depending on PhotostationPlacerOptimizationType)
 * N - number of "preplaced" photostations
 * M - number of points
 * M'- number of projections
 * K - number of cameras in photostation
 *
 * 4            Orientation of first multicamera       DEGENERATE_ORIENTATIONS
 * 4 x (N - 1)  Orientations of multicameras           NON_DEGENERATE_ORIENTATIONS
 * 3            Translation of first multicamera       DEGENERATE_TRANSLATIONS
 * 3 x (N - 1)  Translations of multicameras           NON_DEGENERATE_TRANSLATIONS
 * 1 x K        Focal lengths of cameras               FOCALS
 * 2 x K        Principal point projections of cameras PRINCIPALS
 * 3 x M        3D points                              POINTS
 *
 * Output parameters are stored sequentially as all projections of all 3d points
 * 2 x M'       Reprojections                          ANGULAR
 * 1 x M'       Angles between rays                    REPROJECTION
 * 3 x M'       Ray cross products                     CROSS_PRODUCT
 * 2 x M'       Ray differences                        RAY_DIFF
 * If TUNE_GPS is present then position differences are also being calculated and stored as
 * 3 x N        Normalised (using covariance 'square root') difference
 */
template<>
struct is_bitmask<PhotostationPlacerOptimizationType> : std::true_type {};


}


#endif
