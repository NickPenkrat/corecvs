#ifndef CALIBRATIONFEATUREPOINT_H
#define CALIBRATIONFEATUREPOINT_H

#include <string>
#include <unordered_map>

#include "calibrationCamera.h"


/* Presentation related */
#include "rgbColor.h"

namespace corecvs {

class CalibrationFeaturePoint;

class CalibrationObservation {
public:
    CameraModel *             camera;
    CalibrationFeaturePoint * featurePoint;

    Vector2dd                 observation;
    Vector2dd                 accuracy;
    bool                      isKnown;
    MetaContainer             meta;
};


class CalibrationFeaturePoint
{
public:
    std::string name;

    /** This is a primary position of the FeaturePoint. The one that is know by direct measurement */
    Vector3dd position;
    bool hasKnownPosition;

    /** This is a position that is achived by reconstruction */
    Vector3dd reprojectedPosition;
    bool hasKnownReprojectedPosition;

    enum PointType{
        POINT_USER_DEFINED  = 0x01,  /**< Point that comes from a file */
        POINT_RECONSTRUCTED = 0x02,
        POINT_TEMPORARY     = 0x04,

        POINT_ALL           = 0xFF
    };

    PointType type;

    /** Observation related block */
    typedef std::unordered_map<CameraModel *, CalibrationObservation> ObservContainer;
    ObservContainer observations;




/* This is a presentation related block we should move it to derived class */
    RGBColor color;
/**/

    CalibrationFeaturePoint() :
        hasKnownPosition(false)
    {}


    CalibrationFeaturePoint(Vector3dd _position, const std::string &_name = std::string()) :
        name(_name),
        position(_position),
        hasKnownPosition(true)
    {}

    void transform(const Matrix33 &matrix, const Vector3dd &translate)
    {
        position = matrix * position + translate;
    }


};

}

#endif // CALIBRATIONFEATUREPOINT_H
