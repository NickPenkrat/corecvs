#ifndef DEVICE_H
#define DEVICE_H

#include <string>

#include "core/math/vector/vector3d.h"
#include "core/math/quaternion.h"

namespace corecvs {

class Device
{
public:
    Device(const std::string &name = "", const Vector3dd &pos = Vector3dd(0), const Quaternion &orient = Quaternion::Identity())
        :nameId(name), position(pos), orientation(orient)
    {}

    std::string nameId;
    Vector3dd   position;
    Quaternion  orientation;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(nameId,       std::string(""),            "nameId");
        visitor.visit(position,     Vector3dd(0.0, 0.0, -1.0),  "position");
        visitor.visit(orientation,  Quaternion::Identity(),     "orientation");
    }
};

} // namespace corecvs

#endif // DEVICE_H
