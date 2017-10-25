#ifndef DEVICE_H
#define DEVICE_H

#include "vector2d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "matrix44.h"
#include "line.h"
#include "eulerAngles.h"
#include "affine.h"
#include "printerVisitor.h"

#include "mathUtils.h"

namespace corecvs {
    class Device
    {
    public:
        Vector3dd position;
        Quaternion orientation;
        std::string nameId;

        template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(nameId, std::string(""), "nameId");
            visitor.visit(position, Vector3dd(0.0, 0.0, -1.0), "position");
            visitor.visit(orientation, Quaternion::Identity(), "orientation");
        }

    };
}

#endif // DEVICE_H