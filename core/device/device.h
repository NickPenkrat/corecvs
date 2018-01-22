#ifndef DEVICE_H
#define DEVICE_H

#include "core/math/vector/vector2d.h"
#include "core/math/vector/vector3d.h"
#include "core/math/quaternion.h"
#include "core/math/matrix/matrix44.h"
#include "core/geometry/line.h"
#include "core/math/eulerAngles.h"
#include "core/math/affine.h"
#include "core/reflection/printerVisitor.h"

#include "core/math/mathUtils.h"

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