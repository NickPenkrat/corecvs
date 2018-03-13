#ifndef PROJECTIONFACTORY_H
#define PROJECTIONFACTORY_H

#include "core/cameracalibration/projection/pinholeCameraIntrinsics.h"
#include "core/cameracalibration/projection/equidistantProjection.h"
#include "core/cameracalibration/projection/equisolidAngleProjection.h"
#include "core/cameracalibration/projection/omnidirectionalProjection.h"
#include "core/cameracalibration/projection/stereographicProjection.h"


namespace corecvs {


/**
 * Serialization is a bit more tricky. So far - stupid approach
 *
 * This class is a wrapper that is able to manually transform dinamic polimorphism into static for ProjectionType hierarchy
 *
 **/
class ProjectionFactory {
public:
    std::unique_ptr<CameraProjection> &target;

    ProjectionFactory(std::unique_ptr<CameraProjection> &target) :
        target(target) {}


    static CameraProjection     *projectionById(const ProjectionType::ProjectionType &projection);
    static Reflection           *reflectionById(const ProjectionType::ProjectionType &projection);

    template<class Visitor>
    void accept(Visitor &visitor)
    {

        int projectionNum = (int)target->projection;
        visitor.visit((int&)projectionNum, (int)ProjectionType::PINHOLE, "projectionType");
        ProjectionType::ProjectionType projectionType = (ProjectionType::ProjectionType)projectionNum;

        if (projectionType != target->projection)
        {
            target.reset(projectionById(projectionType));
        }

        switch (projectionType) {
            case  ProjectionType::PINHOLE:
                static_cast<PinholeCameraIntrinsics *>(target.get())->accept<Visitor>(visitor); break;
            case  ProjectionType::EQUIDISTANT:
                static_cast<EquidistantProjection *>  (target.get())->accept<Visitor>(visitor); break;
            case  ProjectionType::OMNIDIRECTIONAL:
                static_cast<OmnidirectionalProjection *> (target.get())->accept<Visitor>(visitor); break;
            case  ProjectionType::STEREOGRAPHIC:
                static_cast<StereographicProjection *>(target.get())->accept<Visitor>(visitor); break;
            case  ProjectionType::EQUISOLID:
                static_cast<EquisolidAngleProjection *>(target.get())->accept<Visitor>(visitor); break;
            /*case  CameraProjection::ORTHOGRAPIC:
                static_cast<EquidistantProjection *>(target.get())->accept<Visitor>(visitor); break;*/
            default:
                break;

        }
    }




};

} // namespace corecvs

#endif // PROJECTIONFACTORY_H
