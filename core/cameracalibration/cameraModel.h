#ifndef CAMERAMODEL_H
#define CAMERAMODEL_H

#include "calibrationLocation.h"
#include "core/buffers/displacementBuffer.h"
#include "core/xml/generated/distortionApplicationParameters.h"
#include "core/rectification/essentialMatrix.h"
#include "core/alignment/lensDistortionModelParameters.h"
#include "core/alignment/pointObservation.h"
#include "core/geometry/polygons.h"

#include "core/alignment/selectableGeometryFeatures.h"


#include "core/cameracalibration/projection/pinholeCameraIntrinsics.h"
#include "core/cameracalibration/projection/equidistantProjection.h"
#include "core/cameracalibration/projection/equisolidAngleProjection.h"
#include "core/cameracalibration/projection/catadioptricProjection.h"
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


    static CameraProjection *projectionById(CameraProjection::ProjectionType &projection)
    {
        switch (projection) {
            case  CameraProjection::PINHOLE:
            default:
                return new PinholeCameraIntrinsics();
            case  CameraProjection::EQUIDISTANT:
                return new EquidistantProjection();
            /*case  CameraProjection::CATADIOPTRIC:
                return new CatadioptricProjection();*/
            case  CameraProjection::STEREOGRAPHIC:
                return new StereographicProjection();
            case  CameraProjection::EQUISOLID:
                return new EquisolidAngleProjection();
        }
        return NULL;
    }

    template<class Visitor>
    void accept(Visitor &visitor)
    {

        CameraProjection::ProjectionType projection = target->projection;
        visitor.visit((int&)projection, (int)CameraProjection::PINHOLE, "projectionType");

        if (projection != target->projection)
        {
            target.reset(projectionById(projection));
        }

        switch (projection) {
            case  CameraProjection::PINHOLE:
                static_cast<PinholeCameraIntrinsics *>(target.get())->accept<Visitor>(visitor); break;
            case  CameraProjection::EQUIDISTANT:
                static_cast<EquidistantProjection *>  (target.get())->accept<Visitor>(visitor); break;
            case  CameraProjection::CATADIOPTRIC:
                static_cast<CatadioptricProjection *> (target.get())->accept<Visitor>(visitor); break;
            case  CameraProjection::STEREOGRAPHIC:
                static_cast<StereographicProjection *>(target.get())->accept<Visitor>(visitor); break;
            case  CameraProjection::EQUISOLID:
                static_cast<EquisolidAngleProjection *>(target.get())->accept<Visitor>(visitor); break;
            /*case  CameraProjection::ORTHOGRAPIC:
                static_cast<EquidistantProjection *>(target.get())->accept<Visitor>(visitor); break;*/
            default:
                break;

        }
    }

};

class CameraModel
{
public:
    /**/
    //PinholeCameraIntrinsics  intrinsics;
    std::unique_ptr<CameraProjection> intrinsics;
    /**/
    LensDistortionModelParameters   distortion;
    /**/
    CameraLocationData              extrinsics;

    /* cache for rotation should be introduced. First of all it is faster... */
    //Matrix33 rotMatrix;

public:

    /* This should be moved to the derived class */
    /*RGB24Buffer    *image;*/
    std::string     nameId;

public:
    CameraModel(CameraProjection *projecton = new PinholeCameraIntrinsics) :
        intrinsics(projecton)
    {}

    CameraModel(
            const PinholeCameraIntrinsics &_intrinsics,
            const CameraLocationData &_extrinsics = CameraLocationData(),
            const LensDistortionModelParameters &_distortion = LensDistortionModelParameters())
      : intrinsics(_intrinsics.clone())
      , distortion(_distortion)
      , extrinsics(_extrinsics)
    {}

    CameraModel(const CameraModel &other)
    {
        copyModelFrom(other);
    }

    CameraModel &operator =(const CameraModel &other)
    {
        copyModelFrom(other);
        return *this;
    }



    template <bool full=false>
    Vector2dd project(const Vector3dd &p) const
    {
        Vector2dd v = intrinsics->project(extrinsics.project(p));
        if (full)
            return distortion.mapForward(v);
        return v;
    }

    Vector2dd project(const Vector3dd &p, bool full) const
    {
        return full ? project<true>(p) : project<false>(p);
    }

    Vector2dd reprojectionError(PointObservation &observation)
    {
        return observation.projection - project(observation.point);
    }

    Vector2dd reprojectionError(const Vector3dd &p, const Vector2dd &pp) const
    {
        return intrinsics->reprojectionError(extrinsics.project(p), pp);
    }

    Vector3dd crossProductError(const Vector3dd &p, const Vector2dd &pp)
    {
        return intrinsics->crossProductError(extrinsics.project(p), pp);
    }

    /**
     *   Depricated
     **/
   /* double angleError(const Vector3dd &p, const Vector2dd &pp)
    {
        return intrinsics->angleError(extrinsics.project(p), pp);
    }*/

    Vector3dd rayDiffError(const Vector3dd &p, const Vector2dd &pp)
    {
        bool fail = std::isnan(p[2]);
        for (int i = 0; i < 2; ++i)
            fail |= std::isnan(p[i]) || std::isnan(pp[i]);
        if (fail)
            std::cout << "CAM:" << p << " " << pp << std::endl;
        return intrinsics->rayDiffError(extrinsics.project(p), pp);
    }

    /**
     * Return a direction in camera corrdinate frame passing though a point p.
     *
     * This only uses extrinsics and provides the way to intercept the projection process before
     * intrinsics application take action
     *
     * \param p
     **/
    Vector3dd dirToPoint(const Vector3dd &p)
    {
        return extrinsics.project(p);
    }

    Matrix33 fundamentalTo(const CameraModel &right) const;
    static Matrix33 Fundamental(const Matrix44 &Pl, const Matrix44 &Pr);
    Matrix33 essentialTo  (const CameraModel &right) const;
    EssentialDecomposition essentialDecomposition(const CameraModel &right) const;
    static EssentialDecomposition ComputeEssentialDecomposition(const CameraLocationData &thisData, const CameraLocationData &otherData);


    PlaneFrame getVirtualScreen(double distance) const;

    /**
      double pyramidLength1
      double pyramidLength2
     **/
    Polygon projectViewport(const CameraModel &right, double pyramidLength1 = -1, double pyramidLength2 = -1) const;

    /**
     * Only checks for the fact that point belongs to viewport.
     * If you are projecting 3d point you should be sure that point is in front
     **/
    bool isVisible(const Vector2dd &point)
    {
        return point.isInRect(Vector2dd(0.0,0.0), intrinsics->size());
    }

    /**
     * Checks full visibility of 3d point
     **/
    bool isVisible(const Vector3dd &pt) const
    {
       return intrinsics->isVisible(extrinsics.project(pt));
    }

    bool isInFront(const Vector3dd &pt)
    {
        return ((pt - extrinsics.position) & forwardDirection()) > 0;
    }

    /**
     * Setups intrinsics.size and modifies distortion shift/scale
     * in a proper way
     */
    void estimateUndistortedSize(const DistortionApplicationParameters &applicationParams);

    Ray3d               rayFromPixel(const Vector2dd &point) const;
    Vector3dd           dirFromPixel(const Vector2dd &point) const
    {
        return (extrinsics.orientation.conjugated() * intrinsics->reverse(point)).normalised();
    }
    Ray3d               rayFromCenter();

    Vector3dd           forwardDirection() const;
    Vector3dd           topDirection() const;
    Vector3dd           rightDirection() const;

    Matrix33            getRotationMatrix() const;
    Vector3dd           getCameraTVector() const;
    Matrix44            getPositionMatrix() const;
    Matrix44            getCameraMatrix() const;

    /* These methods ignore distortion */
    ConvexPolyhedron    getViewport(const Vector2dd &p1, const Vector2dd &p2) const;
    ConvexPolyhedron    getCameraViewport( double farPlane = -1) const;

    /**
     * Sides of the viewport are triangles with points at infinity, so they are stored in projective space
     **/
    vector<GenericTriangle<Vector4dd>> getCameraViewportSides() const;

    /**
     * Some points of the viewport are at infinity, so they are stored in projective space
     **/
    vector<Vector4dd> getCameraViewportPyramid() const;

    Polygon getCameraViewportPolygon() const;




    void copyModelFrom(const CameraModel &other) {
        intrinsics.reset(other.intrinsics->clone());
        distortion = other.distortion;
        extrinsics = other.extrinsics;
    }

    /* This method produces camera model that is a copy, but works for downsampled image */
    CameraModel scaledModel(double scaleFactor = 0.5)
    {
        CameraModel model = *this;
        if (!model.intrinsics->isPinhole())
        {
            return model;
        }

        PinholeCameraIntrinsics *intr = static_cast<PinholeCameraIntrinsics *>(model.intrinsics.get());

        intr->scale(scaleFactor);

        model.distortion.setNormalizingFocal(model.distortion.normalizingFocal() * scaleFactor);
        model.distortion.setPrincipalPoint  (model.distortion.principalPoint() * scaleFactor);
        model.distortion.setShiftX(model.distortion.shiftX() * scaleFactor);
        model.distortion.setShiftY(model.distortion.shiftY() * scaleFactor);
        return model;
    }

    void setLocation(const Affine3DQ &location)
    {
        extrinsics = CameraLocationData(location);
    }

    Affine3DQ getAffine() const
    {
        return extrinsics.toAffine3D();
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        // MEFIXASAP
        //visitor.visit(intrinsics, PinholeCameraIntrinsics()      , "intrinsics");

        ProjectionFactory wrapper(intrinsics);
        visitor.visit(wrapper, "intrinsics");
        //visitor.visit(intrinsics, PinholeCameraIntrinsics()      , "intrinsics");

        visitor.visit(extrinsics, CameraLocationData()           , "extrinsics");
        visitor.visit(distortion, LensDistortionModelParameters(), "distortion");
        visitor.visit(nameId,     std::string("")                , "nameId"    );
    }


    friend ostream& operator << (ostream &out, CameraModel &toSave)
    {
        PrinterVisitor printer(out);
        toSave.accept<PrinterVisitor>(printer);
        return out;
    }

    DisplacementBuffer transform(const DistortionApplicationParameters &applicationParams)
    {
        estimateUndistortedSize(applicationParams);
        int newW = (int)intrinsics->w();
        int newH = (int)intrinsics->h();
        if (newH < 0 || newW < 0)
        {
            SYNC_PRINT(("invalid distortion data for camId=%s outSize(%dx%d)", nameId.c_str(), newW, newH));
            return DisplacementBuffer();
        }

        return RadialCorrection(distortion).getUndistortionTransformation(
             intrinsics->size()
            ,intrinsics->distortedSize(), 0.25, false);
    }

    void prettyPrint(std::ostream &out = cout);

    /**
       Shortcut for most popular distortion type -
            returns PinholeCameraIntrinsics intrisics if this model is pinhole
            returns NULL otherwise;
     **/
    PinholeCameraIntrinsics *getPinhole() const
    {
        if (intrinsics->isPinhole()) {
            return static_cast<PinholeCameraIntrinsics *>(intrinsics.get());
        }
        return NULL;
    }
};


typedef std::vector<ObservationList>  MultiCameraPatternPoints;


} // namespace corecvs

#endif // CAMERAMODEL_H
