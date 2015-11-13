#ifndef MULTIPHOTOSTATIONSCENE
#define MULTIPHOTOSTATIONSCENE

#include "reconstructionStructs.h"

#include <vector>

#include "tbbWrapper.h"
#include "calibrationPhotostation.h"
#include "mesh3d.h"

struct MultiPhotostationScene
{
    std::vector<PointObservation__> pointObservations;
    std::vector<corecvs::Photostation> photostations;
    std::vector<std::vector<CameraObservation> > cameraObservations;

    corecvs::Vector2dd computeReprojectionError (int ps, int cam, const corecvs::Vector3dd &point, const corecvs::Vector2dd &expected) const;
    void   computeReprojectionErrors(int ps, int cam, std::vector<double> &errors) const;
    void   computeReprojectionErrors(std::vector<double> &errors) const;
    corecvs::Vector2dd project(int ps, int cam, const corecvs::Vector3dd &point) const;

    static corecvs::Vector3dd BackProject(const std::vector<std::pair<corecvs::Matrix44, corecvs::Vector2dd>> &points);

    corecvs::Vector3dd backProject(const std::vector<std::pair<std::pair<int, int>, corecvs::Vector2dd>> &points) const;
    corecvs::Vector3dd backProject(const std::vector<PointProjection> &projections, bool updateable = true) const;

    struct ParallelUpdater
    {
        MultiPhotostationScene* scene;
        void operator() (const corecvs::BlockedRange<int> &r) const;

        ParallelUpdater(decltype(scene) scene) : scene(scene)
        {
        }
    };
    void updateBackProjections();

    int getPhotostationCount() const;
    
    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(pointObservations,  "pointObservations");
        visitor.visit(photostations,      "photostations");
        visitor.visit(cameraObservations, "cameraObservations");
    }

    // TODO: Some nasty (and slow) stuff happens here.
    //       Probably, we should remove it or do it on
    //       keypoint detection stage
    struct ParallelColoriser
    {
        std::vector<corecvs::RGBColor> *colors_;
        MultiPhotostationScene* scene_;

        ParallelColoriser(decltype(colors_) colors_, decltype(scene_) scene_) : colors_(colors_), scene_(scene_)
        {
        }
        
        void operator() (const corecvs::BlockedRange<int> &range) const;
    };

    void drawPly(corecvs::Mesh3D &mesh);

    corecvs::Mesh3D drawPly();
};


#endif
