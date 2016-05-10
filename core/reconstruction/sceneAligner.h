#ifndef SCENEALIGNER
#define SCENEALIGNER

#include "reconstructionFixtureScene.h"

namespace corecvs
{
/*
 * This class implements some algorithms for aligning
 * scene w.r.t. last placed multicamera.
 *
 * There are following cases:
 * 1) Scene is already 3d-aligned
 * 2) Scene contains 0..2 gps-initialized cameras and new camera is fixed/static     => shift + rotation
 * 5) Scene contains 0 gps-initialized cameras and new camera is gps                 => shift
 * 6) Scene contains 1 gps-initialized camera  and new camera is gps                 => rotation that fits segment
 * 7) Scene contains 2 gps-initialized cameras and new camera is gps                 => rotation that fits triangle
 * 8) Scene contains 0..2 gps-initialized cameras and new camera is none-initialized => no action taken
 *
 * After 2 and 7 scene becomes "3D-aligned"; after determining position/orientation transformation
 * all cameras and points are converted.
 *
 * The idea is to get as close as possible to real-world coordinate system even at the start of reconstruction
 *
 * Note that before scene is 3D-aligned you cannot treat gps-initialized cameras as "fixed-position"
 */
struct SceneAligner
{
    // Returns true if non-trivial transformation is required; transformation and scale are alpha*(R*X+T) coordinate transform
    static bool TryAlign(ReconstructionFixtureScene* scene, corecvs::Affine3DQ &transformation, double &scale);
    static corecvs::Quaternion EstimateOrientationTransformation(const corecvs::Vector3dd &e1, const corecvs::Vector3dd &e2, const corecvs::Vector3dd &o1, const corecvs::Vector3dd &o2);

private:
    static bool TryAlignStatic(ReconstructionFixtureScene* scene, corecvs::Affine3DQ &transformation, double &scale);
    static bool TryAlignGPS(ReconstructionFixtureScene* scene, corecvs::Affine3DQ &transformation, double &scale);

    static corecvs::Vector3dd ShiftFromRotation(const corecvs::Quaternion &Q, const double scale, const corecvs::Vector3dd &A, const corecvs::Vector3dd &B);
    static void ApplyTransformation(ReconstructionFixtureScene* scene, const corecvs::Affine3DQ &transform, const double scale);
};
}

#endif
