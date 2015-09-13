#include <vector>
#include <cassert>
#include <string>

#include "calibrationJob.h"
#include "jsonGetter.h"

int main(int argc, char **argv)
{
    std::string name = "job.json";
    if (argc > 1)
        name = argv[1];

    CalibrationJob job;
    JSONGetter *get1 = new JSONGetter(name.c_str());
    get1->visit(job, "job");
    delete get1;

    auto ps = job.photostation;
    auto ss = job.calibrationSetupLocations;
    int  N  = (int)ps.cameras.size();
    int  M  = (int)ss.size();
    // Optical axis deviation from "vertical"
    Matrix T(N, 3);
    for (int i = 0; i < N; ++i)
    {
        Vector3dd v = ps.cameras[i].extrinsics.orientation.conjugated() * Vector3dd(0, 0, 1);
        for (int j = 0; j < 3; ++j)
        {
            T.a(i, j) = v[j];
        }
    }
    Matrix D(1, 3), V(3, 3);
    Matrix::svd(&T, &D, &V);
    double min_singular = D.a(0, 0);
    int id = 0;
    for (int i = 1; i < 3; ++i)
    {
        if (D.a(0, i) < min_singular)
        {
            min_singular = D.a(0, i);
            id = i;
        }
    }

    std::cout << "|_.Cam|_.angle|" << std::endl;
    Vector3dd m_axis(V.a(0, id), V.a(1, id), V.a(2, id));
    for (int i = 0; i < N; ++i)
    {
        std::cout << "|" << i << "|" << 180.0 * acos((ps.cameras[i].extrinsics.orientation.conjugated() * Vector3dd(0, 0, 1)) & m_axis) / M_PI - 90.0 << "|" << std::endl;
    }
    std::cout << std::endl;


    // Relative camera poses
    std::cout << "Relative camera poses" << std::endl;
	for (auto& cam: ps.cameras)
    {
        std::cout << cam.extrinsics.position << std::endl;
    }
#if 0
    for (int i = 0; i < M; ++i)
    {
        std::stringstream s;
        s << "mesh_" << M_start + i * M_by << "deg.ply";
        ps.location = ss[i];
        drawPly(ps, s.str());
    }
#endif
	// Cam2cam distances 
    std::cout << "Cam2cam distances" << std::endl;
    std::cout << "|_.Cam #1|_.Cam #2|_.Distance (mm)|" << std::endl;
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            std::cout << "|" << i << "|" << j << "|" << (ps.cameras[i].extrinsics.position - ps.cameras[j].extrinsics.position).l2Metric() << "|" << std::endl;
        }
    }
    std::cout << std::endl;

    // Camera intrinsics
    std::cout << "Final camera intrinsics" << std::endl;
    std::cout << "|_.Cam|_.fx|_.fy|_.fx/fy|_.skew|_.atan(skew/fy)|_.cx|_.cy|" << std::endl;
    for (int i = 0; i < N; ++i)
    {
        double fx = ps.cameras[i].intrinsics.fx;
        double fy = ps.cameras[i].intrinsics.fy;
        double cx = ps.cameras[i].intrinsics.cx;
        double cy = ps.cameras[i].intrinsics.cy;
        double skew=ps.cameras[i].intrinsics.skew;
        double aspect = fx / fy;
        double angle = atan(skew / fx) * 180.0 / M_PI;

        std::cout << "|" << i << "|" << fx << "|" << fy << "|" << aspect << "|" << skew << "|" << angle << "|" << cx << "|" << cy << "|" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Errors: " << std::endl;
    std::cout << "|_.Cam#|_.file|_.Distortion RMSE|_.Distortion max error|_.Calibration RMSE|_.Calibration max error|" << std::endl;
    for (auto &o: job.calibrationSetups)
    {
        for (auto &s: o)
        {
            auto& view = job.observations[s.cameraId][s.imageId];
            std::cout << "|" << s.cameraId << "|" << view.sourceFileName << "|" << view.distortionRmse << "|" << view.distortionMaxError << "|" << view.calibrationRmse << "|" << view.calibrationMaxError << "|" << std::endl;
        }
    }
    return 0;
}
