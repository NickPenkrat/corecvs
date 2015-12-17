/*
 * This test undistorts images from calibrationJob with estimated params
 */
#include <vector>
#include <string>
#include <sstream>

#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"
#include "abstractPainter.h"


struct IntersectionFunctor : corecvs::FunctionArgs
{
    void operator() (const double *in, double *out)
    {
        corecvs::Vector3dd point(in[0], in[1], in[2]);
        int argout = 0;
        for (auto& r: rays)
        {
            r.normalise();
            out[argout++] = !(point - r.projectOnRay(point));
        }
    }
    IntersectionFunctor(std::vector<corecvs::Ray3d> &rays) : corecvs::FunctionArgs(3, rays.size()), rays(rays)
    {
    }
    std::vector<corecvs::Ray3d> rays;
};

corecvs::Vector3dd intersect(std::vector<corecvs::Ray3d> &rays)
{
    IntersectionFunctor iff(rays);
    corecvs::LevenbergMarquardt lm(10000);
    lm.f = &iff;
    std::vector<double> in(3), out(rays.size());
    auto res = lm.fit(in, out);
    return corecvs::Vector3dd(res[0], res[1], res[2]);
}


int main(int argc, char **argv)
{
    std::string filenameIn = "job.json";

    if (argc >= 2)
    {
        filenameIn = std::string(argv[1]);
    }

    std::cout << "Reading job from " << filenameIn << std::endl;

    CalibrationJob job;
    JSONGetter getter(filenameIn.c_str());
    getter.visit(job, "job");
    job.photostation.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    job.photostation.location.shift = corecvs::Vector3dd(0, 0, 0);

    int N = job.photostation.cameras.size();

#ifndef WIN32 // I hope that at some time visual studio will start support c11 features
    int map_fwd[N], map_bwd[N];
#else
    std::vector<int> map_fwd(N), map_bwd(N);
#endif

    int found = 0;
    for (int i = 0; i < N; ++i)
    {
        std::stringstream ss;
        ss << i;
        for (int j = 0; j < N; ++j)
        {
            if (job.photostation.cameras[j].nameId == ss.str())
            {
                map_fwd[i] = j;
                map_bwd[j] = i;
                found++;
            }
        }
    }
    CORE_ASSERT_TRUE_S(found == N);

    /*
     * Y: Output distances to (0, 0, 0)
     */
    for (int i = 0; i < job.photostation.cameras.size(); ++i)
    {
        auto cam = job.photostation.getRawCamera(map_fwd[i]);
        std::cout << "|_." << cam.nameId << "|" << !cam.extrinsics.position << "|" << std::endl;
    }

    /*
     * Z: Output distances to Z=0
     */
    for (int i = 0; i < 6; ++i)
    {
        auto cam = job.photostation.getRawCamera(map_fwd[i]);
        std::cout << "|_." << cam.nameId << "|" << cam.extrinsics.position[2] << "|" << std::endl;
    }


    /*
     * A: Output distances between cameras
     */
    std::cout << "|";
    for (int i = 0; i < N; ++i)
        std::cout << "_." << i << "|";
    std::cout << std::endl;
    for (int i = 0; i < N; ++i)
    {
        std::cout << "|_." << job.photostation.cameras[map_fwd[i]].nameId << "|";
        for (int j = 0; j < N; ++j)
        {
            std::cout << !(job.photostation.cameras[map_fwd[i]].extrinsics.position - job.photostation.cameras[map_fwd[j]].extrinsics.position) << "|";
        }
        std::cout << std::endl;
    }

    /*
     * B: Output angles between optical axes and (0, 0, 1)
     */
    std::cout << std::endl;
    for (int i = 0; i < N; ++i)
    {
        auto cam = job.photostation.getRawCamera(map_fwd[i]);
        auto dir = cam.rayFromPixel(cam.intrinsics.principal).a;
        double angle = dir.angleTo(corecvs::Vector3dd(0, 0, 1));
        std::cout << "|_." << cam.nameId << "|" << angle * 180.0 / M_PI << "|" << std::endl;
    }
    
    /*
     * C: Output distances between cameras
     */
    std::cout << "|";
    for (int i = 0; i < N; ++i)
        std::cout << "_." << i << "|";
    std::cout << std::endl;
    for (int i = 0; i < N; ++i)
    {
        std::cout << "|_." << job.photostation.cameras[map_fwd[i]].nameId << "|";
        for (int j = 0; j < N; ++j)
        {
            auto cam1= job.photostation.getRawCamera(map_fwd[i]);
            auto dir1= cam1.rayFromPixel(cam1.intrinsics.principal).a;
            auto cam2= job.photostation.getRawCamera(map_fwd[j]);
            auto dir2= cam2.rayFromPixel(cam2.intrinsics.principal).a;
            std::cout << dir1.angleTo(dir2) * 180.0 / M_PI << "|";
        }
        std::cout << std::endl;
    }


    /*
     * D: Detect camera angles intersections
     */
    if (job.photostation.cameras.size() <= 6)
        return 0;
    std::vector<corecvs::Ray3d> raysLow, raysHigh;
    for (int i = 0; i < 6; ++i)
    {
        auto cam = job.photostation.getRawCamera(map_fwd[i]);
        raysHigh.emplace_back(cam.rayFromPixel(cam.intrinsics.principal));
    }
    for (int i = 7; i < job.photostation.cameras.size(); ++i)
    {
        auto cam = job.photostation.getRawCamera(map_fwd[i]);
        raysLow.emplace_back(cam.rayFromPixel(cam.intrinsics.principal));
    }

    std::cout << "Low-high :" << !(intersect(raysLow) - intersect(raysHigh)) << std::endl;
    return 0;
}
