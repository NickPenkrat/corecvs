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
void draw(corecvs::ObservationList list, corecvs::RGB24Buffer &buffer)
{
    double minh = 1e10,  minv = 1e10, maxh = 0, maxv = 0;
    for (int i = 0; i < list.size(); ++i)
    {
        auto el = list[i].point;
        if (el[0] < minv) minv = el[0];
        if (el[1] < minh) minh = el[1];
        if (el[0] > maxv) maxv = el[0];
        if (el[1] > maxh) maxh = el[1];

    }

    for (int i = 0; i < list.size(); ++i)
    {
        auto el = list[i].point;
        auto el2= list[i].projection;
        if ((el[0] == minv || el[0] == maxv)
                &&(el[1] == minh || el[1] == maxh))
        {
            corecvs::AbstractPainter<corecvs::RGB24Buffer> p(&buffer);
            p.drawFormat(el2.x(), el2.y(), corecvs::RGBColor(0xffff00), 4, "(%.0f, %.0f)", el[0] / 50.0, el[1] / 50.0);
            for (int j = -4; j <= 4; ++j)
                for(int k = -4; k <= 4; ++k)
                {
                    buffer.element(((int)el2.y()) + j, ((int)el2.x())+k) = corecvs::RGBColor(0x0000ff);
                }
        }
    }

}

int main(int argc, char **argv)
{
    std::string filenameIn = "job.json";
    std::string filenameOut= "job.json";

    if (argc >= 2)
    {
        filenameIn = std::string(argv[1]);
    }
    if (argc >= 3)
    {
        filenameOut = std::string(argv[2]);
    }

    std::cout << "Reading job from " << filenameIn << std::endl <<
                 "Saving job to " << filenameOut << std::endl;

    CalibrationJob job;
    JSONGetter getter(filenameIn.c_str());
    getter.visit(job, "job");


    for (auto& pv: job.observations)
    {
        for (auto&p : pv)
        {
            auto buff = CalibrationJob::LoadImage(p.undistortedFileName);
            draw(p.undistortedPattern, buff);
            std::stringstream ss;
            ss << p.undistortedFileName << "corners.jpg";
            CalibrationJob::SaveImage(ss.str(), buff);
        }
    }


    std::vector<int> cic, rsc;
    std::vector<std::vector<int>> ccr;
    int rps;
    job.calculateRedundancy(cic, ccr, rsc, rps);
    std::cout << "Intrinsics calibration redundancy:" << std::endl;
    for (size_t i = 0; i < cic.size(); ++i)
    {
        std::cout << "Camera #" << i << " has " << cic[i] << " images and " << (rsc[i] ? " can " : " can not ") << "be calibrated" << std::endl;
    }
    std::cout << std::endl << "Photostation calibration: ";
    if (rps >= 0)
        std::cout << " has " << rps << " redundant views and can be calibrated" << std::endl;
    else
        std::cout << " lacks at least " << -rps << " views and can not calibrated" << std::endl;
    std::cout << "More details: " << std::endl;
    for (size_t i = 0; i < ccr.size(); ++i)
    {
        std::cout << "Camera #" << i << " has common views with: ";
        for (auto& cc: ccr[i])
            std::cout << cc << ", ";
        std::cout << std::endl;
    }

    job.calibrate();

    JSONSetter setter(filenameOut.c_str());
    setter.visit(job, "job");
    return 0;
}
