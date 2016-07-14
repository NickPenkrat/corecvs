/*
 * This test undistorts images from calibrationJob with estimated params
 */
#include <vector>
#include <string>
#include <sstream>

#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

int main(int argc, char **argv)
{
    std::string filenameIn = "job.json";
    std::string filenameOut= "job.json";
    //bool undistorted = false;

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

#if 0
    job.allRemoveDistortion();

    JSONSetter setter(filenameOut.c_str());
    setter.visit(job, "job");
#endif
	for (int i = 0; i < 10; ++i)
	{
		std::cout << i << ":" << std::endl;
		auto &cam = job.photostation.cameras[0];
		std::cout << cam.intrinsics.principal[0]  << ", " << cam.intrinsics.principal[1] << ", " << cam.distortion.mShiftX << ", " << cam.distortion.mShiftY << std::endl;
		cam.estimateUndistortedSize(DistortionApplicationParameters());
	}
    return 0;
}
