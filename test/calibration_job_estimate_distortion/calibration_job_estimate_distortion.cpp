/*
 * This test updates calibrationJob with estimated distortion params
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
    bool undistorted = false;

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

    job.allEstimateDistortion();

    JSONSetter setter(filenameOut.c_str());
    setter.visit(job, "job");
    return 0;
}
