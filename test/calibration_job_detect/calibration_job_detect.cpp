/*
 * This test updates calibrationJob with detected patterns
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

    if (!strcmp(argv[argc - 1], "-u") || !strcmp(argv[argc - 1], "--undistorted"))
    {
        argc--;
        undistorted = true;
    }

    if (argc >= 2)
    {
        filenameIn = std::string(argv[1]);
    }
    if (argc >= 3)
    {
        filenameOut = std::string(argv[2]);
    }

    std::cout << "Reading job from " << filenameIn << std::endl <<
                 "Saving job to " << filenameOut << std::endl <<
                 "Running on " << (undistorted ? "Undistorted" : "Source") << " images" << std::endl;

    CalibrationJob job;
    JSONGetter getter(filenameIn.c_str());
    getter.visit(job, "job");

    job.allDetectChessBoard(!undistorted);

    JSONSetter setter(filenameOut.c_str());
    setter.visit(job, "job");
    return 0;
}
