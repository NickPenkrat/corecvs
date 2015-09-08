/*
 * This test just generates default job template for
 * current (6-camera) prototype device.
 *
 * This should be customized in order to provide
 * template jobs for all topcon-like prototypes
 */
#include <vector>
#include <string>
#include <sstream>

#include "calibrationJob.h"
#include "jsonSetter.h"

int main(int argc, char **argv)
{
    std::string filename = "job.json";
    if (argc >= 2)
    {
        filename = std::string(argv[1]);
    }

    std::cout << "Saving job file to " << filename << std::endl;

    int N = 6;
    int M = 24;
    int M_init = 360;
    int M_by = -15;

    CalibrationJob job;
    job.observations.resize(N);
    job.calibrationSetups.resize(M);

    for (int i = 0; i < M; ++i)
    {
        int deg = M_init + i * M_by;
        for (int j = 0; j < N; ++j)
        {
            int fullFrame = 360 - j * 60;
            int dist = std::min((deg - fullFrame + 360) % 360, (fullFrame - deg + 360) % 360);
            if (dist > 30)
                continue;
            
            std::stringstream fs;
            fs << "SPA" << j << "_" << deg << "deg" << ".jpg";
            std::string sourceFileName = fs.str();
            std::stringstream fu;
            fu << "SPA" << j << "_" << deg << "deg" << "_undist" << ".jpg";
            std::string undistFileName = fu.str();

            ImageData img;
            img.sourceFileName = sourceFileName;
            img.undistortedFileName = undistFileName;

            CalibrationSetupEntry entry = { j, job.observations[j].size() };

            job.observations[j].push_back(img);
            job.calibrationSetups[i].push_back(entry);
        }
    }

    JSONSetter setter(filename.c_str());
    setter.visit(job, "job");
    return 0;
}
