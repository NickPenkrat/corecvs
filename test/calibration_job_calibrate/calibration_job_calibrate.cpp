/*
 * This test undistorts images from calibrationJob with estimated params
 */
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"
#include "abstractPainter.h"

#ifdef WITH_TBB
#include <tbb/tbb.h>
#endif

void draw(corecvs::ObservationList list, corecvs::RGB24Buffer &buffer)
{
    double minh = 1e10,  minv = 1e10, maxh = 0, maxv = 0;
    for (uint i = 0; i < list.size(); ++i)
    {
        auto el = list[i].point;
        if (el[0] < minv) minv = el[0];
        if (el[1] < minh) minh = el[1];
        if (el[0] > maxv) maxv = el[0];
        if (el[1] > maxh) maxh = el[1];
    }

    for (uint i = 0; i < list.size(); ++i)
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
    std::string filenameOut ="job.json";
    std::string filenameList= "images.txt";
    bool useNewPipeline = false;

    auto thread_limit_env = std::getenv("CALIBRATION_TEST_THREAD_LIMIT");

    if (thread_limit_env)
	{
		std::cout << "Detected thread limit env. var: " << thread_limit_env << std::endl;
	}
#ifdef WITH_TBB
	decltype(tbb::task_scheduler_init::automatic) tcnt = thread_limit_env ? std::stoi(thread_limit_env) : tbb::task_scheduler_init::automatic;
	tbb::task_scheduler_init init(tcnt);
#endif

    if (argc < 2)
    {
        std::cout << argv[0] << " new|old [images.txt [job.json]]"    << std::endl
                             << "\t new|old    -- pipeline type"      << std::endl
                             << "\t images.txt -- image list"         << std::endl
                             << "\t job.json   -- calibration output" << std::endl;
        return 0;
    }
    useNewPipeline = std::string(argv[1]) == "new";
    if (argc >= 3)
    {
        filenameList = std::string(argv[2]);
    }
    if (argc >= 4)
    {
        filenameOut = std::string(argv[3]);
    }

    std::cout << "Reading file list from "       << filenameList << std::endl <<
                 "Saving calibration result to " << filenameOut << std::endl;

    CalibrationJob job;

    std::ifstream files;
    files.open(filenameList);
    CORE_ASSERT_TRUE_S(files);

    std::vector<std::vector<std::array<std::string, 2>>> filenames;
    std::unordered_map<std::string, size_t> filenameMap;
    int camCnt = 0;

    while (files)
    {
        std::string filename;
        files >> filename;

        if (!filename.size())
            continue;
        size_t lsp = filename.rfind("SP");
        if (lsp == std::string::npos)
            continue;

        size_t ltg = filename.find_first_of("0123456789", lsp + 2);
        size_t ltne= filename.find_first_of("_.", lsp);
        std::string tag(filename.begin() + lsp + 2, filename.begin() + ltg);
        int cam = std::stoi(std::string(filename.begin() + ltg, filename.begin() + ltne));

        if (cam >= camCnt)
        {
            camCnt = cam + 1;
            for (auto& v: filenames)
                v.resize(camCnt);
        }
        if (!filenameMap.count(tag))
        {
            filenameMap[tag] = filenames.size();
            filenames.emplace_back(camCnt);
        }

        filenames[filenameMap[tag]][cam] = {filename, tag};
    }

    job.calibrationSetups.resize(filenameMap.size());
    job.observations.resize(camCnt);

    for (auto& setup: filenames)
    {
        size_t setup_ = &setup - &filenames[0];
        for (auto& fn: setup)
            if (fn[0].size())
            {
                int cam = &fn - &setup[0];
                CalibrationSetupEntry entry = { cam, (int)job.observations[cam].size() };
                job.calibrationSetups[setup_].emplace_back(entry);
                ImageData img;
                img.sourceFileName = fn[0];
                job.observations[cam].push_back(img);
            }
    }

    job.settings.boardAlignerParams = BoardAlignerParams::GetNewBoard();
    if (useNewPipeline)
    {
        job.settings.singleCameraCalibratorConstraints = job.settings.singleCameraCalibratorConstraints | CameraConstraints::UNLOCK_DISTORTION;
        job.settings.photostationCalibratorConstraints = job.settings.photostationCalibratorConstraints | CameraConstraints::UNLOCK_DISTORTION;
    }

    job.allDetectChessBoard();
    if (!useNewPipeline)
    {
        job.allEstimateDistortion();
        job.allDetectChessBoard(false);
    }
    job.calibrate();
    std::vector<int> topLayer;
    for (int i = 0; i < std::min(camCnt, 6); ++i)
        topLayer.push_back(i);
    job.reorient(topLayer);

    for (int i = 0; i < camCnt; ++i)
    {
        std::stringstream ss;
        ss << i;
        job.photostation.cameras[i].nameId = ss.str();
    }

    JSONSetter setter(filenameOut.c_str());
    setter.visit(job, "job");

    return 0;
}
