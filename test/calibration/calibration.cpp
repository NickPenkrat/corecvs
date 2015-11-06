/*
 * This test updates calibrationJob with detected patterns
 */
#include <vector>
#include <string>
#include <sstream>
#include "gtest/gtest.h"

#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

const char* dirGDrive = std::getenv("TOPCON_DIR_GDRIVE");
const char* dirRelPath = "/data/tests/calibration/";

const QString addPath(const char* name)
{
    std::stringstream fs;
    fs << dirGDrive << dirRelPath << name;
    QString sourceFileName = QString(fs.str().c_str());
    return sourceFileName;
}

void addImageToJob(CalibrationJob* job, int camN, int imageN, const char* name)
{
    std::stringstream fs;
    fs << dirGDrive << dirRelPath << name;
    std::string sourceFileName = fs.str();

    job->observations[camN][imageN].sourceFileName = sourceFileName;
}

void addUndistImageToJob(CalibrationJob* job, int camN, int imageN, const char* name)
{
    std::stringstream fs;
    fs << dirGDrive << dirRelPath << name;
    std::string sourceFileName = fs.str();

    job->observations[camN][imageN].undistortedFileName = sourceFileName;
}

void fillJob(CalibrationJob* job)
{
    addImageToJob(job, 0, 4, "SPA0_15deg.jpg" );
    addImageToJob(job, 0, 3, "SPA0_30deg.jpg" );
    addImageToJob(job, 0, 2, "SPA0_330deg.jpg");
    addImageToJob(job, 0, 1, "SPA0_345deg.jpg");
    addImageToJob(job, 0, 0, "SPA0_360deg.jpg");

    addUndistImageToJob(job, 0, 4,  "SPA0_15deg_undist.jpg");
    addUndistImageToJob(job, 0, 3,  "SPA0_30deg_undist.jpg");
    addUndistImageToJob(job, 0, 2, "SPA0_330deg_undist.jpg");
    addUndistImageToJob(job, 0, 1, "SPA0_345deg_undist.jpg");
    addUndistImageToJob(job, 0, 0, "SPA0_360deg_undist.jpg");

    addImageToJob(job, 1, 4, "SPA1_270deg.jpg");
    addImageToJob(job, 1, 3, "SPA1_285deg.jpg");
    addImageToJob(job, 1, 2, "SPA1_300deg.jpg");
    addImageToJob(job, 1, 1, "SPA1_315deg.jpg");
    addImageToJob(job, 1, 0, "SPA1_330deg.jpg");

    addUndistImageToJob(job, 1, 4, "SPA1_270deg_undist.jpg");
    addUndistImageToJob(job, 1, 3, "SPA1_285deg_undist.jpg");
    addUndistImageToJob(job, 1, 2, "SPA1_300deg_undist.jpg");
    addUndistImageToJob(job, 1, 1, "SPA1_315deg_undist.jpg");
    addUndistImageToJob(job, 1, 0, "SPA1_330deg_undist.jpg");

    addImageToJob(job, 2, 4, "SPA2_225deg.jpg");
    addImageToJob(job, 2, 3, "SPA2_240deg.jpg");
    addImageToJob(job, 2, 2, "SPA2_255deg.jpg");
    addImageToJob(job, 2, 1, "SPA2_270deg.jpg");
    addImageToJob(job, 2, 0, "SPA2_210deg.jpg");

    addUndistImageToJob(job, 2, 4, "SPA2_225deg_undist.jpg");
    addUndistImageToJob(job, 2, 3, "SPA2_240deg_undist.jpg");
    addUndistImageToJob(job, 2, 2, "SPA2_255deg_undist.jpg");
    addUndistImageToJob(job, 2, 1, "SPA2_270deg_undist.jpg");
    addUndistImageToJob(job, 2, 0, "SPA2_210deg_undist.jpg");

    addImageToJob(job, 3, 4, "SPA3_150deg.jpg");
    addImageToJob(job, 3, 3, "SPA3_165deg.jpg");
    addImageToJob(job, 3, 2, "SPA3_180deg.jpg");
    addImageToJob(job, 3, 1, "SPA3_195deg.jpg");
    addImageToJob(job, 3, 0, "SPA3_210deg.jpg");

    addUndistImageToJob(job, 3, 4, "SPA3_150deg_undist.jpg");
    addUndistImageToJob(job, 3, 3, "SPA3_165deg_undist.jpg");
    addUndistImageToJob(job, 3, 2, "SPA3_180deg_undist.jpg");
    addUndistImageToJob(job, 3, 1, "SPA3_195deg_undist.jpg");
    addUndistImageToJob(job, 3, 0, "SPA3_210deg_undist.jpg");

    addImageToJob(job, 4, 4, "SPA4_90deg.jpg" );
    addImageToJob(job, 4, 3, "SPA4_105deg.jpg");
    addImageToJob(job, 4, 2, "SPA4_120deg.jpg");
    addImageToJob(job, 4, 1, "SPA4_135deg.jpg");
    addImageToJob(job, 4, 0, "SPA4_150deg.jpg");

    addUndistImageToJob(job, 4, 4,  "SPA4_90deg_undist.jpg");
    addUndistImageToJob(job, 4, 3, "SPA4_105deg_undist.jpg");
    addUndistImageToJob(job, 4, 2, "SPA4_120deg_undist.jpg");
    addUndistImageToJob(job, 4, 1, "SPA4_135deg_undist.jpg");
    addUndistImageToJob(job, 4, 0, "SPA4_150deg_undist.jpg");

    addImageToJob(job, 5, 4, "SPA5_30deg.jpg");
    addImageToJob(job, 5, 3, "SPA5_45deg.jpg");
    addImageToJob(job, 5, 2, "SPA5_60deg.jpg");
    addImageToJob(job, 5, 1, "SPA5_75deg.jpg");
    addImageToJob(job, 5, 0, "SPA5_90deg.jpg");

    addUndistImageToJob(job, 5, 4, "SPA5_30deg_undist.jpg");
    addUndistImageToJob(job, 5, 3, "SPA5_45deg_undist.jpg");
    addUndistImageToJob(job, 5, 2, "SPA5_60deg_undist.jpg");
    addUndistImageToJob(job, 5, 1, "SPA5_75deg_undist.jpg");
    addUndistImageToJob(job, 5, 0, "SPA5_90deg_undist.jpg");
}

int compareFile(FILE* f1, FILE* f2)
{
    const int N = 100000;
    char buf1[N];
    char buf2[N];

    do {
        size_t r1 = fread(buf1, 1, N, f1);
        size_t r2 = fread(buf2, 1, N, f2);

        if (r1 != r2 || memcmp(buf1, buf2, r1))
            return 0;

    } while (!feof(f1) || !feof(f2));

    return 1;
}

TEST(Calibration, testDetectDistChessBoard)
{
    std::cout << dirGDrive << dirRelPath << "Read json gIn.json" << " mark \n";
    CalibrationJob job;
    bool undistorted = false;

    std::cout << "Read json " << addPath("gIn.json").toStdString().c_str() << " mark \n";

    JSONGetter getter(addPath("gIn.json"));
    getter.visit(job, "job");

    fillJob(&job);

    job.allDetectChessBoard(!undistorted);

    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern.size() == 198                        , "Image SPA0_360deg.jpg didn't detect all points");
    CORE_ASSERT_TRUE(job.observations[1][3].sourcePattern.size() == 162                        , "Image SPA1_285deg.jpg didn't detect all points");
    CORE_ASSERT_TRUE(job.observations[2][2].sourcePattern.size() == 162                        , "Image SPA2_255deg.jpg didn't detect all points");

    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].x() == 0                          , "Point 0 has wrong position X");
    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].y() == 0                          , "Point 0 has wrong position Y");
    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].z() == 0                          , "Point 0 has wrong position Z");

    printf("job.observations[0][1].sourcePattern[0].v(), 482.68523701220602 but in var : %.15e", job.observations[0][1].sourcePattern[0].v());

    CORE_ASSERT_DOUBLE_EQUAL_EP(job.observations[0][1].sourcePattern[0].v(), 482.68523701220602   , 1e-3, ( "Point 0 has wrong position V"));
}

TEST(Calibration, testEstimateDistDistortion)
{
    CalibrationJob job;

    JSONGetter getter(addPath("dOutDist.json"));
    getter.visit(job, "job");

    fillJob(&job);
    job.allEstimateDistortion();

    CORE_ASSERT_DOUBLE_EQUAL_EP(job.photostation.cameras[5].distortion.koeff()[1], 1.46316     , 1e-3, ("Camera 5 has wrong distortion koeff 2"));
    printf("job.photostation.cameras[5].distortion.koeff()[1], 1.46316 but in var : %.15e", job.photostation.cameras[5].distortion.koeff()[1]);
}

TEST(Calibration, testCalculate)
{
    CalibrationJob job;

    std::cout << "Read json esOutDist.json " << "mark \n";

    JSONGetter getter(addPath("esDistOutDist.json"));
    getter.visit(job, "job");

    fillJob(&job);
    job.calibrate();



    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.x(), 973.752   , 1e-3, ("Locations point position x error"));
    printf("job.calibrationSetupLocations[0].position.x(), 973.752 but in var : %.15e", job.calibrationSetupLocations[0].position.x());
    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.z(), -728.048  , 1e-3, ("Locations point position z error"));
    printf("(job.calibrationSetupLocations[0].position.z(), -728.048 but in var : %.15e", job.calibrationSetupLocations[0].position.z());
    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.y(), 250.021   , 1e-3, ("Locations point position y error"));
    printf("job.calibrationSetupLocations[0].position.y(), 250.021   , 1e-3 but in var : %.15e", job.calibrationSetupLocations[0].position.y());
}
