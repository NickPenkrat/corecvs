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

void addImageToJob(CalibrationJob* job, int camN, const char* str){
   std::stringstream fs;
   fs << dirGDrive << dirRelPath << str;
   std::string sourceFileName = fs.str();

   ImageData img;
   img.sourceFileName = sourceFileName;

   vector<CalibrationSetupEntry> calibrationSetupEntryVector;
   CalibrationSetupEntry entry = { camN, (int)job->observations[camN].size() };
   calibrationSetupEntryVector.push_back( entry );
   job->observations[camN].push_back(img);
   job->calibrationSetups.push_back(calibrationSetupEntryVector);
}

void fillJob(CalibrationJob* job){

    CheckerboardDetectionParameters params;
    params.setCellSizeHor (26);
    params.setCellSizeVert(18);
    BoardAlignerParams paramsA = BoardAlignerParams::GetNewBoard();

    job->settings.boardAlignerParams = paramsA;
    job->settings.openCvDetectorParameters = params;

    job->observations.resize(6);

    addImageToJob(job, 2, "SPA2_225deg.jpg");
    addImageToJob(job, 2, "SPA2_240deg.jpg");
    addImageToJob(job, 2, "SPA2_255deg.jpg");
    addImageToJob(job, 2, "SPA2_270deg.jpg");
    addImageToJob(job, 3, "SPA3_150deg.jpg");
    addImageToJob(job, 3, "SPA3_165deg.jpg");
    addImageToJob(job, 3, "SPA3_180deg.jpg");
    addImageToJob(job, 3, "SPA3_195deg.jpg");
    addImageToJob(job, 3, "SPA3_210deg.jpg");
    addImageToJob(job, 4, "SPA4_90deg.jpg");
    addImageToJob(job, 4, "SPA4_105deg.jpg");
    addImageToJob(job, 4, "SPA4_120deg.jpg");
    addImageToJob(job, 4, "SPA4_135deg.jpg");
    addImageToJob(job, 4, "SPA4_150deg.jpg");
    addImageToJob(job, 5, "SPA5_30deg.jpg");
    addImageToJob(job, 5, "SPA5_45deg.jpg");
    addImageToJob(job, 5, "SPA5_60deg.jpg");
    addImageToJob(job, 5, "SPA5_75deg.jpg");
    addImageToJob(job, 5, "SPA5_90deg.jpg");
    addImageToJob(job, 0, "SPA0_15deg.jpg");
    addImageToJob(job, 0, "SPA0_30deg.jpg");
    addImageToJob(job, 0, "SPA0_330deg.jpg");
    addImageToJob(job, 0, "SPA0_345deg.jpg");
    addImageToJob(job, 0, "SPA0_360deg.jpg");
    addImageToJob(job, 1, "SPA1_270deg.jpg");
    addImageToJob(job, 1, "SPA1_285deg.jpg");
    addImageToJob(job, 1, "SPA1_300deg.jpg");
    addImageToJob(job, 1, "SPA1_315deg.jpg");
    addImageToJob(job, 1, "SPA1_330deg.jpg");
    addImageToJob(job, 2, "SPA2_180deg.jpg");
    addImageToJob(job, 2, "SPA2_195deg.jpg");
    addImageToJob(job, 2, "SPA2_210deg.jpg");
}

TEST(Calibration, testCalculate)
{
    bool undistorted = false;

    CalibrationJob job;

    fillJob(&job);

    job.allDetectChessBoard(!undistorted);

//    CORE_ASSERT_TRUE(job.observations[1][0].sourcePattern.size() == 260, "Image SPA2_0deg.jpg didn't detect all points");
//    CORE_ASSERT_TRUE(job.observations[0][1].sourcePattern.size() == 468, "Image SPA3_0deg_3.jpg didn't detect all points");
//    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern.size() == 160, "Image SPA3_0deg_4.jpg didn't detect all points");

//    CORE_ASSERT_TRUE(job.observations[0][1].sourcePattern[0].x() == 0         , "Point 0 has wrong position");
//    CORE_ASSERT_TRUE(job.observations[0][1].sourcePattern[0].y() == 306       , "Point 0 has wrong position");
//    CORE_ASSERT_TRUE(job.observations[0][1].sourcePattern[0].z() == 0         , "Point 0 has wrong position");

//    CORE_ASSERT_DOUBLE_EQUAL(job.observations[0][1].sourcePattern[0].u(), 270.40416232494499    , "Point 0 has wrong position");
//    CORE_ASSERT_DOUBLE_EQUAL(job.observations[0][1].sourcePattern[0].v(), 1238.1164270140528   , "Point 0 has wrong position");

    job.allEstimateDistortion();

//    job.observations[0][0].undistortedFileName = "SPA3_0deg_4_test_undist_temp.jpg";
//    job.observations[1][0].undistortedFileName = "SPA2_0deg_test_undist_temp.jpg";
//    job.observations[0][1].undistortedFileName = "SPA3_0deg_3_test_undist_temp.jpg";

    job.allRemoveDistortion();

    job.allDetectChessBoard(undistorted);


    JSONSetter setterU("UnDistDetect.json");
    setterU.visit(job, "job");

//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern.size() == 260, "Image SPA2_0deg.jpg didn't detect all points");
//    CORE_ASSERT_TRUE(job.observations[0][2].undistortedPattern.size() == 468, "Image SPA3_0deg_3.jpg didn't detect all points");
//    CORE_ASSERT_TRUE(job.observations[0][0].undistortedPattern.size() == 160, "Image SPA3_0deg_4.jpg didn't detect all points");

//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern[0].x() == 0         , "Point 0 has wrong position");
//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern[0].y() == 306       , "Point 0 has wrong position");
//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern[0].z() == 0         , "Point 0 has wrong position");

//    CORE_ASSERT_DOUBLE_EQUAL(job.observations[0][1].undistortedPattern[0].u(), 270.40416232494499    , "Point 0 has wrong position");
//    CORE_ASSERT_DOUBLE_EQUAL(job.observations[0][1].undistortedPattern[0].v(), 1238.1164270140528   , "Point 0 has wrong position");

    job.calibrate();

    CameraModel c;
    c.extrinsics.position[0];

//    job.photostation.cameras[0].ex

//    CORE_ASSERT_TRUE(job.observations[1][0].undistortedPattern.size() == 260, "Image SPA2_0deg.jpg didn't detect all points");
//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern.size() == 468, "Image SPA3_0deg_3.jpg didn't detect all points");
//    CORE_ASSERT_TRUE(job.observations[0][0].undistortedPattern.size() == 160, "Image SPA3_0deg_4.jpg didn't detect all points");

//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern[0].x() == 0         , "Point 0 has wrong position");
//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern[0].y() == 306       , "Point 0 has wrong position");
//    CORE_ASSERT_TRUE(job.observations[0][1].undistortedPattern[0].z() == 0         , "Point 0 has wrong position");

//    CORE_ASSERT_DOUBLE_EQUAL(job.observations[0][1].undistortedPattern[0].u(), 270.40416232494499    , "Point 0 has wrong position");
//    CORE_ASSERT_DOUBLE_EQUAL(job.observations[0][1].undistortedPattern[0].v(), 1238.1164270140528   , "Point 0 has wrong position");

    JSONSetter setterC("calc.json");
    setterC.visit(job, "job");
}




