/*
 * This test updates calibrationJob with detected patterns
 */
#include <vector>
#include <string>
#include "gtest/gtest.h"

#include "calibrationJob.h"
#include "jsonGetter.h"
#include "jsonSetter.h"
#include "utils.h"

#include <QFile>
#include <QDir>

#define CALIBRATION_TEST_DIR_SRC    "data/tests/calibration/"   // from TOPCON_DIR
#define CALIBRATION_TEST_DIR_DST         "tests/calibration/"   // to   TEMP_DIR

const QString wrkPath()
{
#ifdef WIN32
    const char* dirTEMP = std::getenv("TEMP");
#else
    const char* dirTEMP = "/tmp";
#endif
    QString path(dirTEMP);
    if (!QSTR_HAS_SLASH_AT_END(path)) {
        path += PATH_SEPARATOR;
    }
    path += CALIBRATION_TEST_DIR_DST;
    path = QDir::toNativeSeparators(path);
    return path;
}

class CalibrationTest : public ::testing::Test
{
protected:
    CalibrationTest() {}

    virtual ~CalibrationTest() {}

    virtual void SetUp()
    {
        string pathSrcDir = corecvs::HelperUtils::getFullPath("TOPCON_DIR", CALIBRATION_TEST_DIR_SRC);

        QDir source_dir(pathSrcDir.c_str());

        QString path = wrkPath();
        QDir dir(path);
        if (!dir.exists())
        {
            cout << "Creating " << path.toStdString() << "directory" << endl;
            dir.mkpath(path);
        }
        else
        {
            cout << path.toStdString() << " already exists" << endl;
        }

        foreach(QFileInfo item, source_dir.entryInfoList())
        {
            if (item.isFile())
            {
                QString srcPath = item.absoluteFilePath();
                QString dstPath = path + item.fileName();

                cout << "copy " << srcPath.toStdString() << " to " << path.toStdString() << endl;

                QFile toCopyFile(srcPath);
                toCopyFile.copy(dstPath);
            }
        }
    }

    virtual void TearDown() {}
};

inline const QString addPath(const char* name)
{
    return wrkPath() + name;
}

inline void addImageToJob(CalibrationJob* job, int camN, int imageN, const char* name)
{
    //string undist = HelperUtils::getFilePathWithSuffixAtName(name, "_undist");

    job->observations[camN][imageN].sourceFileName      = addPath(name).toStdString();
    job->observations[camN][imageN].undistortedFileName = ""; // addPath(undist.c_str()).toStdString();
}

void fillJobByTestDataM15(CalibrationJob* job)
{
    addImageToJob(job, 0, 4, "SPA0_15deg.jpg");
    addImageToJob(job, 0, 3, "SPA0_30deg.jpg");
    addImageToJob(job, 0, 2, "SPA0_330deg.jpg");
    addImageToJob(job, 0, 1, "SPA0_345deg.jpg");
    addImageToJob(job, 0, 0, "SPA0_360deg.jpg");

    addImageToJob(job, 1, 4, "SPA1_270deg.jpg");
    addImageToJob(job, 1, 3, "SPA1_285deg.jpg");
    addImageToJob(job, 1, 2, "SPA1_300deg.jpg");
    addImageToJob(job, 1, 1, "SPA1_315deg.jpg");
    addImageToJob(job, 1, 0, "SPA1_330deg.jpg");

    addImageToJob(job, 2, 4, "SPA2_210deg.jpg");
    addImageToJob(job, 2, 3, "SPA2_225deg.jpg");
    addImageToJob(job, 2, 2, "SPA2_240deg.jpg");
    addImageToJob(job, 2, 1, "SPA2_255deg.jpg");
    addImageToJob(job, 2, 0, "SPA2_270deg.jpg");

    addImageToJob(job, 3, 4, "SPA3_150deg.jpg");
    addImageToJob(job, 3, 3, "SPA3_165deg.jpg");
    addImageToJob(job, 3, 2, "SPA3_180deg.jpg");
    addImageToJob(job, 3, 1, "SPA3_195deg.jpg");
    addImageToJob(job, 3, 0, "SPA3_210deg.jpg");

    addImageToJob(job, 4, 4, "SPA4_90deg.jpg");
    addImageToJob(job, 4, 3, "SPA4_105deg.jpg");
    addImageToJob(job, 4, 2, "SPA4_120deg.jpg");
    addImageToJob(job, 4, 1, "SPA4_135deg.jpg");
    addImageToJob(job, 4, 0, "SPA4_150deg.jpg");

    addImageToJob(job, 5, 4, "SPA5_30deg.jpg");
    addImageToJob(job, 5, 3, "SPA5_45deg.jpg");
    addImageToJob(job, 5, 2, "SPA5_60deg.jpg");
    addImageToJob(job, 5, 1, "SPA5_75deg.jpg");
    addImageToJob(job, 5, 0, "SPA5_90deg.jpg");
}

TEST_F(CalibrationTest, testDetectDistChessBoard)
{
    CalibrationJob job;
    job.processState = new corecvs::StatusTracker;

    JSONGetter getter(addPath("gIn_updated.json"));
    getter.visit(job, "job");

    //{
    //    JSONSetter setter(addPath("gIn_updated_out.json"));
    //    setter.visit(job, "job");
    //}

    fillJobByTestDataM15(&job);

    job.allDetectChessBoard();

    cout << "sourcePattern.sizes = "
        << job.observations[0][0].sourcePattern.size() << " " 
        << job.observations[1][3].sourcePattern.size() << " "
        << job.observations[2][2].sourcePattern.size() << endl
        << "job.observations[0][1].sourcePattern[0] = "
        << job.observations[0][1].sourcePattern[0].v() << ","
        << job.observations[0][1].sourcePattern[0].u() << endl;

    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern.size() > 160, "Image SPA0_360deg.jpg didn't detect all points");
    CORE_ASSERT_TRUE(job.observations[1][3].sourcePattern.size() > 160, "Image SPA1_285deg.jpg didn't detect all points");
    CORE_ASSERT_TRUE(job.observations[2][2].sourcePattern.size() > 160, "Image SPA2_240deg.jpg didn't detect all points");

    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].x() == 0, "Point 0 has wrong position X");
    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].y() == 0, "Point 0 has wrong position Y");
    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].z() == 0, "Point 0 has wrong position Z");

    std::cout << job.observations[0][1].sourcePattern[0].v() << ", "
              << job.observations[0][1].sourcePattern[0].u() << std::endl;
    std::cout.flush();

    CORE_ASSERT_DOUBLE_EQUAL_EP(job.observations[0][1].sourcePattern[0].v(), 266.2, 1e-1, ("Point 0 has wrong position V"));

    delete_safe(job.processState);
}

TEST_F(CalibrationTest, testEstimateDistDistortion)
{
    CalibrationJob job;

    JSONGetter getter(addPath("dOutDist.json"));
    getter.visit(job, "job");

    fillJobByTestDataM15(&job);
    job.allEstimateDistortion();

    double distortionRmse = -1.0;

    for (const std::vector<ImageData>& vim : job.observations)
    {
        for (const ImageData& im : vim)
        {
            cout << im.sourceFileName << "\tdistortion RMSE\t" << im.distortionRmse << endl;
            if (im.distortionRmse > distortionRmse)
                distortionRmse = im.distortionRmse;
        }
    }

    cout << "Max distortion RMSE\t" << distortionRmse << endl;

    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.photostation.cameras[1].distortion.koeff()[1], 2.064727135565     , 1e-12, ("Camera 5 has wrong distortion koeff 2"));
    CORE_ASSERT_TRUE(distortionRmse < 1, "\n maxDistortionRmse more than 1 \n");
}

TEST_F(CalibrationTest, testCalculate)
{
    CalibrationJob job;

    std::cout << "Read esDistOutDist_updated.json " << "mark " << endl;

    JSONGetter getter(addPath("esDistOutDist_updated.json"));
    getter.visit(job, "job");

    fillJobByTestDataM15(&job);
    job.calibrate();

    //{
    //    JSONSetter setter(addPath("esDistOutDist_updated_out.json"));
    //    setter.visit(job, "job");
    //}

    double calibrationRmse = -1.0;

    for (const std::vector<ImageData>& vim : job.observations)
    {
        for (const ImageData& im : vim)
        {
            cout << im.sourceFileName << "\tcalibration RMSE\t" << im.calibrationRmse << endl;
            calibrationRmse = std::max(calibrationRmse, im.calibrationRmse);
        }
    }

    CORE_ASSERT_TRUE(calibrationRmse < 2, "\n calibrationRmse more than 2 \n");

    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.x(), 993.125228460417   , 1e-12, ("Locations point position x error"));
    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.z(), -727.943404402887  , 1e-12, ("Locations point position z error"));
    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.y(), 250.061591854775   , 1e-12, ("Locations point position y error"));
}
