/*
 * This test updates calibrationJob with detected patterns
 */
#include <vector>
#include <string>
#include "gtest/gtest.h"

#include "calibrationJob.h"
#include "jsonGetter.h"
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
        string pathSrcDir = corecvs::HelperUtils::getFullPath("TOPCON_DIR", CALIBRATION_TEST_DIR_SRC, "");

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

                cout << "Src File: " << srcPath.toStdString() << endl;
                cout << "New file: " << dstPath.toStdString() << endl;

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
    job->observations[camN][imageN].sourceFileName = addPath(name).toStdString();
}

inline void addUndistImageToJob(CalibrationJob* job, int camN, int imageN, const char* name)
{
    job->observations[camN][imageN].undistortedFileName = addPath(name).toStdString();
}

void fillJob(CalibrationJob* job)
{
    addImageToJob(job, 0, 4, "SPA0_15deg.jpg");
    addImageToJob(job, 0, 3, "SPA0_30deg.jpg");
    addImageToJob(job, 0, 2, "SPA0_330deg.jpg");
    addImageToJob(job, 0, 1, "SPA0_345deg.jpg");
    addImageToJob(job, 0, 0, "SPA0_360deg.jpg");

    addUndistImageToJob(job, 0, 4, "SPA0_15deg_undist.jpg");
    addUndistImageToJob(job, 0, 3, "SPA0_30deg_undist.jpg");
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

    addImageToJob(job, 4, 4, "SPA4_90deg.jpg");
    addImageToJob(job, 4, 3, "SPA4_105deg.jpg");
    addImageToJob(job, 4, 2, "SPA4_120deg.jpg");
    addImageToJob(job, 4, 1, "SPA4_135deg.jpg");
    addImageToJob(job, 4, 0, "SPA4_150deg.jpg");

    addUndistImageToJob(job, 4, 4, "SPA4_90deg_undist.jpg");
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

//int compareFile(FILE* f1, FILE* f2)
//{
//    const int N = 100000;
//    char buf1[N];
//    char buf2[N];
//
//    do {
//        size_t r1 = fread(buf1, 1, N, f1);
//        size_t r2 = fread(buf2, 1, N, f2);
//
//        if (r1 != r2 || memcmp(buf1, buf2, r1))
//            return 0;
//
//    } while (!feof(f1) || !feof(f2));
//
//    return 1;
//}

TEST_F(CalibrationTest, testDetectDistChessBoard)
{
    CalibrationJob job;
    bool undistorted = false;

    JSONGetter getter(addPath("gIn.json"));
    getter.visit(job, "job");

    fillJob(&job);

    job.allDetectChessBoard(!undistorted);

    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern.size() == 198, "Image SPA0_360deg.jpg didn't detect all points");
    CORE_ASSERT_TRUE(job.observations[1][3].sourcePattern.size() == 162, "Image SPA1_285deg.jpg didn't detect all points");
    CORE_ASSERT_TRUE(job.observations[2][2].sourcePattern.size() == 162, "Image SPA2_255deg.jpg didn't detect all points");

    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].x() == 0, "Point 0 has wrong position X");
    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].y() == 0, "Point 0 has wrong position Y");
    CORE_ASSERT_TRUE(job.observations[0][0].sourcePattern[0].z() == 0, "Point 0 has wrong position Z");

    std::cout << job.observations[0][1].sourcePattern[0].v() << ", " << job.observations[0][1].sourcePattern[0].u() << std::endl;
    CORE_ASSERT_DOUBLE_EQUAL_EP(job.observations[0][1].sourcePattern[0].v(), 164.79241529168607, 1e-6, ("Point 0 has wrong position V"));
}

TEST_F(CalibrationTest, testEstimateDistDistortion)
{
    CalibrationJob job;

    JSONGetter getter(addPath("dOutDist.json"));
    getter.visit(job, "job");

    fillJob(&job);
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

    std::cout << "Read json esOutDist.json " << "mark " << endl;

    JSONGetter getter(addPath("esDistOutDist.json"));
    getter.visit(job, "job");

    fillJob(&job);
    job.calibrate();

    double calibrationRmse = -1.0;

    for (const std::vector<ImageData>& vim : job.observations)
    {
        for (const ImageData& im : vim)
        {
            cout << im.sourceFileName << "\tcalibration RMSE\t" << im.calibrationRmse << endl;
            if (im.calibrationRmse > calibrationRmse)
                calibrationRmse = im.calibrationRmse;
        }
    }

    CORE_ASSERT_TRUE(calibrationRmse < 2, "\n calibrationRmse more than 2 \n");

    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.x(), 993.125228460417   , 1e-12, ("Locations point position x error"));
    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.z(), -727.943404402887  , 1e-12, ("Locations point position z error"));
    //    CORE_ASSERT_DOUBLE_EQUAL_EP(job.calibrationSetupLocations[0].position.y(), 250.061591854775   , 1e-12, ("Locations point position y error"));
}
