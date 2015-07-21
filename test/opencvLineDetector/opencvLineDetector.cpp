#include "cameraControlParameters.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "global.h"
#include <stdio.h>
#include <QtCore/QObject>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#include <QtCore/QMutex>
#include "QTimer"
#ifdef WIN32
# include <windows.h>
# define tSleep  Sleep
#else
# include <unistd.h>
# include <stdlib.h>
# define tSleep _sleep
#endif
#include "imageCaptureInterface.h"
#include "bmpLoader.h"
#include "qtFileLoader.h"

/// OpenCV wrapper
#include "OpenCVTools.h"

#include "camerasCalibration/camerasCalibrationFunc.h"
#include "curvatureFunc.h"
#include "radialFunc.h"
#include "angleFunction.h"
#include "anglePointsFunction.h"
#include "distPointsFunction.h"
#include "levenmarq.h"
#include "distortionCorrectTransform.h"
#include "distortionParameters.h"
#include "displacementBuffer.h"

#include "lmDistortionSolver.h"

#include "jsonGetter.h"
#include "jsonSetter.h"
#include "printerVisitor.h"

#include "openCvCheckerboardDetector.h"
#include "selectableGeometryFeatures.h"

using namespace cv;
using namespace corecvs;

bool verbose = 0;
bool drawProccess = 0;
bool useGreenChannel = 0;


static bool getIntCmdOption(const std::string & value, const std::string & option, int *param)
{
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        std::string strSub = value.substr (position + option.length());
        *param = atoi(strSub.c_str());
        return 1;
    }
    return 0;
}

static bool getDoubleCmdOption(const std::string & value, const std::string & option, double *param)
{
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        std::string strSub = value.substr (position + option.length());
        *param = stod(strSub.c_str());
        return 1;
    }
    return 0;
}

static bool getStringCmdOption(const std::string & value, const std::string & option, std::string *param)
{
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        *param = value.substr (position + option.length());
        return 1;
    }
    return 0;
}

static bool getStringCmdOption(const std::string & value, const std::string & option, QString *param)
{
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        *param = QString::fromUtf8(value.substr (position + option.length()).c_str());
        return 1;
    }
    return 0;
}

static bool cmdIfOption(const vector<string> &all_args, const std::string& option, unsigned* pos)
{
    for (unsigned i=0; i<all_args.size(); i++)
    {
        size_t found = all_args[i].find(option);
        if(found != string::npos){
            *pos = i;
            return 1;
        }
    }
    return 0;
}

int main (int argc, char **argv)
{
    QCoreApplication app(argc, argv);

    vector<string> all_args;
    all_args.assign(argv + 1, argv + argc);

    unsigned pos;

    QString jsonFileName = "out.json";
    string prefix = "dist";

    if(cmdIfOption(all_args, "--prefix", &pos))
    {
        getStringCmdOption(argv[pos], "--prefix:", &prefix);
        SYNC_PRINT(("prifix: %s\n", prefix.c_str()));
    }

    if(cmdIfOption(all_args, "--json_file_name", &pos))
    {
        getStringCmdOption(argv[pos], "--json_file_name:", &jsonFileName);
        SYNC_PRINT(("json_file_name: %s\n", jsonFileName.toLatin1().constData()));
    }

    if (cmdIfOption(all_args, "--verbose", &pos))
    {
        verbose = 1;
    }

    if (cmdIfOption(all_args, "--use_green_channel", &pos))
    {
        useGreenChannel = 1;
    }

    if (cmdIfOption(all_args, "--draw_proccess", &pos))
    {
        drawProccess = 1;
    }

    if (cmdIfOption(all_args, "--test_opencv", &pos))
    {
        BMPLoader *loader   = new BMPLoader();
        G8Buffer *image    = loader->loadRGB("SPA0_360deg_1.bmp")->getChannel(ImageChannel::G);
        IplImage  *iplImage = OpenCVTools::getCVImageFromG8Buffer(image);
                Mat            view = cv::Mat(iplImage, false);
                imshow("sf",view);
        int key = waitKey(1000000);
        return 0;
    }

    string fileName = "";
    int chessW = 18;
    int chessH = 11;
    int found;
    int precise = 11;
    int maxIterationCount = 100;
    double minAccuracy = 0.001;

    if (getStringCmdOption(argv[1], "--calcFullCheckerBoard:", &fileName) &&
        getIntCmdOption(argv[2], "--chessW:", &chessW) &&
        getIntCmdOption(argv[3], "--chessH:", &chessH) )
    {
        if(cmdIfOption(all_args, "--max_iteration_count", &pos))
        {
             getIntCmdOption(argv[pos], "--max_iteration_count:", &maxIterationCount);
        }
        if(cmdIfOption(all_args, "--min_accuracy", &pos))
        {
             getDoubleCmdOption(argv[pos], "--min_accuracy:", &minAccuracy);
        }
        if(cmdIfOption(all_args, "--precise", &pos))
        {
             getIntCmdOption(argv[pos], "--precise:", &precise);
        }
        if(verbose)
        {
            SYNC_PRINT(("Calc full %s with %ix%i\n", fileName.c_str(), chessW, chessH));
        }

        BMPLoader *loader   = new BMPLoader();
        IplImage  *iplImage;
        Vector2dd center;

        if(useGreenChannel){
            G8Buffer *image    = loader->loadRGB(fileName)->getChannel(ImageChannel::G);
            center = Vector2dd(image->w / 2.0, image->h /2.0);
            iplImage = OpenCVTools::getCVImageFromG8Buffer(image);
        }else{
            RGB24Buffer *image    = loader->loadRGB(fileName);
            center = Vector2dd(image->w / 2.0, image->h /2.0);
            iplImage = OpenCVTools::getCVImageFromRGB24Buffer(image);
        }

        if(verbose)
        {
            SYNC_PRINT(("Loaded %s.\n",fileName.c_str()));
        }

        Mat view = cv::Mat(iplImage);

//        vector<vector<Vector2dd> > straights;
        SelectableGeometryFeatures lineList;

        found = OpenCvCheckerboardDetector::DetectFullCheckerboard(view, chessW, chessH, &lineList, precise);

        if(found)
        {
            if(verbose)
            {
                SYNC_PRINT(("Checkerboard found.\n"));
            }
            if(drawProccess)
            {
//                OpenCvCheckerboardDetector::DrawCheckerboardLines(view, straights);
                imwrite("test_with_chess_LINES.jpg", view);
                if(verbose)
                {
                    SYNC_PRINT(("test_with_chess_LINES.jpg saved.\n"));
                }
            }

//            /// Set default camera param values
//            RadialCorrection correction(LensDistortionModelParameters(
//               center.x(),
//               center.y(),
//               0.0, 0.0,
//               vector<double>(6), //vector<double>(mUi->degreeSpinBox->value()), //TODO: Read degree from JSON
//               1.0,
//               1.0
//            ));


//            FunctionArgs *costFuntion = NULL;
//            int isAngleCost = 0; //TODO: now use angle cost function in future read from JSON
//            if (isAngleCost) {
//                costFuntion = new AnglePointsFunction (straights, modelFactory);
//                //straightF.simpleJacobian = mUi->simpleJacobianCheckBox->isChecked();
//            } else {
//                costFuntion = new DistPointsFunction  (straights, modelFactory);
//            }


            LMLinesDistortionSolver solver;
            LineDistortionEstimatorParameters params;

            params = LineDistortionEstimatorParameters();
            params.setEstimateCenter(true);
            params.setEstimateTangent(true);
            params.setEvenPowersOnly(false);
            params.setSimpleJacobian(true);
            params.setPolinomDegree(6);
            params.setCostAlgorithm(LineDistortionEstimatorCost::LINE_DEVIATION_COST);


            solver.initialCenter = center;
            solver.lineList = &lineList;
            solver.parameters = params;

            RadialCorrection mLinesRadialCoorection = solver.solve();

            {
                JSONSetter setter(jsonFileName);
                setter.visit(mLinesRadialCoorection.mParams, "intrinsic");
            }
            if(verbose)
            {
                solver.computeCosts(mLinesRadialCoorection, false);

                SYNC_PRINT(("Score is: %f\n", solver.costs[LineDistortionEstimatorCost::LINE_DEVIATION_COST]));
                SYNC_PRINT(("Written\n"));
            }
        }
    }
    else if (cmdIfOption(all_args, "--apply", &pos))
    {
        string filename = argv[2];

        if(verbose)
        {
            SYNC_PRINT(("Apply %s\n",filename.c_str()));
        }

        LensDistortionModelParameters loaded;
        JSONGetter getter(jsonFileName);
        getter.visit(loaded, "intrinsic");

        RadialCorrection mLinesRadialCoorection(loaded);
        if(verbose)
        {
            cout << mLinesRadialCoorection.mParams << endl;
        }

        BMPLoader  *loader   = new BMPLoader();
        RGB24Buffer *image   = loader->loadRGB(filename.c_str());

        if(verbose)
        {
            SYNC_PRINT(("Loaded %s.\n",filename.c_str()));
        }

        DisplacementBuffer* mDistortionCorrectTransform;

        if (false) {
            mDistortionCorrectTransform = new DisplacementBuffer(&mLinesRadialCoorection, image->h, image->w, true);
        } else {
            mDistortionCorrectTransform = DisplacementBuffer::CacheInverse(&mLinesRadialCoorection,
                image->h, image->w,
                0.0,0.0,
                (double)image->w, (double)image->h, 0.5
            );
        }

        image = image->doReverseDeformationBlTyped<DisplacementBuffer>(
                    mDistortionCorrectTransform);
        loader->save("dist_" + filename, image);

//        for (int i = 3; i < argc; i++)
//        {
//            if(!cmdIfOption(all_args, "--", &pos))
//            {
//                if(verbose)
//                {
//                    SYNC_PRINT(("Apply %s\n",argv[i]));
//                }

//                image = loader->loadRGB(argv[i]);
//                image = image-><RGB24Buffer, DisplacementBuffer>(
//                            mDistortionCorrectTransform,
//                            image->h, image->w);

//                string newFileName(argv[i]);
//                loader->save(prefix + "_" + newFileName, image);
//            }
//        }
    }
    return 0;
}
