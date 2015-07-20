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

#include "jsonGetter.h"
#include "jsonSetter.h"
#include "printerVisitor.h"

#include "openCvCheckerboardDetector.h"

using namespace cv;
using namespace corecvs;

bool verbose = 0;
bool drawProccess = 0;


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

static unsigned cmdIfOption(const vector<string> &all_args, const std::string& option, unsigned* pos)
{
    for (unsigned i=0;i<all_args.size();i++)
    {
        if(all_args[i].find(option)){
            *pos = i;
            return i;
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

    if (cmdIfOption(all_args, "--verbose", &pos))
    {
        verbose = 1;
    }

    if (cmdIfOption(all_args, "--draw_proccess", &pos))
    {
        drawProccess = 1;
    }

    if (cmdIfOption(all_args, "--test_opencv", &pos))
    {
        BMPLoader *loader   = new BMPLoader();
        G12Buffer *image    = loader->load("SPA0_360deg_1.bmp");
        IplImage  *iplImage = OpenCVTools::getCVImageFromG12Buffer(image);
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

    if (getStringCmdOption(argv[1], "--calcFullCheckerBoard:", &fileName) &&
        getIntCmdOption(argv[2], "--chessW:", &chessW) &&
        getIntCmdOption(argv[3], "--chessH:", &chessH) )
    {
        if(cmdIfOption(all_args, "--precise", &pos))
        {
             getIntCmdOption(argv[pos], "--precise:", &precise);
        }
        if(verbose)
        {
            SYNC_PRINT(("Calc full %s with %ix%i\n", fileName.c_str(), chessW, chessH));
        }

        BMPLoader *loader   = new BMPLoader();
        G12Buffer *image    = loader->load(fileName);
        if(verbose)
        {
            SYNC_PRINT(("Loaded %s.\n",fileName.c_str()));
        }

        IplImage  *iplImage = OpenCVTools::getCVImageFromG12Buffer(image);
        Mat            view = cv::Mat(iplImage, false);

        vector<vector<Vector2dd> > straights;

        found = OpenCvCheckerboardDetector::DetectFullCheckerboard(view, chessW, chessH, &straights, precise);

        if(found)
        {
            if(verbose)
            {
                SYNC_PRINT(("Checkerboard found.\n"));
            }
            if(drawProccess)
            {
                OpenCvCheckerboardDetector::DrawCheckerboardLines(view, straights);
                imwrite("test_with_chess_LINES.jpg", view);
                if(verbose)
                {
                    SYNC_PRINT(("test_with_chess_LINES.jpg saved.\n"));
                }
            }

            Vector2dd center(image->w / 2.0, image->h /2.0);

            /// Set default camera param values
            RadialCorrection correction(LensDistortionModelParameters(
               center.x(),
               center.y(),
               0.0, 0.0,
               vector<double>(6), //vector<double>(mUi->degreeSpinBox->value()), //TODO: Read degree from JSON
               1.0,
               1.0
            ));

            ModelToRadialCorrection modelFactory(
                  correction,
                  1, //mUi->estimateCenterCheckBox->isChecked(),
                  1, //mUi->estimateTangentCheckBox->isChecked(),
                  6 //mUi->degreeSpinBox->value()//TODO: Read degree from JSON
              );

            FunctionArgs *costFuntion = NULL;
            int isAngleCost = 0; //TODO: now use angle cost function in future read from JSON
            if (isAngleCost) {
                costFuntion = new AnglePointsFunction (straights, modelFactory);
                //straightF.simpleJacobian = mUi->simpleJacobianCheckBox->isChecked();
            } else {
                costFuntion = new DistPointsFunction  (straights, modelFactory);
            }

            int iterations = 1000; //mUi->iterationsSpinBox->value();
            LevenbergMarquardt straightLevMarq(iterations, 2, 1.5);
            straightLevMarq.f = costFuntion;

            /* First aproximation is zero vector */

            vector<double> first(costFuntion->inputs, 0);
            modelFactory.getModel(correction, &first[0]);

            vector<double> value(costFuntion->outputs, 0);
            vector<double> straightParams = straightLevMarq.fit(first, value);

            RadialCorrection mLinesRadialCoorection = modelFactory.getRadial(&straightParams[0]);
            {
                JSONSetter setter("out.json");
                setter.visit(mLinesRadialCoorection.mParams, "intrinsic");
            }
            if(verbose)
            {
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
        JSONGetter getter("out.json");
        getter.visit(loaded, "intrinsic");

        RadialCorrection mLinesRadialCoorection(loaded);
        if(verbose)
        {
            cout << mLinesRadialCoorection.mParams << endl;
        }

        BMPLoader *loader   = new BMPLoader();
        G12Buffer *image    = loader->load(filename);
        DisplacementBuffer* mDistortionCorrectTransform =
                 DisplacementBuffer::CacheInverse(&mLinesRadialCoorection,
                 image->h, image->w,
                 0.0,0.0,
                 (double)image->w, (double)image->h,
                 0.5, 0 //TODO: Use right value of mUi->preciseInvertionCheckBox->isChecked())
        );

        image = image->doReverseDeformationBl<G12Buffer, DisplacementBuffer>(
                    mDistortionCorrectTransform,
                    image->h, image->w
                );
        loader->save("dist_" + filename, image);

        for (int i = 3; i < argc; i++)
        {
            if(verbose)
            {
                SYNC_PRINT(("Apply %s\n",argv[i]));
            }

            image = loader->load(argv[i]);
            image = image->doReverseDeformationBl<G12Buffer, DisplacementBuffer>(
                        mDistortionCorrectTransform,
                        image->h, image->w
                    );

            string newFileName(argv[i]);
            loader->save("dist_" + newFileName, image);
        }

    }
    return 0;
}
