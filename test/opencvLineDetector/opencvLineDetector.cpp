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


using namespace cv;
using namespace corecvs;

int main (int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if (argc == 4)
    {
        char *filename = argv[1];
        int chessW = atoi(argv[2]);
        int chessH = atoi(argv[3]);

        BMPLoader *loader   = new BMPLoader();
        G12Buffer *image    = loader->load(filename);
        IplImage  *iplImage = OpenCVTools::getCVImageFromG12Buffer(image);

        int             found;
        vector<Point2f> pointbuf;
        Mat             view = cv::Mat(iplImage, false);
        Size            boardSize(chessW,chessH);

        found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );

        printf("Found in %s size %ix%i is %i\n", filename, chessW, chessH,  found);

        for (unsigned i = 0; i < pointbuf.size(); i++)
        {
            printf("Point %f %f\n", pointbuf[i].x,pointbuf[i].y);
        }
        drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
        imwrite("test_with_chess.jpg", view);

        Vector2dd center(image->w / 2.0, image->h /2.0);

        vector<vector<Vector2dd> > straights;

        for (int ih = 0; ih < chessH; ih++)
        {
            printf("Line %i ", ih);
            vector<Vector2dd> straight;
            double prevX = 0;
            double prevY = 0;
            for (int iw = 0; iw < chessW; iw++)
            {
                straight.push_back(Vector2dd(pointbuf.at(ih * chessW + iw).x,pointbuf.at(ih * chessW + iw).y));
                printf("  Point %i  %f %f", ih * chessW + iw, pointbuf[ih * chessW + iw].x,pointbuf[ih * chessW + iw].y);
                if (prevX != 0 && prevY != 0)
                {
                    cv::line(view, Point(prevX, prevY)
                        , Point(pointbuf[ih * chessW + iw].x
                              , pointbuf[ih * chessW + iw].y), Scalar(0,255,0));
                }
                prevX = pointbuf[ih * chessW + iw].x;
                prevY = pointbuf[ih * chessW + iw].y;
            }
            printf("\n");
            straights.push_back(straight);
        }
        imwrite("test_with_chess_LINES.jpg", view);

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
          SYNC_PRINT(("Written"));
    }

    if (argc == 2)
    {
       char *filename = argv[1];

       LensDistortionModelParameters loaded;
       JSONGetter getter("out.json");
       getter.visit(loaded, "intrinsic");

       PrinterVisitor printer;
       printer.visit(loaded, "intrinsic");

       RadialCorrection mLinesRadialCoorection(loaded);

       BMPLoader *loader   = new BMPLoader();
       G12Buffer *image    = loader->load(filename);

       DisplacementBuffer* mDistortionCorrectTransform =
                DisplacementBuffer::CacheInverse(&mLinesRadialCoorection,
                image->h, image->w,
                0.0,0.0,
                (double)image->w, (double)image->h,
                0.5, 1 //TODO: Use right value of mUi->preciseInvertionCheckBox->isChecked())
       );

#if 0
       printf("\n aspect %f \n", mLinesRadialCoorection.mParams.aspect);
       printf(" center %f x %f \n", mLinesRadialCoorection.mParams.center.x(), mLinesRadialCoorection.mParams.center.y());
       printf(" focal %f \n", mLinesRadialCoorection.mParams.focal);
       for (int k = 0; k < mLinesRadialCoorection.mParams.koeff.size(); k++) {
           printf(" koeff %f,", mLinesRadialCoorection.mParams.koeff.at(k));
       }
       printf("\n p1 %f \n", mLinesRadialCoorection.mParams.p1);
       printf(" p2 %f \n",   mLinesRadialCoorection.mParams.p2);
#endif
       cout << mLinesRadialCoorection.mParams << endl;

       image = image->doReverseDeformationBl<G12Buffer, DisplacementBuffer>(
                   mDistortionCorrectTransform,
                   image->h, image->w
               );

       string newFileName(filename);

       loader->save("dist_" + newFileName, image);

//    delete_safe(image);
//    delete_safe(loader);
    }
    return 0;
}
