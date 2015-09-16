#include "global.h"

#include <vector>
#include <cassert>
#include <iostream>
#include <fstream>

#include <QtCore/QString>
#include <QtCore/QCoreApplication>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // imshow, waitkey

#include <string>

/// OpenCV wrapper
#include "OpenCVTools.h"

#include "commandLineSetter.h"
#include "bmpLoader.h"
#include "qtFileLoader.h"
#include "jsonGetter.h"
#include "jsonSetter.h"
#include "displacementBuffer.h"
#include "lmDistortionSolver.h"
#include "printerVisitor.h"
#include "openCvCheckerboardDetector.h"
#include "checkerboardDetectionParameters.h"
#include "selectableGeometryFeatures.h"

#include "chessBoardDetector.h"
#include "chessBoardCornerDetector.h"
#include "chessBoardAssembler.h"

#include "calibrationStructs.h"

using namespace cv;
using namespace corecvs;


vector<string> fileList;

vector<string> splitString(const string &s, const char &delimiter)
{
    vector<string> res;
    string st;
    istringstream f(s);
    while (getline(f, st, delimiter))
    {
        res.push_back(st);
    }
    return res;
}

bool checkIfExists(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);

    return (bool)ifs;
}

void readImage(const std::string &filename, corecvs::RGB24Buffer &img)
{
    cv::Mat im = cv::imread(filename);
    im.convertTo(im, CV_64FC1, 1.0);
    img = corecvs::RGB24Buffer(im.rows, im.cols);
    for (int i = 0; i < im.rows; ++i)
    {
        for (int j = 0; j < im.cols; ++j)
        {
            img.element(i, j) = corecvs::RGBColor(im.at<double>(i, j * 3 + 2), im.at<double>(i, j * 3 + 1), im.at<double>(i, j * 3));
        }
    }
}

void readFileList(const string &filename)
{
    ifstream f(filename);
    string currentLine;
    while (!f.eof())
    {
        getline(f, currentLine);
        if (!currentLine.empty())
        {
            string imageFileName = currentLine.substr(0, currentLine.find(" "));
            fileList.push_back(imageFileName);
        }
    }
    f.close();
}

int main (int argc, char **argv)
{
    QTRGB24Loader::registerMyself();

    QCoreApplication app(argc, argv);

    CommandLineSetter setter(argc, (const char **)argv);

    bool verbose         = setter.getBool("verbose"          );

    string prefix = setter.getString("prefix", "dist");
    if (verbose)
    {    SYNC_PRINT(("prifix: %s\n", prefix.c_str()));
    }

    QString jsonFileName = QString(setter.getString("json_file_name", "out.json").c_str());
    if (verbose)
    {    SYNC_PRINT(("json_file_name: %s\n", jsonFileName.toLatin1().constData()));
    }


    bool useGreenChannel = setter.getBool("use_green_channel");
    bool isInverse       =!setter.getBool("is_direct"        ); // "inverse" works always except "direct" is requested
    bool drawProccess    = setter.getBool("draw_proccess"    );

    if (setter.getBool("test_opencv"))
    {
        BMPLoader *loader   = new BMPLoader();
        G8Buffer  *image    = loader->loadRGB("SPA0_360deg_1.bmp")->getChannel(ImageChannel::G);
        IplImage  *iplImage = OpenCVTools::getCVImageFromG8Buffer(image);
        Mat            view = cv::Mat(iplImage, false);
        imshow("sf", view);
        waitKey(1000000);
        return 0;
    }

    string fileName = "";

    int chessW      = setter.getInt("chessW",       18);
    int chessH      = setter.getInt("chessH",       11);
    double minAccuracy = setter.getDouble("min_accuracy", 0.001);
    double precise     = setter.getDouble("precise",      50);   // TODO: =100 doesn't work!!! Why???
    double cellSize    = setter.getDouble("cell_size",    50);


    if (setter.hasOption("calcMultiCheckerBoard")){
        corecvs::ObservationList observations;

        CheckerboardDetectionParameters params;
        BoardAlignerParams alignerParams = BoardAlignerParams::GetOldBoard();
#if 0
        params.setHorCrossesCount (chessW);
        params.setVertCrossesCount(chessH);
        params.setFitWidth(true);
        params.setFitHeight(false);
#else
        alignerParams.idealWidth = chessW;
        alignerParams.idealHeight = chessH;
        alignerParams.type = AlignmentType::FIT_WIDTH;
#endif


        if (setter.hasOption("files"))
        {
            string fileNames = setter.getString("files", "");
            fileList = splitString(fileNames, ',');
        }

        if (setter.hasOption("fileslist"))
        {
            string fileNames = setter.getString("fileslist", "");
            readFileList(fileNames);
        }

        int w = 0;
        int h = 0;

        for (string fileName: fileList)
        {
            if (!checkIfExists(fileName.c_str()))
            {
                std::cout << "File " << fileName << " not found!" << std::endl;
                continue;
            }
            corecvs::RGB24Buffer img;
            readImage(fileName.c_str() , img);

            if(!w){
                w = img.w;
                h = img.h;
            }

            ChessboardDetector detector(params, alignerParams);
            detector.detectPattern(img);

            corecvs::ObservationList localObservations;
            detector.getPointData(localObservations);
            for (auto o: observations)
            {
                observations.push_back(o);
            }
        }

        SelectableGeometryFeatures features;
        features.addAllLinesFromObservationList(observations);

        /// TODO: read from JSON
        LineDistortionEstimatorParameters parameters;

        parameters = LineDistortionEstimatorParameters();
        parameters.setSimpleJacobian(true);
        parameters.setEstimateCenter(true);
        parameters.setEstimateTangent(true);
        parameters.setEvenPowersOnly(false);
        parameters.setIterationNumber(1000);
        parameters.setCostAlgorithm(LineDistortionEstimatorCost::LINE_DEVIATION_COST);

        LMLinesDistortionSolver solver;
        solver.initialCenter = Vector2dd(w / 2.0, h /2.0);
        solver.lineList = &features;
        solver.parameters = parameters;

                RadialCorrection correction = solver.solve();

                Camera_ camera;
                camera.distortion = correction.mParams;
                JSONSetter setter(jsonFileName);
                setter.visit(camera, "intrinsic");
    }

    else if (setter.hasOption("calcFullCheckerBoard"))
    {
        fileName = setter.getString("calcFullCheckerBoard", "");

        int maxIterationCount = setter.getInt("max_iteration_count", 1000);

        if (verbose)
        {
            SYNC_PRINT(("Calc full %s with %ix%i\n", fileName.c_str(), chessW, chessH));
        }

        RGB24Buffer *image = BufferFactory::getInstance()->loadRGB24Bitmap(fileName);
        if (image == NULL)
        {
            SYNC_PRINT(("Failed to load <%s>.\n",fileName.c_str()));
            return 1;
        }

        Vector2dd center = Vector2dd(image->w, image->h) / 2.0;

        CheckerboardDetectionParameters params;
        BoardAlignerParams alignerParams = BoardAlignerParams::GetOldBoard();
//        params.setHorCrossesCount(chessW);
//        params.setVertCrossesCount(chessH);
        alignerParams.type = AlignmentType::FIT_ALL;
        params.setPreciseDiameter(precise);
        params.setMinAccuracy(minAccuracy);
        params.setIterationCount(maxIterationCount);

        G8Buffer *channel = image->getChannel(useGreenChannel ? ImageChannel::G : ImageChannel::GRAY);

        if (verbose)
        {
            SYNC_PRINT(("Loaded %s.\n", fileName.c_str()));
        }

        SelectableGeometryFeatures lineList;
//        G8Buffer *boardOutput = NULL;
//        found = OpenCvCheckerboardDetector::DetectFullCheckerboard(channel, params, &lineList, &boardOutput);
        OpenCvCheckerboardDetector detector(params, alignerParams);
        bool found = detector.detectPattern(*channel);
        detector.getPointData(lineList);


        if (found)
        {
            if (verbose)
            {
                SYNC_PRINT(("Checkerboard found.\n"));
            }
            // FIXME: Not Implemented Yet
#if 0
            if (drawProccess)
            {
//                OpenCvCheckerboardDetector::DrawCheckerboardLines(view, straights);
                //imwrite("test_with_chess_LINES.jpg", view);
                RGB24Buffer *toSave = new RGB24Buffer(boardOutput);
                QTFileLoader().save("test_with_chess_LINES.jpg", toSave, 100);
                delete_safe(toSave);
                if (verbose)
                {
                    SYNC_PRINT(("test_with_chess_LINES.jpg saved.\n"));
                }
            }
#endif

            LMLinesDistortionSolver solver;
            LineDistortionEstimatorParameters params;

            params = LineDistortionEstimatorParameters();
            params.setSimpleJacobian(true);
            params.setEstimateCenter(true);
            params.setEstimateTangent(true);
            params.setEvenPowersOnly(false);
            params.setIterationNumber(1000);
            params.setCostAlgorithm(LineDistortionEstimatorCost::LINE_DEVIATION_COST);

            lineList.print();

            solver.initialCenter = center;
            solver.lineList      = &lineList;
            solver.parameters    = params;

            RadialCorrection linesRadialCorrection = solver.solve();
            {
                JSONSetter setter(jsonFileName);
                setter.visit(linesRadialCorrection.mParams, "intrinsic");
            }

            if (verbose)
            {
                solver.computeCosts(linesRadialCorrection, false);

                SYNC_PRINT(("Score     is: %f\n", solver.costs[LineDistortionEstimatorCost::LINE_DEVIATION_COST].getRadiusAround0()));
                SYNC_PRINT(("Max error is: %f\n", solver.costs[LineDistortionEstimatorCost::LINE_DEVIATION_COST].getMax()));
            }
        }
    }
    else if (setter.hasOption("calcPartCheckerBoard"))
    {
        fileName = setter.getString("calcFullCheckerBoard", "");

        int maxIterationCount = setter.getInt("max_iteration_count", 1000);

        // FIXME: WTF? detect part of chessboard and do nothing?!
        RGB24Buffer *image   = BufferFactory::getInstance()->loadRGB24Bitmap(fileName);
        G8Buffer    *channel = image->getChannel(ImageChannel::GRAY);

        CheckerboardDetectionParameters params;
        params.setPartialBoard(true);
        params.setPreciseDiameter(precise);
        params.setMinAccuracy(minAccuracy);
        params.setIterationCount(maxIterationCount);
        params.setCellSizeHor(cellSize);
        params.setCellSizeVert(cellSize);

        BoardAlignerParams alignerParams = BoardAlignerParams::GetOldBoard();
        alignerParams.idealWidth = chessW;
        alignerParams.idealHeight = chessH;
        alignerParams.type = AlignmentType::FIT_WIDTH;

        ObservationList observationList;

        //OpenCvCheckerboardDetector::DetectPartCheckerboardV(channel, params, &observationList, NULL);
        OpenCvCheckerboardDetector detector(params, alignerParams);
        bool found = detector.detectPattern(*channel);
        detector.getPointData(observationList);

        delete_safe(channel);
        delete_safe(image);
    }
    else if (setter.hasOption("apply"))
    {
        LensDistortionModelParameters loaded;
        JSONGetter getter(jsonFileName);
        getter.visit(loaded, "intrinsic");

        RadialCorrection linesRadialCorrection(loaded);
        if (verbose)
        {
            cout << linesRadialCorrection.mParams << endl;
        }

        DisplacementBuffer* distortionCorrectTransform = NULL;

        QString fileNames = QString(setter.getString("calcMultiCheckerBoard", "").c_str());
        QStringList fileList = fileNames.split(",", QString::SkipEmptyParts);
        for (QString curFileName: fileList)
        {
            fileName = curFileName.toStdString();
            if (checkIfExists(fileName.c_str()))
            {
                if (verbose)
                {
                    SYNC_PRINT(("Apply to <%s>\n", fileName.c_str()));
                }

                RGB24Buffer *image = BufferFactory::getInstance()->loadRGB24Bitmap(fileName);
                if (image == NULL)
                {
                    SYNC_PRINT(("Failed to load <%s>.\n", fileName.c_str()));
                    return 1;
                }
                else if (verbose)
                {
                    SYNC_PRINT(("Loaded <%s>.\n", fileName.c_str()));
                }

                if (distortionCorrectTransform == NULL)
                {
                    if (!isInverse) {
                        if (verbose) { SYNC_PRINT(("Forward transform\n")); }
                        distortionCorrectTransform = new DisplacementBuffer(&linesRadialCorrection
                            , image->h, image->w, false); // false - !inverse
                    }
                    else {
                        if (verbose) { SYNC_PRINT(("Inverse transform\n")); }

                        distortionCorrectTransform = DisplacementBuffer::CacheInverse(&linesRadialCorrection
                            , image->h, image->w
                            , 0.0, 0.0
                            , (double)image->w, (double)image->h, 0.5);
                    }
                }

                RGB24Buffer *deformed = image->doReverseDeformationBlTyped<DisplacementBuffer>(distortionCorrectTransform);
                if (deformed == NULL)
                {
                    SYNC_PRINT(("Failed to do transformation.\n"));
                    return 2;
                }

                string path = "";
                string name = fileName;

                size_t pos = name.find_last_of("/\\");
                if (pos != string::npos)
                {
                    path = name.substr(0, pos + 1);
                    name = name.substr(pos + 1);
                }

                string outputFileName = path + prefix + name;

                SYNC_PRINT(("Saving to <%s>\n", outputFileName.c_str()));
                QTFileLoader().save(outputFileName, deformed, 100);

                delete_safe(deformed);
                delete_safe(image);
            }
        }
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
//                            distortionCorrectTransform,
//                            image->h, image->w);

//                string newFileName(argv[i]);
//                loader->save(prefix + "_" + newFileName, image);
//            }
//        }
    }
    else
    {
        printf("Usage examples:\n\n"
            "To detect distortion params:\n"
            "opencvLineDetector.exe --calcFullCheckerBoard:<filename> --chessW:18 --chessH:11\n"
            "                       --use_green_channel --max_iteration_count:50 --min_accuracy:0.001\n\n"
            "To apply found distortion params:\n"
            "opencvLineDetector.exe --apply <filenames_via_spaces>\n"
            );
        //--prefix
        //--json_file_name
        //--verbose
        //--is_inverse
        //--draw_proccess
        //--test_opencv
    }

    return 0;
}
