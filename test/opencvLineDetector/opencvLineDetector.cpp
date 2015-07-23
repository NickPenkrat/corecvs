#include "global.h"

#include <QtCore/QString>
#include <QtCore/QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // imshow, waitkey

#include <string>

/// OpenCV wrapper
#include "OpenCVTools.h"

#include "bmpLoader.h"
#include "qtFileLoader.h"
#include "jsonGetter.h"
#include "jsonSetter.h"
#include "displacementBuffer.h"
#include "lmDistortionSolver.h"
#include "printerVisitor.h"
#include "openCvCheckerboardDetector.h"
#include "selectableGeometryFeatures.h"

using namespace cv;
using namespace corecvs;

static bool_t getIntCmdOption(const std::string & value, const std::string & option, int *param)
{
    cout << "Value" << value << std::endl;
    cout << "Option" << option << std::endl;

    size_t position = value.find(option);
    if (position != std::string::npos)
    {
        std::string strSub = value.substr (position + option.length());
        *param = atoi(strSub.c_str());
        return 1;
    }
    return 0;
}

static bool_t getDoubleCmdOption(const std::string & value, const std::string & option, double *param)
{
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        std::string strSub = value.substr (position + option.length());
        *param = std::stod(strSub.c_str());
        return 1;
    }
    return 0;
}

static bool_t getStringCmdOption(const std::string & value, const std::string & option, std::string *param)
{
    cout << "Value" << value << std::endl;
    cout << "Option" << option << std::endl;
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        *param = value.substr (position + option.length());
        return 1;
    }
    return 0;
}

static bool_t getStringCmdOption(const std::string & value, const std::string & option, QString *param)
{
    cout << "Value" << value << std::endl;
    cout << "Option" << option << std::endl;
    size_t  position = value.find(option);
    if (position != std::string::npos)
    {
        *param = QString::fromUtf8(value.substr (position + option.length()).c_str());
        return 1;
    }
    return 0;
}

static bool_t cmdIfOption(const vector<string> &all_args, const std::string& option, unsigned* pos)
{
    for (unsigned i = 0; i < all_args.size(); i++)
    {
        size_t found = all_args[i].find(option);
        if (found != string::npos) {
            *pos = i;
            return 1;
        }
    }
    return 0;
}

int main (int argc, char **argv)
{
    QTRGB24Loader::registerMyself();

    QCoreApplication app(argc, argv);

    vector<string> all_args;
    all_args.assign(argv, argv + argc);

    QString jsonFileName = "out.json";
    string prefix = "dist";
    unsigned pos;

    if (cmdIfOption(all_args, "--prefix", &pos))
    {
        getStringCmdOption(argv[pos], "--prefix:", &prefix);
        SYNC_PRINT(("prifix: %s\n", prefix.c_str()));
    }
    if (cmdIfOption(all_args, "--json_file_name", &pos))
    {
        getStringCmdOption(argv[pos], "--json_file_name:", &jsonFileName);
        SYNC_PRINT(("json_file_name: %s\n", jsonFileName.toLatin1().constData()));
    }

    bool verbose         = cmdIfOption(all_args, "--verbose",           &pos);
    bool useGreenChannel = cmdIfOption(all_args, "--use_green_channel", &pos);
    bool isInverse       =!cmdIfOption(all_args, "--is_direct",         &pos); // "inverse" works always except "direct" is requested
    bool drawProccess    = cmdIfOption(all_args, "--draw_proccess",     &pos);

    if (cmdIfOption(all_args, "--test_opencv", &pos))
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
    int chessW = 18;
    int chessH = 11;
    int found;
    int precise = 100;
    int maxIterationCount = 50;
    double minAccuracy = 0.001;

    if (argc > 1 && getStringCmdOption(argv[1], "--calcFullCheckerBoard:", &fileName))
    {
        if (cmdIfOption(all_args, "--chessW", &pos))
        {
            getIntCmdOption(argv[pos], "--chessW:", &chessW);
        }
        if (cmdIfOption(all_args, "--chessH", &pos))
        {
            getIntCmdOption(argv[pos], "--chessH:", &chessH);
        }
        if (cmdIfOption(all_args, "--max_iteration_count", &pos))
        {
            getIntCmdOption(argv[pos], "--max_iteration_count:", &maxIterationCount);
        }
        if (cmdIfOption(all_args, "--min_accuracy", &pos))
        {
            getDoubleCmdOption(argv[pos], "--min_accuracy:", &minAccuracy);
        }
        if (cmdIfOption(all_args, "--precise", &pos))
        {
            getIntCmdOption(argv[pos], "--precise:", &precise);
        }
        if (verbose)
        {
            SYNC_PRINT(("Calc full %s with %ix%i\n", fileName.c_str(), chessW, chessH));
        }

        IplImage  *iplImage;
        RGB24Buffer *image = BufferFactory::getInstance()->loadRGB24Bitmap(fileName);
        if (image == NULL)
        {
            SYNC_PRINT(("Failed to load <%s>.\n",fileName.c_str()));
            return 1;
        }

        Vector2dd center = Vector2dd(image->w, image->h) / 2.0;

        if (useGreenChannel) {
            G8Buffer *channel = image->getChannel(ImageChannel::G);
            iplImage = OpenCVTools::getCVImageFromG8Buffer(channel);
        }
        else {
            iplImage = OpenCVTools::getCVImageFromRGB24Buffer(image);
        }

        if (verbose)
        {
            SYNC_PRINT(("Loaded %s.\n", fileName.c_str()));
        }

        Mat view = cv::Mat(iplImage);

//        vector<vector<Vector2dd> > straights;
        SelectableGeometryFeatures lineList;

        found = OpenCvCheckerboardDetector::DetectFullCheckerboard(view, chessW, chessH, &lineList, precise);

        if (found)
        {
            if (verbose)
            {
                SYNC_PRINT(("Checkerboard found.\n"));
            }
            if (drawProccess)
            {
//                OpenCvCheckerboardDetector::DrawCheckerboardLines(view, straights);
                imwrite("test_with_chess_LINES.jpg", view);
                if (verbose)
                {
                    SYNC_PRINT(("test_with_chess_LINES.jpg saved.\n"));
                }
            }

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

                SYNC_PRINT(("Score is: %f\n", solver.costs[LineDistortionEstimatorCost::LINE_DEVIATION_COST]));
                SYNC_PRINT(("Written\n"));
            }
        }
    }
    else if (cmdIfOption(all_args, "--apply", &pos))
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

        for (int i = 1; i < all_args.size(); i++)
        {
            if (!getIntCmdOption(all_args[i], "--", &found))
            {
                fileName = all_args[i].c_str();
                if (verbose)
                {
                    SYNC_PRINT(("Apply to <%s>\n",fileName.c_str()));
                }

                RGB24Buffer *image = BufferFactory::getInstance()->loadRGB24Bitmap(fileName);
                if (image == NULL)
                {
                    SYNC_PRINT(("Failed to load <%s>.\n",fileName.c_str()));
                    return 1;
                }
                else if (verbose)
                {
                    SYNC_PRINT(("Loaded <%s>.\n",fileName.c_str()));
                }

                if (distortionCorrectTransform == NULL)
                {
                    if (!isInverse) {
                        distortionCorrectTransform = new DisplacementBuffer(&linesRadialCorrection
                            , image->h, image->w, true); // true - inverse  //FixMe: true/false doesn't work!!!
                    }
                    else {
                        distortionCorrectTransform = DisplacementBuffer::CacheInverse(&linesRadialCorrection
                            , image->h, image->w
                            , 0.0, 0.0
                            , (double)image->w, (double)image->h, 0.5);
                    }
                }

                image = image->doReverseDeformationBlTyped<DisplacementBuffer>(distortionCorrectTransform);
                if (image == NULL)
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
                QTFileLoader().save(outputFileName, image, 100);
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
            "opencvLineDetector.exe --apply --is_inverse <filenames_via_spaces>\n"
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
