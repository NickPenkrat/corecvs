#include "core/patterndetection/chessBoardDetector.h"
#include "core/patterndetection/chessBoardCornerDetector.h"
#include "core/patterndetection/chessBoardAssembler.h"
#include "core/buffers/focusEstimator1.h"

#include <vector>
#include <cassert>
#include <iostream>
#include <fstream>
#include <memory>

#include "core/reflection/commandLineSetter.h"

/*#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
*/
#ifdef WITH_OPENCV
#include "openCvFileReader.h"
#endif
#ifdef WITH_LIBJPEG
#include "libjpegFileReader.h"
#endif
#ifdef WITH_LIBPNG
#include "libpngFileReader.h"
#endif

using namespace corecvs;

void usage()
{
    std::cout << "test_chessboard_detector image [W H] [true/false - exec imagequality]\n"
        "  * image - input image\n"
        "  * W x H - expected full size of pattern\n"
        "  * true/false - exec imagequality analisys\n"<< std::endl;

}

bool checkIfExists(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);
    return (bool)ifs;
}


bool parseArgs(int argc, char **argv, std::string &filename, int &W, int &H, bool &imageQuality)
{
    //if (argc < 2 || argc > 2 && argc < 4 || argc > 4)
    if (!(argc == 2 || argc == 4 || argc == 5))
    {
        return false;
    }
    filename = std::string(argv[1]);
    if (!checkIfExists(argv[1]))
    {
        std::cout << "File " << filename << " not found!" << std::endl;
        return false;
    }
    if (argc > 2)
    {
        W = std::stoi(argv[2]);
        H = std::stoi(argv[3]);
    }
    if (argc > 3)
    {
        imageQuality = (std::string(argv[4]) == "true");
    }


    return true;
}

#if 0
void readImage(const std::string &filename, DpImage &img)
{
    cv::Mat im = cv::imread(filename);
    im.convertTo(im, CV_64FC1, 1.0 / 255.0);
    // XXX:  OPENCV DOES NOT SUPPORTS FP64 RGB->GRAY
    // TODO: BTW, I hope that it is always BGR order, is it correct?!
    img = DpImage(im.rows, im.cols);
    for (int i = 0; i < im.rows; ++i)
    {
        for (int j = 0; j < im.cols; ++j)
        {
                (img.element(i, j) = 0.299 * im.at<double>(i, j * 3 + 2) + 0.587 * im.at<double>(i, j * 3 + 1) + 0.114 * im.at<double>(i, j * 3))
                ;
        }
    }
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
#endif

#define TRACE

int main(int argc, char **argv)
{

#ifdef WITH_OPENCV
    init_opencv_reader_provider();
#endif
#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif

    bool testImageQuality = false;
    int W, H;
    std::string filename;
    if (!parseArgs(argc, argv, filename, W, H, testImageQuality))
    {
        usage();
        return 0;
    }
#if 0
    DpImage img;
#else
    std::unique_ptr<corecvs::RGB24Buffer> img;
#endif

    img.reset(BufferFactory::getInstance()->loadRGB24Bitmap(filename));

    CheckerboardDetectionParameters params;
    BoardAlignerParams alignerParams;
    alignerParams.idealWidth = W;
    alignerParams.idealHeight = H;
    ChessBoardCornerDetectorParams cbparams;
    ChessBoardAssemblerParams cbap;
    cbap.setHypothesisDimFirst(W);
    cbap.setHypothesisDimSecond(H);

    ChessboardDetector detector(params, alignerParams, cbparams, cbap);

#ifdef TRACE
    cout << "We are using following configs" << endl;

    PrinterVisitor printer(2,2);
    cout << "CheckerboardDetectionParameters:"  << endl;
    params.accept(printer);
    cout << "BoardAlignerParams:"  << endl;
    alignerParams.accept(printer);
    cout << "ChessBoardAssemblerParams:"  << endl;
    cbap.accept(printer);
    cout << "ChessBoardCornerDetectorParams:"  << endl;
    cbparams.accept(printer);
    cout << std::flush;

    Statistics stats;
    detector.setStatistics(&stats);
#endif

    bool result = detector.detectPattern(*img);

    corecvs::ObservationList observations;
    detector.getPointData(observations);

#ifdef TRACE
    BaseTimeStatisticsCollector collector;
    collector.addStatistics(stats);
    collector.printAdvanced();
    std::cout << "result = " << result << ";" << endl;
#endif

    std::cout << "board = [";
    for (auto o: observations)
    {
        std::cout << o.projection[0] << " " << o.projection[1] << ";";
    }
    std::cout << "];" << std::endl;

    if (result && testImageQuality) {
        std::cout << "We will run Image Quality test";
        std::vector<BoardCornersType> boards;
        boards.push_back(detector.getBestBoard());

        FocusEstimator1 focusEstimator;
        focusEstimator.setBoardInfo(boards);
        focusEstimator.setInputImage(img.get());
        focusEstimator.operator ()();
        focusEstimator.getResult().print();
    }


    return 0;
}
