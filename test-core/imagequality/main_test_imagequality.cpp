/**
 * \file main_test_imagequality.cpp
 * \brief This is the main file for the test image quality tools
 *
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "core/utils/global.h"


#include "core/fileformats/bmpLoader.h"
#include "core/patterndetection/chessBoardDetector.h"
#include "core/buffers/bufferFactory.h"
#include "core/patterndetection/patternDetector.h"
#include "core/buffers/rgb24/rgb24Buffer.h"
#include "core/xml/generated/focusEstimationParameters.h"
#include "core/xml/generated/focusEstimationResult.h"

#include "core/buffers/focusEstimator1.h"
#include "core/buffers/rgb24/abstractPainter.h"


using namespace corecvs;
using namespace std;



TEST(imagequality, DISABLED_testimagequality)
{
    cout << "Starting test <imagequality>" << endl;
    std::string filename = "data/calib-object.bmp";

    //filename = "/media/workarea/work/topcon-data/calib1/IMG_0080.JPG";
    filename = "data/test.bmp";

    RGB24Buffer *input = BufferFactory::getInstance()->loadRGB24Bitmap(filename);
    if (input == nullptr)
    {
        cout << "Could not open test image" << endl;
        return;
    }
    CORE_ASSERT_TRUE(input != NULL, "Can't load image");

    int H = 18;
    int W = 11;

    CheckerboardDetectionParameters checkParams;
    BoardAlignerParams alignerParams;
    alignerParams.idealWidth = W;
    alignerParams.idealHeight = H;
    ChessBoardCornerDetectorParams cbparams;
    ChessBoardAssemblerParams cbap;
    cbap.setHypothesisDimFirst(W);
    cbap.setHypothesisDimSecond(H);

    ChessboardDetector detector(checkParams, alignerParams, cbparams, cbap);

    Statistics stats;
    detector.setStatistics(&stats);

    std::vector<BoardCornersType> boards;
    DpImage grayscale(input->h, input->w);
    grayscale.binaryOperationInPlace(*input, [](const double & /*a*/, const corecvs::RGBColor &b) {
        return b.yd() / 255.0;
    });
    bool hasDetected = detector.detectPatternCandidates(grayscale, boards);
    cout << "hasDetected:" << hasDetected << endl;

    cout << "Boards " << boards.size() << endl;

    FocusEstimator1 estimator;
    estimator.mParams.setProduceDebug(true);
    estimator.setStatistics(&stats);
    estimator.setBoardInfo(boards);
    estimator.setInputImage(input);
    estimator.operator ()();
    BMPLoader().save("debug.bmp", estimator.getDebug());


    BaseTimeStatisticsCollector collector;
    collector.addStatistics(stats);
    collector.printAdvanced();

    cout << "Result" << endl;
    estimator.getResult().print();


    cout << "Test <imagequality> PASSED" << endl;
}
