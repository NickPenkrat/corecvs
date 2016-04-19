#include "chessBoardDetector.h"

#include "abstractPainter.h"
#include "homographyReconstructor.h"
#include <fstream>

ChessboardDetector::ChessboardDetector (
        CheckerboardDetectionParameters params,
        BoardAlignerParams alignerParams,
        ChessBoardCornerDetectorParams detectorParams,
        ChessBoardAssemblerParams assemblerParams
    )
    : CheckerboardDetectionParameters(params),
      aligner(new BoardAligner(alignerParams)),     
      stats(NULL),
      detector(detectorParams)
{

    BoardAlignerParams activeAlignerParams = aligner->getAlignerParams();

    assemblerParams.setHypothesisDimensions(0);

    switch (aligner->type)
    {
        case AlignmentType::FIT_MARKER_ORIENTATION:
        case AlignmentType::FIT_MARKERS:
            break;
        case AlignmentType::FIT_WIDTH:
            assemblerParams.setHypothesisDimensions(1);
            assemblerParams.setHypothesisDimFirst(activeAlignerParams.idealWidth);
            break;
        case AlignmentType::FIT_HEIGHT:
            assemblerParams.setHypothesisDimensions(1);
            assemblerParams.setHypothesisDimFirst(activeAlignerParams.idealHeight);
            break;
        case AlignmentType::FIT_ALL:
            assemblerParams.setHypothesisDimensions(2);
            assemblerParams.setHypothesisDimFirst(activeAlignerParams.idealWidth);
            assemblerParams.setHypothesisDimSecond(activeAlignerParams.idealHeight);
            break;
    }
    assembler = ChessBoardAssembler(assemblerParams);
    sharedGenerator = std::shared_ptr<CirclePatternGenerator>(aligner->FillGenerator(activeAlignerParams));
}

ChessBoardDetectorMode ChessboardDetector::getMode(const BoardAlignerParams &params)
{
    ChessBoardDetectorMode mode = ChessBoardDetectorMode::BEST;
    switch (params.type)
    {
        case AlignmentType::FIT_MARKER_ORIENTATION:
        case AlignmentType::FIT_MARKERS:
            break;
        case AlignmentType::FIT_WIDTH:
            mode = ChessBoardDetectorMode::FIT_WIDTH;
            break;
        case AlignmentType::FIT_HEIGHT:
            mode = ChessBoardDetectorMode::FIT_HEIGHT;
            break;
        case AlignmentType::FIT_ALL:
            mode = ChessBoardDetectorMode::FIT_HEIGHT | ChessBoardDetectorMode::FIT_WIDTH;
            break;
    }
    return mode;
}

bool ChessboardDetector::detectPattern(corecvs::G8Buffer &buffer)
{
    DpImage grayscale(buffer.h, buffer.w);
    grayscale.binaryOperationInPlace(buffer, [](const double & /*a*/, const unsigned char &b) {
        return ((double)b) / 255.0;
    });
    return detectPattern(grayscale);
}

bool ChessboardDetector::detectPattern(corecvs::RGB24Buffer &buffer)
{
    DpImage grayscale(buffer.h, buffer.w);
    grayscale.binaryOperationInPlace(buffer, [](const double & /*a*/, const corecvs::RGBColor &b) {
        return b.yd() / 255.0;
    });
    return detectPattern(grayscale);
}

bool ChessboardDetector::detectPatternCandidates(DpImage &buffer, std::vector<std::vector<std::vector<corecvs::Vector2dd>>> &boards)
{
    if (stats != NULL) stats->startInterval();

    corners.clear();
    bestPattern = RectangularGridPattern();

    std::string prefix;
    if (stats != NULL) {
        prefix = stats->prefix;
        stats->prefix = "Corners -> " + stats->prefix;
        detector.setStatistics(stats);
    }

    detector.detectCorners(buffer, corners);

    if (stats != NULL) stats->prefix = prefix;
    if (stats != NULL) stats->resetInterval("Corners");

    if (stats != NULL) {
        prefix = stats->prefix;
        stats->prefix = "Assembler -> " + stats->prefix;
        assembler.setStatistics(stats);
    }

    BoardAlignerParams activeAlignerParams = aligner->getAlignerParams();
    sharedGenerator->flushCache();
    BoardAligner aligner(activeAlignerParams, sharedGenerator);
    assembler.assembleBoards(corners, boards, &aligner, &buffer);
    if (stats != NULL) stats->prefix = prefix;

    if (!boards.size())
        return false;

    if (stats != NULL) stats->resetInterval("Assemble");
    return true;
}

bool ChessboardDetector::detectPattern(DpImage &buffer)
{
    BoardAlignerParams activeAlignerParams = aligner->getAlignerParams();

    ChessBoardDetectorMode mode =  getMode(activeAlignerParams);
    corners.clear();
    bestPattern = RectangularGridPattern();

    if (stats != NULL) stats->startInterval();
    if (stats != NULL) detector.setStatistics(stats->enterContext("Corners->"));

    detector.detectCorners(buffer, corners);

    if (stats != NULL) stats->leaveContext();
    if (stats != NULL) stats->resetInterval("Corners");

    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> boards;

    if (stats != NULL) assembler.setStatistics(stats->enterContext("Assembler -> "));

    sharedGenerator->flushCache();
    BoardAligner activeAligner(activeAlignerParams, sharedGenerator);

    assembler.assembleBoards(corners, boards, &activeAligner, &buffer);

    if (boards.empty())
        return false;

    if (stats != NULL) stats->leaveContext();
    if (stats != NULL) stats->resetInterval("Assemble");

    bool /*transposed = false,*/ found = false;

    bool checkW = !!(mode & ChessBoardDetectorMode::FIT_WIDTH);
    bool checkH = !!(mode & ChessBoardDetectorMode::FIT_HEIGHT);

    for (auto& b: boards)
    {
        int bw = (int)b[0].size();
        int bh = (int)b.size();

        bool fitw = (bw == activeAlignerParams.idealWidth);
        bool fith = (bh == activeAlignerParams.idealHeight);


        if ((!checkW || fitw) && (!checkH || fith))
        {
            aligner->bestBoard = b;
            found = true;
            break;
        }
        fitw = (bh == activeAlignerParams.idealWidth);
        fith = (bw == activeAlignerParams.idealHeight);
        if ((!checkW || fitw) && (!checkH || fith))
        {
            aligner->bestBoard = b;
            found = true;
            //transposed = true;
            break;
        }
    }

    if (!found)
        return false;

    bool aligned = aligner->align(buffer);
    if (aligned)
    {
        std::cout << (aligned ? "ALIGN OK" : "ALIGN FAILED") << std::endl;
        result = aligner->observationList;
        for (auto& p: result)
        {
            p.point.x() *= cellSizeHor();
            p.point.y() *= cellSizeVert();
        }
        std::cout << result.size() << " features" << std::endl;
        return true;
    }
    return false;
}

void ChessboardDetector::getPointData(ObservationList &observations)
{
    observations = result;
}

void ChessboardDetector::drawClassifier(corecvs::RGB24Buffer &buffer)
{
    aligner->drawDebugInfo(buffer);
}

// FIXME: Temporary code; needs serious rework
bool ChessboardDetector::classify(DpImage &img, CirclePatternGenerator &gen, corecvs::RGB24Buffer &buffer)
{
    vector<vector<Vector2dd>> &bestBoard = aligner->bestBoard;

    int w = (int)bestBoard[0].size();
    int h = (int)bestBoard.size();
    std::vector<std::vector<int>> classifier(h - 1);
    for (auto& cv: classifier) cv.resize(w - 1);
    std::cout << w << " x " << h << std::endl;

    DpImage fin(buffer.h, buffer.w);

    corecvs::RGBColor colors[8] =
    {
        corecvs::RGBColor(0xffff00),
        corecvs::RGBColor(0x00ffff),
        corecvs::RGBColor(0xff00ff),
        corecvs::RGBColor(0x00ff00),
        corecvs::RGBColor(0x0000ff),
        corecvs::RGBColor(0xff0000),
        corecvs::RGBColor(0x7f0000),
        corecvs::RGBColor(0x007f00)
    };
    corecvs::RGBColor board_col = corecvs::RGBColor(0x7f0000);
    for (int i = 0; i + 1 < h; ++i)
    {
        for (int j = 0; j + 1 < w; ++j)
        {
            corecvs::Vector2dd A, B, C, D;
            A = bestBoard[i + 0][j + 0];
            B = bestBoard[i + 1][j + 0];
            C = bestBoard[i + 0][j + 1];
            D = bestBoard[i + 1][j + 1];

            corecvs::Matrix33 res, orientation;
            double score;
            int cl = classifier[i][j] = gen.getBestToken(img, {A, B, C, D}, score, orientation, res);
            if (cl >= 0)
            std::cout << "DETECTED: " << cl << " " << score << std::endl;

            DpImage mask(buffer.h, buffer.w);
            for (int i = 0; i < 1000; ++i)
            {
                for (int j = 0; j < 1000; ++j)
                {
                    corecvs::Vector3dd pt = res * Vector3dd(j * (1e-3), i * (1e-3), 1.0);

                    int xx = pt[0], yy = pt[1];
                    if (xx >= 0 && xx < img.w && yy >= 0 && yy < img.h)
                    {
                        mask.element(yy, xx) = 1.0;
                    }

                }
            }
            {

                for (int i = 0; i < buffer.h; ++i)
                {
                    for (int j = 0; j < buffer.w; ++j)
                    {
                        auto A = buffer.element(i, j);
                        auto B = cl >= 0 ? colors[cl] : board_col;
                        if (mask.element(i, j) > 0.0 && fin.element(i, j) < 0.5)
                        {
                            buffer.element(i, j) = RGBColor::lerpColor(A, B, 0.3);
                        }
                    }
                }
            }
            if (cl >= 0)
            {
                DpImage maskA(buffer.h, buffer.w), maskB(buffer.h, buffer.w);
                Vector2dd A = (orientation * Vector3dd(0.0, 0.0, 1.0)).project(),
                          B = (orientation * Vector3dd(1.0, 0.0, 1.0)).project(),
                          C = (orientation * Vector3dd(0.0, 1.0, 1.0)).project();
                std::cout << "ABC: " <<  A << " " << B << " " << C << std::endl;
                for (int i = 0; i < 1000; ++i)
                {
                    double alpha = i * 1e-3;
                    Vector2dd AB = A * alpha + B * (1.0 - alpha);
                    Vector2dd AC = A * alpha + C * (1.0 - alpha);
                    Vector3dd ab = res * Vector3dd(AB[0], AB[1], 1.0);
                    Vector3dd ac = res * Vector3dd(AC[0], AC[1], 1.0);

                                    ab = ab * (1.0 / ab[2]);
                                    int xxx = ab[0], yyy = ab[1];
                                    for (int xx = xxx; xx < xxx + 3; ++xx)
                                        for (int yy = yyy; yy < yyy + 3; ++yy)
                                    if (xx >= 0 && xx < img.w && yy >= 0 && yy < img.h)
                                    {
                                        maskA.element(yy, xx) = 1.0;
                                        fin.element(yy, xx) = 1.0;
                                    }
                                    ac = ac * (1.0 / ac[2]);
                                    xxx = ac[0], yyy = ac[1];
                                    for (int xx = xxx; xx < xxx + 3; ++xx)
                                        for (int yy = yyy; yy < yyy + 3; ++yy)
                                    if (xx >= 0 && xx < img.w && yy >= 0 && yy < img.h)
                                    {
                                        fin.element(yy, xx) = 1.0;
                                        maskB.element(yy, xx) = 1.0;
                                    }

                }
                for (int i = 0; i < buffer.h; ++i)
                {
                    for (int j = 0; j < buffer.w; ++j)
                    {
                        if (maskA.element(i, j) > 0.0)
                            buffer.element(i, j) = RGBColor::Blue();
                        if (maskB.element(i, j) > 0.0)
                            buffer.element(i, j) = RGBColor::Green();
                    }
                }
            }
        }
    }
    for (auto&v :classifier)
    {
        for (auto &c : v)
            std::cout << c << "\t";
        std::cout << std::endl;
    }

    if (stats != NULL) stats->resetInterval("Filling result");

    return true;
}

void ChessboardDetector::setStatistics(Statistics *stats)
{
    this->stats = stats;
}

Statistics *ChessboardDetector::getStatistics()
{
    return stats;
}

size_t ChessboardDetector::detectPatterns(corecvs::RGB24Buffer &buffer, std::vector<ObservationList> &patterns)
{
    patterns.clear();
    DpImage grayscale(buffer.h, buffer.w);
    grayscale.binaryOperationInPlace(buffer, [](const double & /*a*/, const corecvs::RGBColor &b) {
        return b.yd() / 255.0;
    });
    if(detectPatterns(grayscale))
        getPatterns(patterns);
    return patterns.size();
}

size_t ChessboardDetector::detectPatterns(corecvs::DpImage &buffer)
{
    BoardAlignerParams activeAlignerParams = aligner->getAlignerParams();

    ChessBoardDetectorMode mode =  getMode(activeAlignerParams);
    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> boards;
    auto detected = detectPatternCandidates(buffer, boards);
    std::cout << (detected ? "[Success]" : "[Fail]")<< " Detected " << corners.size() << " corners; assembled " << boards.size() << " boards." << std::endl;
    if(!detected)
        return false;

    bool checkW = !!(mode & ChessBoardDetectorMode::FIT_WIDTH);
    bool checkH = !!(mode & ChessBoardDetectorMode::FIT_HEIGHT);

    allPatterns.clear();

    for (auto& b : boards)
    {
        bool found = false;
        int bw = (int)b[0].size();
        int bh = (int)b.size();
        std::vector<std::vector<corecvs::Vector2dd>> best;

        bool fitw = (bw == aligner->idealWidth);
        bool fith = (bh == aligner->idealHeight);


        if ((!checkW || fitw) && (!checkH || fith))
        {
            best = b;
            found = true;
        }
        fitw = (bw == aligner->idealHeight);
        fith = (bh == aligner->idealWidth);
        if ((!checkW || fitw) && (!checkH || fith))
        {
            best = b;
            found = true;
            decltype(best) best_t;
            int bw = (int)best[0].size(), bh = (int)best.size();
            best_t.resize(bw);
            for (auto& r : best_t)
                r.resize(bh);

            for (int i = 0; i < bh; ++i)
            {
                for (int j = 0; j < bw; ++j)
                {
                    best_t[j][i] = best[i][j];
                }
            }
            best = best_t;
        }
        if (!found)
            continue;

        aligner->bestBoard = best;
        bool aligned = aligner->align(buffer);
        std::cout << (aligned ? "ALIGN OK" : "ALIGN FAILED") << std::endl;
        if (aligned)
        {
            result = aligner->observationList;
            for (auto& p : result)
            {
                p.point.x() *= cellSizeHor();
                p.point.y() *= cellSizeVert();
            }
            std::cout << result.size() << " features" << std::endl;
            std::cout << "found marker #" << result.patternIdentity << std::endl;
            auto detectedPattern = result;
            allPatterns.emplace_back(detectedPattern);
        }
    }
    return allPatterns.size();
}

void ChessboardDetector::getPatterns(std::vector<corecvs::ObservationList>& patterns)
{
    patterns = allPatterns;
}

void ChessboardDetector::drawCorners(corecvs::RGB24Buffer & image)
{
    if (corners.size() > 0)
    {
        AbstractPainter<RGB24Buffer> p(&image);
        for (size_t i = 0; i < corners.size(); i++)
        {
            p.drawCircle(corners[i].pos.x(), corners[i].pos.y(), 1, corecvs::RGBColor(0xffff00));
            p.drawFormat(corners[i].pos.x() + 1, corners[i].pos.y() + 1, corecvs::RGBColor(0xffff00), 2, "%d", i);
        }
    }

}

#if 0
void ChessboardDetector::dumpState()
{
    PrinterVisitor printer(2,2);
    cout << "ChessboardDetector::dumpState():We are using following configs" << endl;

    cout << "CheckerboardDetectionParameters:"  << endl;
    static_cast<CheckerboardDetectionParameters *>(this)->accept(printer);
    /*cout << "BoardAlignerParams:"  << endl;
    boardParams.accept(printer);*/
    cout << "ChessBoardAssemblerParams:"  << endl;
    static_cast<ChessBoardAssemblerParams *>(&assembler)->accept(printer);
    cout << "ChessBoardCornerDetectorParams:"  << endl;
    static_cast<ChessBoardCornerDetectorParams *>(&detector)->accept(printer);
}
#endif

