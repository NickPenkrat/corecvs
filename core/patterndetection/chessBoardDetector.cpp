#include "chessBoardDetector.h"
#include "homographyReconstructor.h"
#include <fstream>

ChessboardDetector::ChessboardDetector (
        CheckerboardDetectionParameters params,
        BoardAlignerParams alignerParams,
        ChessBoardCornerDetectorParams detectorParams,
        ChessBoardAssemblerParams assemblerParams
    )
    : CheckerboardDetectionParameters(params),
      BoardAligner(alignerParams),
      detector(detectorParams),
      stats(NULL)
{
    assemblerParams.hypothesisDimensions = 0;
    switch (type)
    {
        case AlignmentType::FIT_MARKER_ORIENTATION:
        case AlignmentType::FIT_MARKERS:
            break;
        case AlignmentType::FIT_WIDTH:
            assemblerParams.hypothesisDimensions = 1;
            assemblerParams.hypothesisDim[0] = idealWidth;
            break;
        case AlignmentType::FIT_HEIGHT:
            assemblerParams.hypothesisDimensions = 1;
            assemblerParams.hypothesisDim[0] = idealHeight;
            break;
        case AlignmentType::FIT_ALL:
            assemblerParams.hypothesisDimensions = 2;
            assemblerParams.hypothesisDim[0] = idealWidth;
            assemblerParams.hypothesisDim[1] = idealHeight;
            break;
    }
    assembler = ChessBoardAssembler(assemblerParams);
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

bool ChessboardDetector::detectPattern(DpImage &buffer)
{
    if (stats != NULL) stats->startInterval();

    ChessBoardDetectorMode mode =  getMode(*this);
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

    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> boards;

    if (stats != NULL) {
        prefix = stats->prefix;
        stats->prefix = "Assembler -> " + stats->prefix;
        assembler.setStatistics(stats);
    }
    BoardAlignerParams params(*this);
    BoardAligner aligner(params);
    assembler.assembleBoards(corners, boards, &aligner, &buffer);
    if (stats != NULL) stats->prefix = prefix;

    if (!boards.size())
        return false;

    if (stats != NULL) stats->resetInterval("Assemble");

    bool /*transposed = false,*/ found = false;

    bool checkW = !!(mode & ChessBoardDetectorMode::FIT_WIDTH);
    bool checkH = !!(mode & ChessBoardDetectorMode::FIT_HEIGHT);

    for (auto& b: boards)
    {
        int bw = (int)b[0].size();
        int bh = (int)b.size();

        bool fitw = (bw == idealWidth);
        bool fith = (bh == idealHeight);


        if ((!checkW || fitw) && (!checkH || fith))
        {
            bestBoard = b;
            found = true;
            break;
        }
        fitw = (bh ==  idealWidth);
        fith = (bw == idealHeight);
        if ((!checkW || fitw) && (!checkH || fith))
        {
            bestBoard = b;
            found = true;
            //transposed = true;
            break;
        }
    }

    if (!found)
        return false;

    bool aligned = align(buffer);
    if (aligned)
    {
        std::cout << (aligned ? "ALIGN OK" : "ALIGN FAILED") << std::endl;
        result = observationList;
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
    BoardAligner::drawDebugInfo(buffer);
}

// FIXME: Temporary code; needs serious rework
bool ChessboardDetector::classify(DpImage &img, CirclePatternGenerator &gen, corecvs::RGB24Buffer &buffer)
{
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
            if (cl >= 0){
                DpImage maskA(buffer.h, buffer.w), maskB(buffer.h, buffer.w);
                corecvs::Vector2dd A = (orientation * corecvs::Vector3dd(0.0, 0.0, 1.0)).project(),
                                B = (orientation * corecvs::Vector3dd(1.0, 0.0, 1.0)).project(),
                                C = (orientation * corecvs::Vector3dd(0.0, 1.0, 1.0)).project();
                std::cout << "ABC: " <<  A << " " << B << " " << C << std::endl;
                for (int i = 0; i < 1000; ++i)
                {
                    double alpha = i * 1e-3;
                    corecvs::Vector2dd AB = A * alpha + B * (1.0 - alpha);
                    corecvs::Vector2dd AC = A * alpha + C * (1.0 - alpha);
                    corecvs::Vector3dd ab = res * corecvs::Vector3dd(AB[0], AB[1], 1.0);
                    corecvs::Vector3dd ac = res * corecvs::Vector3dd(AC[0], AC[1], 1.0);
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
                            buffer.element(i, j) = RGBColor(0xff0000);
                        if (maskB.element(i, j) > 0.0)
                            buffer.element(i, j) = RGBColor(0x00ff00);
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

