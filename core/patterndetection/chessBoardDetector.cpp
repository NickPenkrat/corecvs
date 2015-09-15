#include "chessBoardDetector.h"

ChessboardDetector::ChessboardDetector (
        CheckerboardDetectionParameters params,
        ChessBoardCornerDetectorParams detectorParams,
        ChessBoardAssemblerParams assemblerParams
    )
    : CheckerboardDetectionParameters(params),
      detector(detectorParams),
      stats(NULL)
{
    ChessBoardDetectorMode mode =  getMode(*this);

    assemblerParams.hypothesisDimensions = 0;
    if (!!(mode & ChessBoardDetectorMode::FIT_WIDTH))
    {
        assemblerParams.hypothesisDimensions++;
        assemblerParams.hypothesisDim[0] = horCrossesCount();
    }
    if (!!(mode &ChessBoardDetectorMode::FIT_HEIGHT))
    {
        assemblerParams.hypothesisDim[assemblerParams.hypothesisDimensions++] = vertCrossesCount();
    }
    assembler = ChessBoardAssembler(assemblerParams);
}

ChessBoardDetectorMode ChessboardDetector::getMode(const CheckerboardDetectionParameters &params)
{
    ChessBoardDetectorMode mode = ChessBoardDetectorMode::BEST;
    if (params. fitWidth()) mode = mode | ChessBoardDetectorMode::FIT_WIDTH;
    if (params.fitHeight()) mode = mode | ChessBoardDetectorMode::FIT_HEIGHT;
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
    assembler.assembleBoards(corners, boards);
    if (stats != NULL) stats->prefix = prefix;

    if (!boards.size())
        return false;


    if (stats != NULL) stats->resetInterval("Assemble");


    std::vector<std::vector<corecvs::Vector2dd>> best;
    bool transposed = false, found = false;

    bool checkW = !!(mode & ChessBoardDetectorMode::FIT_WIDTH);
    bool checkH = !!(mode & ChessBoardDetectorMode::FIT_HEIGHT);

    for (auto& b: boards)
    {
        int bw = (int)b[0].size();
        int bh = (int)b.size();

        bool fitw = (bw == horCrossesCount());
        bool fith = (bh == vertCrossesCount());


        if ((!checkW || fitw) && (!checkH || fith))
        {
            best = b;
            found = true;
            break;
        }
        fitw = (bh ==  horCrossesCount());
        fith = (bw == vertCrossesCount());
        if ((!checkW || fitw) && (!checkH || fith))
        {
            best = b;
            found = true;
            transposed = true;
            break;
        }
    }

    if (!found)
        return false;

    std::cout << best[0].size() << " x " << best.size() << std::endl;
    if (transposed)
    {
        decltype(best) best_t;
        int bw = (int)best[0].size(), bh = (int)best.size();
        best_t.resize(bw);
        for (auto& r: best_t)
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

    // XXX: Here we need to fix some orientation of partially-visible pattern
    //      It is not possible, if we have no additional data (e.g. markers)
    //      So we orient it in some way.
    std::vector<std::pair<double, int>> ymeans;
    int idx = 0;
    for (auto& r: best)
    {
        Vector2dd sum(0.0, 0.0);
        for (auto& p: r)
            sum += p;
        sum /= r.size();
        ymeans.emplace_back(sum[1], idx++);
    }
    std::sort(ymeans.begin(), ymeans.end(), [](std::pair<double, int> a, std::pair<double, int> b) { return a.first < b.first; });

    decltype(best) best_reorder(best.size());
    for (size_t i = 0; i < best.size(); ++i)
    {
        best_reorder[i] = best[ymeans[i].second];
        std::sort(best_reorder[i].begin(), best_reorder[i].end(), [](corecvs::Vector2dd a, corecvs::Vector2dd b) { return a[0] < b[0]; });
    }
    best = best_reorder;

    corecvs::Vector2dd center(0.0, 0.0);
    int cnt = 0;
    for (auto& r: best)
    {
        for (auto& p: r)
        {
            center += p;
            cnt++;
        }
    }
    center /= cnt;
    center *= 2.0;
    int bw = (int)best[0].size(), bh = (int)best.size();

    int l = (bw ==  horCrossesCount()) ? 0 : (center[0] > buffer.w ? 0 : ( horCrossesCount() - bw));
    int t = (bh == vertCrossesCount()) ? 0 : (center[1] > buffer.h ? 0 : (vertCrossesCount() - bh));

    result.clear();
    for (int i = 0; i < bh; ++i)
    {
        for (int j = 0; j < bw; ++j)
            result.emplace_back(corecvs::Vector3dd((j + l) * cellSizeHor(), (i + t) * cellSizeVert(), 0.0), best[i][j]);
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

void ChessboardDetector::getPointData(ObservationList &observations)
{
    observations = result;
}
