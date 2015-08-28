#include "chessBoardDetector.h"

ChessboardDetector::ChessboardDetector(ChessBoardDetectorParams params, ChessBoardCornerDetectorParams detectorParams, ChessBoardAssemblerParams assemblerParams)
    : ChessBoardDetectorParams(params), detector(detectorParams)
{
    assemblerParams.hypothesisDimensions = 0;
    if (!!(mode & ChessBoardDetectorMode::FIT_WIDTH))
    {
        assemblerParams.hypothesisDimensions++;
        assemblerParams.hypothesisDim[0] = w;
    }
    if (!!(mode &ChessBoardDetectorMode::FIT_HEIGHT))
    {
        assemblerParams.hypothesisDim[assemblerParams.hypothesisDimensions++] = h;
    }
    assembler = ChessBoardAssembler(assemblerParams);
}

bool ChessboardDetector::detectPattern(corecvs::G8Buffer &buffer)
{
    DpImage grayscale(buffer.h, buffer.w);
    grayscale.binaryOperationInPlace(buffer, [](const double & /*a*/, const unsigned char &b) { return ((double)b) / 255.0; });
    return detectPattern(grayscale);
}

bool ChessboardDetector::detectPattern(corecvs::RGB24Buffer &buffer)
{
    DpImage grayscale(buffer.h, buffer.w);
    grayscale.binaryOperationInPlace(buffer, [](const double & /*a*/, const corecvs::RGBColor &b) { return (b.b() * 0.114 + b.g() * 0.587 + b.r() * 0.299 ) / 255.0; });
    return detectPattern(grayscale);
}

bool ChessboardDetector::detectPattern(DpImage &buffer)
{
    corners.clear();
    bestPattern = RectangularGridPattern();
    detector.detectCorners(buffer, corners);

    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> boards;

    assembler.assembleBoards(corners, boards);

    if (!boards.size())
        return false;

    std::vector<std::vector<corecvs::Vector2dd>> best;
    bool transposed = false, found = false;

    bool checkW = !!(mode & ChessBoardDetectorMode::FIT_WIDTH);
    bool checkH = !!(mode & ChessBoardDetectorMode::FIT_HEIGHT);

    for (auto& b: boards)
    {
        int bw = (int)b[0].size(), bh = (int)b.size();

        bool fitw = bw == w, fith  = bh == h;
        if ((!checkW || fitw) && (!checkH || fith))
        {
            best = b;
            found = true;
            break;
        }
        fitw = bh == w;
        fith = bw == h;
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

    int l = bw == w ? 0 : center[0] > buffer.w ? 0 : w - bw;
    int t = bh == h ? 0 : center[1] > buffer.h ? 0 : h - bh;

    result.clear();
    for (int i = 0; i < bh; ++i)
    {
        for (int j = 0; j < bw; ++j)
            result.emplace_back(corecvs::Vector3dd((j + l) * stepX, (i + t) * stepY, 0.0), best[i][j]);
    }
    return true;
}

void ChessboardDetector::getPointData(ObservationList &observations)
{
    observations = result;
}
