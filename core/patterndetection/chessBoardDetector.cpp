#include "chessBoardDetector.h"
#include "homographyReconstructor.h"
#include <fstream>

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
    grayscale.binaryOperationInPlace(buffer, [](const double &a, const unsigned char &b) { return ((double)b) / 255.0; });
    return detectPattern(grayscale);
}

bool ChessboardDetector::detectPattern(corecvs::RGB24Buffer &buffer)
{
    DpImage grayscale(buffer.h, buffer.w);
    grayscale.binaryOperationInPlace(buffer, [](const double &a, const corecvs::RGBColor &b) { return (b.b() * 0.114 + b.g() * 0.587 + b.r() * 0.299 ) / 255.0; });
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
   
    bool transposed = false, found = false;

    bool checkW = !!(mode & ChessBoardDetectorMode::FIT_WIDTH);
    bool checkH = !!(mode & ChessBoardDetectorMode::FIT_HEIGHT);

    for (auto& b: boards)
    {
        int bw = b[0].size(), bh = b.size();

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
        int bw = best[0].size(), bh = best.size();
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
#if 0

    // XXX: Here we need to fix some orientation of partially-visible pattern
    //      It is not possible, if we have no additional data (e.g. markers)
    //      So we orient it in some way.
    std::vector<std::pair<double, int>> ymeans;
    std::vector<std::pair<double, int>> xmeans;
    int idx = 0;
    for (auto& r: best)
    {
        Vector2dd sum(0.0, 0.0);
        for (auto& p: r)
        {
            sum += p;
        }
        sum /= r.size();
        ymeans.emplace_back(sum[1], idx);
        xmeans.emplace_back(sum[0], idx++);
    }
    double sx = 0.0, sy = 0.0, ssqx = 0.0, ssqy = 0.0;
    for (auto &v: xmeans)
    {
        sx += v.first;
        ssqx += v.first * v.first;
    }
    for (auto &v: ymeans)
    {
        sy += v.first;
        ssqy += v.first * v.first;
    }
    sx /= idx;
    sy /= idx;
    ssqx = std::sqrt(ssqx / idx - sx * sx);
    ssqy = std::sqrt(ssqy / idx - sy * sy);
    if (ssqx > ssqy)
    {
        decltype(best) best_t;
        int bw = best[0].size(), bh = best.size();
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
        ymeans.clear();
    idx = 0;
    for (auto& r: best)
    {
        Vector2dd sum(0.0, 0.0);
        for (auto& p: r)
        {
            sum += p;
        }
        sum /= r.size();
        ymeans.emplace_back(sum[1], idx);
    }
    }
    std::sort(ymeans.begin(), ymeans.end(), [](decltype(ymeans[0]) a, decltype(ymeans[0]) b) { return a.first < b.first; });
    if (transposed)
    {
        decltype(best) best_t;
        int bw = best[0].size(), bh = best.size();
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

    decltype(best) best_reorder(best.size());
    for (int i = 0; i < best.size(); ++i)
    {
        best_reorder[i] = best[ymeans[i].second];
        std::sort(best_reorder[i].begin(), best_reorder[i].end(), [](corecvs::Vector2dd &a, corecvs::Vector2dd &b) { return a[0] < b[0]; });
    }
    best = best_reorder;
#endif
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
    int bw = best[0].size(), bh = best.size();

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

// FIXME: Temporary code; needs serious rework
void ChessboardDetector::classify(DpImage &img, CirclePatternGenerator &gen, corecvs::RGB24Buffer &buffer)
{
    int w = best[0].size();
    int h = best.size();
    std::vector<std::vector<int>> classifier(h - 1);
    for (auto& cv: classifier) cv.resize(w - 1);
    std::cout << w << " x " << h << std::endl;

    DpImage queries((h - 1) * gen.w, (w - 1) * gen.w);
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
            A = best[i + 0][j + 0];
            B = best[i + 1][j + 0];
            C = best[i + 0][j + 1];
            D = best[i + 1][j + 1];

            corecvs::HomographyReconstructor c2i;
            c2i.addPoint2PointConstraint(corecvs::Vector2dd(0.0, 0.0), A);
            c2i.addPoint2PointConstraint(corecvs::Vector2dd(1.0, 0.0), B);
            c2i.addPoint2PointConstraint(corecvs::Vector2dd(0.0, 1.0), C);
            c2i.addPoint2PointConstraint(corecvs::Vector2dd(1.0, 1.0), D);
            corecvs::Matrix33 AA, BB;
            c2i.normalisePoints(AA, BB);
            auto res = c2i.getBestHomographyLSE();
            res = c2i.getBestHomographyLM(res);
            res = BB.inv() * res * AA;

            DpImage query(gen.w, gen.w);
            for (int y = 0; y < gen.w; ++y)
            {
                for (int x = 0; x < gen.w; ++x)
                {
                    corecvs::Vector3dd pt = res * Vector3dd(x * (1.0 / gen.w), y * (1.0 / gen.w), 1.0);
                    pt = pt * (1.0 / pt[2]);
                    int xx = pt[0], yy = pt[1];
                    if (xx >= 0 && xx < img.w && yy >= 0 && yy < img.h)
                    {
                        queries.element(i * gen.w + y, j * gen.w + x) = query.element(y, x) = img.element(yy, xx);
                    }
                }
            }

            double score, secondBest;
            corecvs::Matrix33 orientation;
            std::cout << i << ", " << j << std::endl;
            int cl = classifier[i][j] = gen.getBestToken(query, score, orientation, secondBest);




            DpImage mask(buffer.h, buffer.w);
            for (int i = 0; i < 1000; ++i)
            {
                for (int j = 0; j < 1000; ++j)
                {
                    corecvs::Vector3dd pt = res * Vector3dd(j * (1e-3), i * (1e-3), 1.0);
                    pt = pt * (1.0 / pt[2]);
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
    std::ofstream of;
    of.open("data.m");
    of << "qq = [" << std::endl;
    for (int i = 0; i < queries.h; ++i)
    {
        for (int j = 0; j < queries.w; ++j)
        {
            of << queries.element(i, j) << " ";
        }
        of << "; ";
    }
    of << "]; " << std::endl;
    for (auto&v :classifier)
    {
        for (auto &c : v)
            std::cout << c << "\t";
        std::cout << std::endl;
    }
}
