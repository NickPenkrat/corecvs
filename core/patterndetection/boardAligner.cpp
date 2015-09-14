#include "boardAligner.h"

#include <cassert>

BoardAligner::BoardAligner(BoardAlignerParams params) : BoardAlignerParams(params)
{
}

bool BoardAligner::align(DpImage &img)
{
    if (!bestBoard.size() || !bestBoard[0].size())
        return false;
    observationList.clear();
    // used in 4/5 cases; means nothing in 5th
    fixOrientation();
    bool result = false;
    switch(type)
    {
        case AlignmentType::FIT_ALL:
           result = alignDim(img,  true,  true);
           break;
        case AlignmentType::FIT_WIDTH:
           result = alignDim(img,  true, false);
           break;
        case AlignmentType::FIT_HEIGHT:
           result = alignDim(img, false,  true);
           break;
        case AlignmentType::FIT_MARKER_ORIENTATION:
           result = alignSingleMarker(img);
           break;
        case AlignmentType::FIT_MARKERS:
           result = alignMarkers(img);
           break;
    }
    return result && createList();
}

void BoardAligner::transpose()
{


    if (!bestBoard.size()) return;

    int w = bestBoard[0].size(), h = bestBoard.size();
    decltype(bestBoard) board(w);
    for (auto& r: board) r.resize(h);

    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            board[j][i] = bestBoard[i][j];
        }
    }
    bestBoard = board;
}

// transpose to decrease row-wise y variance
// sort in-rows increasing x
// sort rows increasing y
void BoardAligner::fixOrientation()
{
    int w = bestBoard[0].size();
    int h = bestBoard.size();

    std::vector<double> sr(h), ssr(h), sc(w), ssc(w);
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            double yv = bestBoard[i][j][1];
            sr[i] += yv;
            sc[j] += yv;
            ssr[i] += yv * yv;
            ssc[j] += yv * yv;
        }
    }

    double mr = 0.0, mc = 0.0;
    for (int i = 0; i < h; ++i)
    {
        sr[i] /= w;
        mr += ssr[i] = std::sqrt(ssr[i] / w - sr[i] * sr[i]);
    }
    for (int j = 0; j < w; ++j)
    {
        sc[j] /= h;
        mc += ssc[j] = std::sqrt(ssc[j] / h - sc[j] * sc[j]);
    }
    if (mr > mc)
        transpose();
    w = bestBoard[0].size();
    h = bestBoard.size();

    for (auto& r: bestBoard)
        std::sort(r.begin(), r.end(), [](const corecvs::Vector2dd &a, const corecvs::Vector2dd &b) { return a[0] < b[0]; });

    std::vector<std::pair<double, int>> ymeans(h);
    for (int i = 0; i < h; ++i)
    {
        ymeans[i].second = i;
        for (auto& v: bestBoard[i])
            ymeans[i].first += v[1];
    }
    std::sort(ymeans.begin(), ymeans.end(), [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first < b.first; });

    decltype(bestBoard) board;
    for (auto& p: ymeans)
        board.emplace_back(std::move(bestBoard[p.second]));
    bestBoard = board;
}

bool BoardAligner::alignDim(DpImage &img, bool fitW, bool fitH)
{
    if (!fitW && !fitH) return false;
    int w = bestBoard[0].size(), h = bestBoard.size();
    std::cout << "Best board: " << w << " x " << h << "; Req: " << idealWidth << " x " << idealHeight << std::endl;
    if (fitW && fitH)
    {
        if (w != idealWidth)
        {
            std::swap(w, h);
            transpose();
        }
        if (w != idealWidth || h != idealHeight)
            return false;
    }
    else
    {
        if (fitW && bestBoard[0].size() != idealWidth)
        {
            return false;
        }
        if (fitH && bestBoard.size() != idealHeight)
        {
            return false;
        }
    }
    std::cout << "Ok, continue..." << std::endl;
    corecvs::Vector2dd mean;
    for (auto& r: bestBoard)
        for (auto& c: r)
            mean += c;
    mean /= (w * h) * 0.5;

    int lb = mean[0] > img.w ? 0 : idealWidth - w;
    int tb = mean[1] > img.h ? 0 : idealHeight - h;

    classifier.resize(h);
    for (auto& row: classifier)
        row.resize(w);

    for (int i = 0; i < h; ++i)
    {

        for (int j = 0; j < w; ++j)
        {
            classifier[i][j].first = lb + j;
            classifier[i][j].second = tb + i;
            std::cout << "[" << lb + j << ", " << tb + i << "] ";
        }
        std::cout << std::endl;
    }
#if 0
    for (int i = lb; i < lb + w; ++i)
    {
        for (int j = tb; j < tb + h; ++j)
        {
            observationList.emplace_back(bestBoard[j][i], corecvs::Vector3dd(i, j, 0.0));
        }
    }
#endif
    return true;
}

bool BoardAligner::alignSingleMarker(DpImage &img)
{
    assert(markerCells.size() == circleCenters.size());
    assert(markerCells.size() == 1);

    classify(false, img);
    return bfs();
}

bool BoardAligner::alignMarkers(DpImage &img)
{
    assert(markerCells.size() == circleCenters.size());

    classify(true, img);
    return bfs();
}

bool BoardAligner::bfs()
{
    decltype(classifier) cc = classifier;
    int h = classifier.size();
    int w = classifier[0].size();

    int seedx = -1, seedy = -1;
    int xdx, xdy, ydx, ydy;

    int corner[4][2][2] =
    {
        {{ 1, 0}, {0, 1}},
        {{ 1, 0}, {0,-1}},
        {{-1, 0}, {0, 1}},
        {{-1, 0}, {0,-1}}
    };

    for (int y = 0; y < h && seedx < 0; ++y)
    {
        for (int x = 0; x < w && seedx < 0; ++x)
        {
            for (int corner_id = 0; corner_id < 4; ++corner_id)
            {
                int x1 = x + corner[corner_id][0][0];
                int y1 = y + corner[corner_id][0][1];
                int x2 = x + corner[corner_id][1][0];
                int y2 = y + corner[corner_id][1][1];

                if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0)
                    continue;
                if (x1 >= w || x2 >= w || y1 >= h || y2 >= h)
                    continue;

                if (classifier[y1][x1].first < 0 ||
                    classifier[y ][x ].first < 0 ||
                    classifier[y2][x2].first < 0)
                    continue;
                seedx = x; seedy = y;

                int xv = classifier[y ][x ].first,
                    yv = classifier[y ][x ].second,
                    xv1= classifier[y1][x1].first,
                    yv1= classifier[y1][x1].second,
                    xv2= classifier[y2][x2].first,
                    yv2= classifier[y2][x2].second;
                xdx = corner[corner_id][0][0] * (xv1 - xv);
                ydx = corner[corner_id][0][0] * (yv1 - yv);
                xdy = corner[corner_id][1][1] * (xv2 - xv);
                ydy = corner[corner_id][1][1] * (yv2 - yv);
                break;
            }
        }
    }

    if (seedx < 0)
        return false;
    if ((xdx * xdx + xdy * xdy != 1) || (ydx * ydx + ydy * ydy != 1))
        return false;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            int cx = classifier[y][x].first  = (x - seedx) * xdx + (y - seedy) * xdy + seedx;
            int cy = classifier[y][x].second = (x - seedx) * ydx + (y - seedy) * ydy + seedy;

            if (cx < 0 || cx >= idealWidth || cy < 0 || cy >= idealHeight)
                return false;
            if (cc[y][x].first >= 0)
            {
                if (cx != cc[y][x].first || cy != cc[y][x].second)
                    return false;
            }

        }
    }
    return true;
}

bool BoardAligner::createList()
{
    int h = classifier.size();
    int w = classifier[0].size();
    observationList.clear();
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            observationList.emplace_back(
                    corecvs::Vector3dd(
                        classifier[y][x].first * 1.0,
                        classifier[y][x].second * 1.0, 
                        0.0),
                    bestBoard[y][x]);
            if (classifier[y][x].first < 0 ||
                classifier[y][x].second < 0 ||
                classifier[y][x].first >= idealWidth ||
                classifier[y][x].second >= idealHeight)
                return false;
        }
    }
    return true;
}

void BoardAligner::classify(bool trackOrientation, DpImage &img)
{
    int h = bestBoard.size(), w = bestBoard[0].size();
    classifier.clear();
    classifier.resize(h);
    for (auto& r: classifier)
        r.resize(w, std::make_pair(-1, -1));

    CirclePatternGenerator gen;
    int N = circleCenters.size();
    for (int i = 0; i < N; ++i)
        gen.addToken(i, circleRadius, circleCenters[i]);

    for (int i = 0; i + 1 < h; ++i)
    {
        for (int j = 0; j + 1 < w; ++j)
        {
            corecvs::Vector2dd A, B, C, D;
            A = bestBoard[i + 0][j + 0];
            B = bestBoard[i + 0][j + 1];
            C = bestBoard[i + 1][j + 0];
            D = bestBoard[i + 1][j + 1];
            
            double score;
            corecvs::Matrix33 orientation, res;
            int cl =  gen.getBestToken(img, {A, B, C, D}, score, orientation, res);

            if (!trackOrientation)
            {
                orientation = corecvs::Matrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
            }

            int order[][2] = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};

            if (cl >= 0)
            {
                // Take sq. orientation and place classifier
                for (int k = 0; k < 4; ++k)
                {
                    auto P = (orientation * corecvs::Vector3dd(order[k][0], order[k][1], 1.0).project()) + corecvs::Vector2dd(0.5, 0.5);
                    assert(P[0] > 0.0 && P[1] > 0.0);
                    assert(P[0] < 1.5 && P[1] < 1.5);
                    classifier[i + P[1]][j + P[0]] = markerCells[cl][k];

                }

            }

        }
    }
}
