#include "core/patterndetection/chessBoardAssembler.h"
#include "core/patterndetection/boardAligner.h"
#include "core/stats/calculationStats.h"

#include <unordered_set>
#include <queue>

using corecvs::Vector2dd;
using corecvs::Statistics;

ChessBoardAssembler::ChessBoardAssembler(ChessBoardAssemblerParams params) : ChessBoardAssemblerParams(params)
{
}

ChessBoardAssembler::ChessBoardAssembler(const ChessBoardAssembler &other) : ChessBoardAssemblerParams(other)
{
}

ChessBoardAssembler& ChessBoardAssembler::operator=(const ChessBoardAssembler& other)
{
    boards.clear();
    corners.clear();
    ChessBoardAssemblerParams::operator=(other);
    return *this;
}

void ChessBoardAssembler::assembleBoards(std::vector<OrientedCorner> &corners_, std::vector<BoardCornersType> &boards_, BoardAligner* aligner_, DpImage *buffer_)
{
    aligner = aligner_;
    buffer = buffer_;
    corners = corners_;
    int N = (int)corners.size();
    int bs = std::max(1, N / 128);

    if (kdtree())
    {
        std::vector<OrientedCorner*> ptrs;
        for (auto& c: corners)
            ptrs.push_back(&c);
        kd = std::unique_ptr<corecvs::KDTree<OrientedCorner, 2>>(new corecvs::KDTree<OrientedCorner, 2>(ptrs));
    }

    stats->startInterval();
    Statistics::startInterval(stats);

    boards = corecvs::parallelable_reduce(0, N, bs,
        std::vector<RectangularGridPattern>(),
        [&](const corecvs::BlockedRange<int> &r, const std::vector<RectangularGridPattern> &init)
        {
            auto boards = init;
            BoardExpander expander(this);
            for (int seed = r.begin(); seed != r.end(); ++seed)
            {
                RectangularGridPattern board;
                if (!expander.initBoard(seed) || !expander.getExpandedBoard(board))
                    continue;

                acceptHypothesis(board, boards);
            }
            return boards;
        },
        [&](const std::vector<RectangularGridPattern> &l, const std::vector<RectangularGridPattern> &r)
        {
            auto res = l;
            for (auto &b : r)
                acceptHypothesis(b, res);
            return res;
        });

    Statistics::resetInterval(stats, "Board Expander");

    boards_.clear();
    std::sort(boards.begin(), boards.end(), [](const RectangularGridPattern &a, const RectangularGridPattern &b) { return a.score < b.score; });
    for (RectangularGridPattern& b: boards)
    {
        BoardCornersType board;
        for (auto& v: b.cornerIdx)
        {
            std::vector<corecvs::Vector2dd> row;
            for (int c: v) {
                row.push_back(corners[c].pos);
            }
            board.emplace_back(std::move(row));
        }
        boards_.emplace_back(std::move(board));
    }

    Statistics::resetInterval(stats, "Board Outputing");
}

bool ChessBoardAssembler::acceptBoard(const RectangularGridPattern &board)
{
    if (board.score > costThreshold())
        return false;
    int w = board.w();
    int h = board.h();
    if (hypothesisDimensions())
    {
        int maxfit = 0;
        for (int i = 0; i < 2; ++i)
        {
            int fit = 0;
            if (w == hypothesisDim(i & 1))
                fit++;
            if (h == hypothesisDim(i ^ 1))
                fit++;
            if (fit > maxfit) maxfit = fit;
        }
        if (maxfit < hypothesisDimensions())
            return false;
    }
    for (int i = 0; i < w; ++i)
        for (int j = 0; j + 1 < h; ++j)
        {
            double a = !(corners[board.cornerIdx[j][i]].pos - corners[board.cornerIdx[j + 1][i]].pos);
            if (a < minSeedDistance())
                return false;
        }
    for (int i = 0; i < h; ++i)
        for (int j = 0; j + 1 < w; ++j)
        {
            double a = !(corners[board.cornerIdx[i][j]].pos - corners[board.cornerIdx[i][j + 1]].pos);
            if (a < minSeedDistance())
                return false;
        }

    if (aligner && buffer)
    {
        DpImage bufferC = *buffer;
        BoardAligner alignerc = *aligner;
        alignerc.bestBoard.clear();
        for (auto &v: board.cornerIdx)
        {
            std::vector<corecvs::Vector2dd> row;
            for (auto &p: v)
                row.push_back(corners[p].pos);
            alignerc.bestBoard.push_back(row);
        }
        if (!alignerc.align(bufferC))
            return false;
    }
    return true;
}

void ChessBoardAssembler::addNonIntersectingBoardIntersections(const RectangularGridPattern &board, std::vector<RectangularGridPattern> &boards)
{
    bool intersects = false;
    bool best = true;
    std::vector<int> intersections;
    std::unordered_set<int> cset;
    for (auto& v: board.cornerIdx)
        for (auto& c: v)
            cset.insert(c);
    for (size_t i = 0; i < boards.size(); ++i)
    {
        auto& bb = boards[i];
        bool localIntersect = false;
        for (auto& v: bb.cornerIdx)
        {
            if (localIntersect) break;
            for (auto& c: v)
                if (cset.count(c))
                {
                    localIntersect = true;
                    break;
                }
        }
        if (localIntersect)
        {
            intersects = true;
            intersections.push_back((int)i);
            if (bb.score < board.score)
                best = false;
        }
        if (!best)
            break;
    }

    if (best)
    {
        if (intersects)
        {
            int idx = 0;
            size_t j = 0;
            for (size_t i = 0; i < boards.size(); ++i)
                if (j == intersections.size() || (size_t)intersections[j] != i)
                    boards[idx++] = boards[i];
                else
                    j++;
            boards.resize(idx);
        }
        boards.push_back(board);
    }

}

void ChessBoardAssembler::acceptHypothesis(const RectangularGridPattern &board, std::vector<RectangularGridPattern> &boards)
{
    if (!acceptBoard(board))
        return;
    addNonIntersectingBoardIntersections(board, boards);
}

void ChessBoardAssembler::acceptHypothesis(const RectangularGridPattern &board)
{
#ifdef WITH_TBB
    tbb::mutex::scoped_lock lock(mutex);
#endif
    acceptHypothesis(board, boards);
}

ChessBoardAssembler::BoardExpander::BoardExpander(ChessBoardAssembler *assembler)
    : assembler(assembler)
{}

bool ChessBoardAssembler::BoardExpander::initBoard(int seed)
{
    auto& corners = assembler->corners;
    double varThreshold = assembler->seedThreshold();
    // Find closest corners
    // Mark'em as used
    // Init board
    if (corners.size() < 9)
        return false;
    board.cornerIdx.clear();
    board.cornerIdx.resize(3);
    usedCorners.clear();
    usedCorners.resize(corners.size());
    for (auto& v: board.cornerIdx)
        v.resize(3);

    board.cornerIdx[1][1] = seed;
    usedCorners[seed] = 1;

    OrientedCorner& from = corners[seed];

    // North neighb
    int nwse[4], corn[4];
    double d_nwse[4], d_corn[4];
    for (int i = 0; i < 4; ++i)
    {
        Vector2dd dir = ((i & 1) * 2.0 - 1.0) * (i & 2 ? from.v1 : from.v2);
        if(!getNearest(seed, dir, nwse[i], d_nwse[i]))
            return false;
        usedCorners[nwse[i]] = 1;
    }
    for (int i = 0; i < 4; ++i)
    {
        Vector2dd dir = ((i & 1) * 2.0 - 1.0) * from.v2;
        if (!getNearest(nwse[2+((i & 2) >> 1)], dir, corn[i], d_corn[i]))
            return false;
        usedCorners[corn[i]] = 1;
    }

    board.cornerIdx[1][0] = nwse[1];
    board.cornerIdx[1][2] = nwse[0];
    board.cornerIdx[0][1] = nwse[2];
    board.cornerIdx[2][1] = nwse[3];

    board.cornerIdx[0][0] = corn[1];
    board.cornerIdx[0][2] = corn[0];
    board.cornerIdx[2][0] = corn[3];
    board.cornerIdx[2][2] = corn[2];

    double mindist = 1e100;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
        {
            if (i == 1 && j == 1)
                continue;
            mindist = std::min(mindist, !(corners[board.cornerIdx[i][j]].pos - corners[board.cornerIdx[1][1]].pos));
        }
    if (mindist < assembler->minSeedDistance())
        return false;

    std::vector<double> dv[2] = {{ d_nwse[0], d_nwse[1] }, { d_nwse[2], d_nwse[3], d_corn[0], d_corn[1], d_corn[2], d_corn[3]}};
    double s[2]   = {0.0, 0.0};
    double ssq[2] = {0.0, 0.0};
    for (int i = 0; i < 2; ++i)
    {
        for (auto& v: dv[i])
        {
            s[i] += v;
            ssq[i] += v * v;
        }
        int N;
        s[i] /= (N = (int)dv[i].size());
        ssq[i] = std::sqrt(ssq[i] / N - s[i] * s[i]);

        if (ssq[i] / s[i] > varThreshold)
        {
            return false;
        }
    }
    return true;
}

bool ChessBoardAssembler::BoardExpander::getNearest(int from_id, corecvs::Vector2dd dir, int& id, double &score)
{
    std::vector<OrientedCorner>& corners = assembler->corners;
    double tgPenalty = assembler->seedTgPenalty();
    OrientedCorner& from = corners[from_id];

    double best_dist = 1e100;
    int best_id = -1;
    int N = (int)corners.size();

    for (int i = 0; i < N; ++i)
    {
        if (usedCorners[i]) continue;

        auto& to = corners[i];
        auto dirp = to.pos - from.pos;

        if ((dir & dirp) < 0.0)
            continue;

        double dist_proj = dir & dirp;
        auto dist_rect = (dirp - dist_proj * dir).l2Metric();
        auto _score = dist_proj + tgPenalty * dist_rect;

        if (_score < best_dist)
        {
            best_dist = _score;
            best_id = i;
        }
    }

    if (best_id >= 0)
    {
        score = best_dist;
        id = best_id;
        return true;
    }
    return false;
}

bool ChessBoardAssembler::BoardExpander::growBoard()
{
    auto &corners = assembler->corners;
    board.getScore(corners);

    RectangularGridPattern best = board;
    std::vector<int> bestUsed = usedCorners;
    bool updated = false;

    for (int i = 0; i < 4; ++i)
    {
        std::vector<int> used = usedCorners;

        RectangularGridPattern dst = board;
        if(!growDir(static_cast<Direction>(i), dst, used))
            continue;
        if (dst.score < best.score)
        {
            updated = true;
            best = dst;
            bestUsed = used;
        }
    }

    if (!updated)
       return false;

    board = best;
    usedCorners = bestUsed;
    return true;
}

bool ChessBoardAssembler::BoardExpander::growDir(Direction dir, RectangularGridPattern &dst, std::vector<int> &usedCorners)
{
    std::vector<corecvs::Vector2dd> prediction;
    std::vector<int> assignment;

    predictor(dir, prediction);

    if (!assignNearest(prediction, usedCorners, assignment))
        return false;

    dst = board;
    switch (dir)
    {
        case Direction::UP:
            dst.cornerIdx.insert(dst.cornerIdx.begin(), assignment);
            break;
        case Direction::DOWN:
            dst.cornerIdx.push_back(assignment);
            break;
        case Direction::LEFT:
            for (int i = 0; i < dst.h(); ++i)
            {
                auto& v = dst.cornerIdx[i];
                v.insert(v.begin(), assignment[i]);
            }
            break;
        case Direction::RIGHT:
            for (int i = 0; i < dst.h(); ++i)
            {
                dst.cornerIdx[i].push_back(assignment[i]);
            }
            break;
    }

    auto &corners = assembler->corners;
    dst.getScore(corners);
    return true;
}


Vector2dd ChessBoardAssembler::BoardExpander::predict(Vector2dd a, Vector2dd b, Vector2dd c)
{
    double conservativity = assembler->conservativity();
    Vector2dd d1 = b - a, d2 = c - b;
    double alpha1 = d1.argument();
    double alpha2 = d2.argument();
    double alpha = 2.0 * alpha2 - alpha1;

    double l1 = d1.l2Metric();
    double l2 = d2.l2Metric();

    Vector2dd res = c + Vector2dd::FromPolar(alpha, conservativity * (2.0 * l2 - l1));
    return res;
}

void ChessBoardAssembler::BoardExpander::predictor(Direction dir, std::vector<Vector2dd> &prediction)
{
    auto& corners = assembler->corners;
    prediction.clear();
    int N;
    int i[3] = {0,0,0};
    int j[3] = {0,0,0};
    int inc_i = 0;
    int inc_j = 0;
    int h = board.h();
    int w = board.w();

    switch (dir)
    {
        case Direction::UP:
            N = w;
            i[0] = 2; i[1] = 1; i[2] = 0;
            inc_j = 1;
            break;
        case Direction::DOWN:
            N = w;
            i[0] = h - 3; i[1] = h - 2; i[2] = h - 1;
            inc_j = 1;
            break;
        case Direction::LEFT:
            N = h;
            j[0] = 2; j[1] = 1; j[2] = 0;
            inc_i = 1;
            break;
        default:
        case Direction::RIGHT:
            N = h;
            j[0] = w - 3; j[1] = w - 2; j[0] = w - 1;
            inc_i = 1;
            break;
    }

    prediction.resize(N);
    for (int ii = 0; ii < N; ++ii)
    {
        auto A = corners[board.cornerIdx[i[0]][j[0]]].pos,
             B = corners[board.cornerIdx[i[1]][j[1]]].pos,
             C = corners[board.cornerIdx[i[2]][j[2]]].pos;
        prediction[ii] = predict(A, B, C);
        for (int k = 0; k < 3; ++k)
        {
            i[k] += inc_i;
            j[k] += inc_j;
        }
    }
}

bool ChessBoardAssembler::BoardExpander::getExpandedBoard(RectangularGridPattern &board)
{
    while (growBoard());
    board = this->board;
    return true;
}

bool ChessBoardAssembler::BoardExpander::assignNearest(std::vector<corecvs::Vector2dd> &prediction, std::vector<int> &usedCorners, std::vector<int> &assignment)
{
    auto& corners = assembler->corners;
    int M = (int)prediction.size();

    if (assembler->kdtree())
    {
        assignment.resize(M);
        CORE_ASSERT_TRUE_S(assembler->kd);
        auto& tree = *assembler->kd;

        for (size_t i = 0; i < prediction.size(); ++i)
        {
            auto res = tree.nearestNeighbour(OrientedCorner(prediction[i]), [&](OrientedCorner* v) { return usedCorners[v - &corners[0]] == 0; });
            if (!res)
                return false;
            auto id = res - &corners[0];
            usedCorners[id] = 1;
            assignment[i] = id;
        }
        return true;
    }

    std::vector<int> unused;
    unused.reserve(corners.size());
    for (size_t i = 0; i < corners.size(); ++i) {
        if (!usedCorners[i]) {
            unused.push_back((int)i);
        }
    }
    int N = (int)unused.size();
    std::vector<int> assigned(M);
    assignment.resize(M);

    if (N < M)
        return false;

    std::vector<std::tuple<double, int, int>> queue;
    queue.reserve(M * N);
    for (int j = 0; j < M; ++j)
    {
        for (int i = 0; i < N; ++i)
        {
            queue.emplace_back(!(prediction[j] - corners[unused[i]].pos), i, j);
        }
    }

    int total_assigned = 0;
    int sort_by = M * 3;

    for (int ii = 0; ii < N; ++ii)
    {
        if (ii % sort_by == 0)
        {
            int from = ii;
            int to = std::max(ii + sort_by, N);
            std::partial_sort(queue.begin() + from, queue.begin() + to, queue.end(), [](const std::tuple<double, int, int> &a, const std::tuple<double, int, int> &b) { return a < b; });

        }
        auto& T = queue[ii];
        int i = unused[std::get<1>(T)], j = std::get<2>(T);
        if (usedCorners[i])
            continue;
        if (assigned[j])
            continue;
        assignment[j] = i;
        usedCorners[i] = 1;
        assigned[j] = 1;
        total_assigned++;
        if (M == (int)total_assigned)
            break;

    }
    return true;
}


void ChessBoardAssembler::setStatistics(corecvs::Statistics *stats)
{
    this->stats = stats;
}

corecvs::Statistics *ChessBoardAssembler::getStatistics()
{
    return stats;
}

double RectangularGridPattern::getStructureScore(std::vector<OrientedCorner> &corners) const
{
    double e_struct = 0.0;
    for (int i = 0; i < w(); ++i)
    {
        for (int j = 0; j + 2 < h(); ++j)
        {
            Vector2dd c1 = corners[cornerIdx[j + 0][i]].pos;
            Vector2dd c2 = corners[cornerIdx[j + 1][i]].pos;
            Vector2dd c3 = corners[cornerIdx[j + 2][i]].pos;
            double err = (c1 + c3 - 2.0 * c2).l2Metric() / (c1 - c3).l2Metric();
            if (err > e_struct)
                e_struct = err;
        }
    }

    for (int i = 0; i < h(); ++i)
    {
        for (int j = 0; j + 2 < w(); ++j)
        {
            Vector2dd c1 = corners[cornerIdx[i][j + 0]].pos;
            Vector2dd c2 = corners[cornerIdx[i][j + 1]].pos;
            Vector2dd c3 = corners[cornerIdx[i][j + 2]].pos;
            double err = (c1 + c3 - 2.0 * c2).l2Metric() / (c1 - c3).l2Metric();
            if (err > e_struct)
                e_struct = err;
        }
    }
    return w() * h() * e_struct;
}
