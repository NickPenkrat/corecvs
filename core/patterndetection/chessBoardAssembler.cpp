#include "chessBoardAssembler.h"

#include <unordered_set>
#include <queue>

ChessBoardAssembler::ChessBoardAssembler(ChessBoardAssemblerParams params) : ChessBoardAssemblerParams(params)
{
}

void ChessBoardAssembler::assembleBoards(std::vector<OrientedCorner> &corners_, std::vector<std::vector<std::vector<corecvs::Vector2dd>>> &boards_)
{
    corners = corners_;
    int N = corners.size();

    corecvs::parallelable_for(0, N, ParallelBoardExpander(this));

    boards_.clear();
    std::sort(boards.begin(), boards.end(), [](const RectangularGridPattern &a, const RectangularGridPattern &b) { return a.score < b.score; });
    for (auto& b: boards)
    {
        std::vector<std::vector<corecvs::Vector2dd>> board;
        for (auto& v: b.cornerIdx)
        {
            std::vector<corecvs::Vector2dd> row;
            for (auto& c: v)
                row.push_back(corners[c].pos);
            board.push_back(row);
        }
        boards_.push_back(board);
    }
}

void ChessBoardAssembler::acceptHypothesis(RectangularGridPattern &board)
{

    bool intersects = false;
    bool best = true;
    std::vector<int> intersections;
    std::unordered_set<int> cset;
    for (auto& v: board.cornerIdx)
        for (auto& c: v)
            cset.insert(c);
    if (board.score > costThreshold)
        return;

    // We may run in parallel, so need to lock
#ifdef WITH_TBB
    tbb::mutex::scoped_lock lock(mutex);
#endif

    for (int i = 0; i < boards.size(); ++i)
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
            intersections.push_back(i);
            if (bb.score < board.score)
                best = false;
        }
        if (!best) break;
    }

    if (best)
    {
        if (intersects)
        {
            int idx = 0;
            int j = 0;
            for (int i = 0; i < boards.size(); ++i)
                if (j == intersections.size() || intersections[j] != i)
                    boards[idx++] = boards[i];
                else
                    j++;
            boards.resize(idx);
        }
        boards.push_back(board);
    }
}

ChessBoardAssembler::ParallelBoardExpander::ParallelBoardExpander(ChessBoardAssembler *assembler) : assembler(assembler)
{
}

void ChessBoardAssembler::ParallelBoardExpander::operator() (const corecvs::BlockedRange<int> &r) const
{
    BoardExpander expander(assembler);
    for (int seed = r.begin(); seed != r.end(); ++seed)
    {
        RectangularGridPattern board;
        if (!expander.initBoard(seed) || !expander.getExpandedBoard(board))
            continue;

        assembler->acceptHypothesis(board);
    }
}

ChessBoardAssembler::BoardExpander::BoardExpander(ChessBoardAssembler *assembler) : assembler(assembler)
{
}

bool ChessBoardAssembler::BoardExpander::initBoard(int seed)
{
    auto& corners = assembler->corners;
    auto varThreshold = assembler->seedThreshold;
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

    auto& from = corners[seed];

    // North neighb
    int nwse[4], corn[4];
    double d_nwse[4], d_corn[4];
    for (int i = 0; i < 4; ++i)
    {
        auto dir = ((i & 1) * 2 - 1) * (i & 2 ? from.v1 : from.v2);
        if(!getNearest(seed, dir, nwse[i], d_nwse[i]))
            return false;
        usedCorners[nwse[i]] = 1;
    }
    for (int i = 0; i < 4; ++i)
    {
        auto dir = ((i & 1) * 2 - 1) * from.v2;
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

    std::vector<double> dv[2] = {{ d_nwse[0], d_nwse[1] }, { d_nwse[2], d_nwse[3], d_corn[0], d_corn[1], d_corn[2], d_corn[3]}};
    double s[2] = {0.0}, ssq[2] = {0.0};
    for (int i = 0; i < 2; ++i)
    {
        for (auto& v: dv[i])
        {
            s[i] += v;
            ssq[i] += v * v;
        }
        int N;
        s[i] /= (N = dv[i].size());
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
    auto& corners = assembler->corners;
    auto tgPenalty = assembler->seedTgPenalty;
    auto& from = corners[from_id];

    double best_dist = 1e100;
    int best_id = -1, N = corners.size();

    for (int i = 0; i < N; ++i)
    {
        if (usedCorners[i]) continue;

        auto& to = corners[i];
        auto dirp = to.pos - from.pos;
        
        if ((dir & dirp) < 0.0)
            continue;

        auto dist_proj = dir & dirp;
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
//        RectangularGridPattern curr;
        std::vector<int> used = usedCorners;

        RectangularGridPattern dst = board;
        if(!growDir(static_cast<Direction>(i), dst, used))
            continue;
//        std::cout << "Score update: " << curr.score << " -> " << best.score << std::endl;
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
    switch(dir)
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


corecvs::Vector2dd ChessBoardAssembler::BoardExpander::predict(corecvs::Vector2dd a, corecvs::Vector2dd b, corecvs::Vector2dd c)
{
    auto conservativity = assembler->conservativity;
    auto d1 = b - a, d2 = c - b;
    auto alpha1 = atan2(d1[1], d1[0]),
         alpha2 = atan2(d2[1], d2[0]);
    auto alpha = 2.0 * alpha2 - alpha1;
    auto l1 = d1.l2Metric(), l2 = d2.l2Metric();

    auto res = c + conservativity * (2.0 * l2 - l1) * corecvs::Vector2dd(cos(alpha), sin(alpha));
    return res;
}

void ChessBoardAssembler::BoardExpander::predictor(Direction dir, std::vector<corecvs::Vector2dd> &prediction)
{
    auto& corners = assembler->corners;
    prediction.clear();
    int N, i[3] = {0}, j[3] = {0}, inc_i = 0, inc_j = 0, h = board.h(), w = board.w();
    switch(dir)
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

    std::vector<int> unused;
    for (int i = 0; i < corners.size(); ++i)
        if (!usedCorners[i])
            unused.push_back(i);

    int N = unused.size();
    int M = prediction.size();
    std::vector<int> assigned(M);
    assignment.resize(M);

    if (N < M)
        return false;


    std::priority_queue<std::tuple<double, int, int>> queue;
    for (int j = 0; j < M; ++j)
    {
        for (int i = 0; i < N; ++i)
        {
            queue.push(std::make_tuple(-(prediction[j] - corners[unused[i]].pos).l2Metric(), i, j));
        }
    }


    while (queue.size())
    {
        auto T = queue.top(); queue.pop();
        int i = unused[std::get<1>(T)], j = std::get<2>(T);
        if (usedCorners[i])
            continue;
        if (assigned[j])
            continue;
        assignment[j] = i;
        usedCorners[i] = 1;
        assigned[j] = 1;
    }
    return true;
}
