#ifndef CHESSBOARDASSEMBLER
#define CHESSBOARDASSEMBLER

#include "chessBoardCornerDetector.h"

#include <vector>

#include "tbbWrapper.h"
#ifdef WITH_TBB
#include <tbb/mutex.h>
#endif

#include "generated/chessBoardAssemblerParamsBase.h"

using corecvs::Vector2dd;

/*
 * Here we implement second part (= assembling chessboard from oriented corners)
 * of algo presented in A. Geiger et. al Automatic Camera and Range Sensor Calibration using a single Shot http://www.cvlibs.net/publications/Geiger2012ICRA.pdf
 *
 * Basic explanation:
 * 1. Try to initalize 3x3 grid using point and it's 8 nearest neighborhs 
 * 2. Expand this solution till expanding leads to increasing error function
 * 3. From intersecting boards select one with the best energy
 */


class BoardAligner;

typedef std::vector<std::vector<Vector2dd>> BoardCornersType;

struct RectangularGridPattern
{

    double getScore(std::vector<OrientedCorner> &corners) const
    {
        return (score = getCornersScore(corners) + getStructureScore(corners));
    }

    double getCornersScore(std::vector<OrientedCorner> & /*corners*/ ) const
    {
        //CORE_UNUSED(corners);
        return -w() * h();
    }

    double getStructureScore(std::vector<OrientedCorner> &corners) const;

    std::vector<std::vector<int> > cornerIdx;
    mutable double score = 0.0;

    int w() const { return      cornerIdx.size() ? (int)cornerIdx[0].size() : 0; }
    int h() const { return (int)cornerIdx.size(); }
};

class ChessBoardAssemblerParams : public ChessBoardAssemblerParamsBase
{
public:
    ChessBoardAssemblerParams() {}

    ChessBoardAssemblerParams(const ChessBoardAssemblerParamsBase &base) :
        ChessBoardAssemblerParamsBase(base)
    {}

    int hypothesisDim(int id)
    {
        if (id == 0) {
            return hypothesisDimFirst();
        }

        return hypothesisDimSecond();
    }
};


class ChessBoardAssembler : ChessBoardAssemblerParams
{


public:
    ChessBoardAssembler(ChessBoardAssemblerParams params = ChessBoardAssemblerParams());
    ChessBoardAssembler(const ChessBoardAssembler &other);
    ChessBoardAssembler& operator=(const ChessBoardAssembler &other);
    void assembleBoards(std::vector<OrientedCorner> &corners_,
                        std::vector<BoardCornersType> &boards,
                        BoardAligner* aligner = 0, DpImage* buffer = 0);

protected://iivate:
    enum class Direction {UP, DOWN, LEFT, RIGHT};

    class BoardExpander
    {
        public:
            BoardExpander(ChessBoardAssembler *assembler);
            bool initBoard(int seed);
            bool getExpandedBoard(RectangularGridPattern &board);
        private:
            bool getNearest(int from, Vector2dd dir, int &res, double &dist);
            bool growBoard();
            bool growDir(Direction dir, RectangularGridPattern &dst, std::vector<int> &usedCorners);
            bool assignNearest(std::vector<Vector2dd> &prediction, std::vector<int> &usedCorners, std::vector<int> &assignment);
            void predictor(Direction dir, std::vector<Vector2dd> &prediction);
            Vector2dd predict(Vector2dd a, Vector2dd b, Vector2dd c);
            
            ChessBoardAssembler* assembler;
            std::vector<int> usedCorners;
//            std::vector<corecvs::Vector2dd> prediction;
            RectangularGridPattern board;
    };

    class ParallelBoardExpander
    {
        public:
            ParallelBoardExpander(ChessBoardAssembler *assembler);
            void operator() (const corecvs::BlockedRange<int>& r) const;
        private:
            ChessBoardAssembler *assembler;
    };
    void acceptHypothesis(RectangularGridPattern &board);
    std::vector<RectangularGridPattern> boards;   

    std::vector<OrientedCorner> corners;
    BoardAligner *aligner;
    DpImage* buffer;

#ifdef WITH_TBB
    tbb::mutex mutex;
#endif

public:
    void setStatistics(corecvs::Statistics *stats);
    corecvs::Statistics *getStatistics();


private:
    corecvs::Statistics *stats = 0;
};

#endif
