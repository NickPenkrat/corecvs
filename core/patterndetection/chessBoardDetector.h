#ifndef CHESSBOARDDETECTOR_H
#define CHESSBOARDDETECTOR_H

#include <memory>

#include "calculationStats.h"

#include "patternDetector.h"
#include "checkerboardDetectionParameters.h"
#include "chessBoardCornerDetector.h"
#include "chessBoardAssembler.h"
#include "typesafeBitmaskEnums.h"
#include "circlePatternGenerator.h"
#include "boardAligner.h"

namespace corecvs
{

enum class ChessBoardDetectorMode
{
    BEST = 0,
    FIT_WIDTH = 1,
    FIT_HEIGHT = 2
};
template<>
struct is_bitmask<ChessBoardDetectorMode> : std::true_type {};

class ChessboardDetector : CheckerboardDetectionParameters, public PatternDetector
{
    /**
     *  Aligner manages the form of the checkerboard as well as positioning inside it
     **/
    BoardAligner *aligner;

public:
    ChessboardDetector(
            CheckerboardDetectionParameters params        = CheckerboardDetectionParameters(),
            BoardAlignerParams alignerParams              = BoardAlignerParams(),
            ChessBoardCornerDetectorParams detectorParams = ChessBoardCornerDetectorParams(),
            ChessBoardAssemblerParams assemblerParams     = ChessBoardAssemblerParams()
    );

    ~ChessboardDetector() { delete_safe(aligner); }

    static ChessBoardDetectorMode getMode(const BoardAlignerParams &params);

    // PatternDetector interface
    bool detectPattern(corecvs::G8Buffer    &buffer);
    bool detectPattern(corecvs::RGB24Buffer &buffer);
    
    using PatternDetector::getPointData;
    void getPointData(corecvs::ObservationList &observations);

    // Real pattern detection happens here
    bool detectPattern(DpImage &buffer);

    bool classify(DpImage &img, CirclePatternGenerator& generator, RGB24Buffer &buffer);

    void drawClassifier(corecvs::RGB24Buffer &buffer);

    void setStatistics(Statistics *stats);
    Statistics *getStatistics();


    size_t detectPatterns(RGB24Buffer &buffer, std::vector<ObservationList> &patterns);
    size_t detectPatterns(corecvs::DpImage     &buffer);

    void getPatterns(std::vector<ObservationList> &patterns);

    void drawCorners(RGB24Buffer &image, bool details = false);

#if 0
    void dumpState();
#endif

private:
    RectangularGridPattern                  bestPattern;
    ObservationList                         result;

    std::vector<ObservationList>            allPatterns;
    std::vector<OrientedCorner>             corners;

    ChessBoardCornerDetector                detector;
    ChessBoardAssembler                     assembler;
    std::shared_ptr<CirclePatternGenerator> sharedGenerator;

    /* Some statistics */
    Statistics  *stats;

    bool    detectPatternCandidates(DpImage &buffer, std::vector<BoardCornersType> &boards);

public:  /* We need generic interface for debug data. It could be hidden inside Statistics*/
    const ChessBoardCornerDetector *cornerDet() const   { return &detector; }

};

} // namespace corecvs

#endif // CHESSBOARDDETECTOR_H
