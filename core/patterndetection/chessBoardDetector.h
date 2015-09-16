#ifndef CHESSBOARDDETECTOR
#define CHESSBOARDDETECTOR

#include "calculationStats.h"

#include "patternDetector.h"
#include "checkerboardDetectionParameters.h"
#include "chessBoardCornerDetector.h"
#include "chessBoardAssembler.h"
#include "typesafeBitmaskEnums.h"
#include "circlePatternGenerator.h"
#include "boardAligner.h"

enum class ChessBoardDetectorMode
{
    BEST = 0,
    FIT_WIDTH = 1,
    FIT_HEIGHT = 2
};
template<>
struct is_bitmask<ChessBoardDetectorMode> : std::true_type {};

class ChessboardDetector : CheckerboardDetectionParameters,
                           public PatternDetector,
                           protected BoardAligner
{
public:
    ChessboardDetector(
            CheckerboardDetectionParameters params = CheckerboardDetectionParameters(),
            BoardAlignerParams alignerParams = BoardAlignerParams(),
            ChessBoardCornerDetectorParams detectorParams = ChessBoardCornerDetectorParams(),
            ChessBoardAssemblerParams assemblerParams = ChessBoardAssemblerParams()
    );

    static ChessBoardDetectorMode getMode(const BoardAlignerParams &params);

    // PatternDetector interface
    bool detectPattern(corecvs::G8Buffer    &buffer);
    bool detectPattern(corecvs::RGB24Buffer &buffer);
    
    using PatternDetector::getPointData;
    void getPointData(corecvs::ObservationList &observations);

    // Real pattern detection happens here
    bool detectPattern(DpImage     &buffer);

    bool classify(DpImage &img, CirclePatternGenerator& generator, RGB24Buffer &buffer);

    void drawClassifier(corecvs::RGB24Buffer &buffer);

    void setStatistics(Statistics *stats);
    Statistics *getStatistics();

private:

    RectangularGridPattern bestPattern;
    corecvs::ObservationList result;
    std::vector<OrientedCorner> corners;

    ChessBoardCornerDetector detector;
    ChessBoardAssembler assembler;

/* Some statistics */
    Statistics *stats;

};

#endif
