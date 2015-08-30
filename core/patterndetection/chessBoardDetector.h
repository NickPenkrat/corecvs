#ifndef CHESSBOARDDETECTOR
#define CHESSBOARDDETECTOR

#include "patternDetector.h"
#include "checkerboardDetectionParameters.h"
#include "chessBoardCornerDetector.h"
#include "chessBoardAssembler.h"
#include "typesafeBitmaskEnums.h"

enum class ChessBoardDetectorMode
{
    BEST = 0,
    FIT_WIDTH = 1,
    FIT_HEIGHT = 2
};

template<>
struct is_bitmask<ChessBoardDetectorMode> : std::true_type {};

#if 0
struct ChessBoardDetectorParams
{
    int w = 18;
    int h = 11;
    double stepX = 50.0;
    double stepY = 50.0;
    // XXX: Note that in case of detecting not full chessboard
    //      in order to assign correct coordinates we need some
    //      additional information, so default mode is FIT_WIDTH
    ChessBoardDetectorMode mode = ChessBoardDetectorMode::FIT_WIDTH;
};
#endif

class ChessboardDetector : CheckerboardDetectionParameters, public PatternDetector
{
public:
    ChessboardDetector(
            CheckerboardDetectionParameters params = CheckerboardDetectionParameters(),
            ChessBoardCornerDetectorParams detectorParams = ChessBoardCornerDetectorParams(),
            ChessBoardAssemblerParams assemblerParams = ChessBoardAssemblerParams()
    );

    static ChessBoardDetectorMode getMode(const CheckerboardDetectionParameters &params);

    // PatternDetector interface
    bool detectPattern(corecvs::G8Buffer    &buffer);
    bool detectPattern(corecvs::RGB24Buffer &buffer);
    
    using PatternDetector::getPointData;
    void getPointData(corecvs::ObservationList &observations);

    // Real pattern detection happens here
    bool detectPattern(DpImage     &buffer);

private:
    RectangularGridPattern bestPattern;
    corecvs::ObservationList result;
    std::vector<OrientedCorner> corners;

    ChessBoardCornerDetector detector;
    ChessBoardAssembler assembler;
};

#endif
