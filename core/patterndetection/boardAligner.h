#ifndef BOARDALIGNER
#define BOARDALIGNER

#include "vector2d.h"
#include "circlePatternGenerator.h"
#include "selectableGeometryFeatures.h"

enum class AlignmentType
{
    // Fit all = fit 2 dimensions regardless of orientations
    FIT_ALL,
    // Fit width = fit width (e.g. dimension along X axis)
    FIT_WIDTH,
    // Fit height = fit height (e.g. dimension along Y)
    FIT_HEIGHT,
    // Keep orientaion and select position using central marker
    FIT_MARKER_ORIENTATION,
    // Detect orientation 
    FIT_MARKERS
};

struct BoardAlignerParams
{
    AlignmentType type;
    std::vector<std::vector<corecvs::Vector2dd>> circleCenters;
    /*
     * Square poses:
     * A B
     * C D
     * Array poses:
     * A B C D
     */
    std::vector<std::array<std::pair<int, int>, 4>> markerCells;
    double circleRadius = 0.08;
    int idealWidth = 18;
    int idealHeight = 11;
};


class BoardAligner : protected BoardAlignerParams
{
public:
    BoardAligner(BoardAlignerParams params = BoardAlignerParams());
    bool align(DpImage &img);
protected:
    std::vector<std::vector<corecvs::Vector2dd>> bestBoard;
    std::vector<std::vector<std::pair<int, int>>> classifier;
    corecvs::ObservationList observationList;
private:
    bool alignDim(DpImage &img, bool fitW, bool fitH);
    bool alignSingleMarker(DpImage &img);
    bool alignMarkers(DpImage &img);
    void classify(bool trackOrientation, DpImage &img);
    void fixOrientation();
    void transpose();
    bool bfs();
    bool createList();
};

#endif
