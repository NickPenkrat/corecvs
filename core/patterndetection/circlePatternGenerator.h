#ifndef CIRCLEPATTERNGENERATOR
#define CIRCLEPATTERNGENERATOR

#include <vector>
#include "vector2d.h"
#include "matrix33.h"
#include "chessBoardCornerDetector.h"

struct PatternOrientation
{
    double angle;
    bool flip;
};


class CirclePatternGenerator
{
public:
    CirclePatternGenerator(int w = 32);
    void addToken(int token, double r, const std::vector<corecvs::Vector2dd> &centers);
    int getBestToken(DpImage &query, double &score, corecvs::Matrix33 &orientation, double &secondBest);
    int w;
    std::vector<std::vector<DpImage>> patterns;
    std::vector<int> tokens;
protected:
    void studentize(DpImage &image);
    void studentize(DpImage &image, double &mean, double &stdev);
    void createFlip(DpImage &source, DpImage &destination, int rotation, bool flip);
    corecvs::Matrix33 getFlipMatrix(int rotation, bool flip);
};

#endif
