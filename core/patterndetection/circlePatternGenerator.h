#ifndef CIRCLEPATTERNGENERATOR
#define CIRCLEPATTERNGENERATOR

#include <vector>
#include <array>
#include <algorithm>

#include "abstractBuffer.h"
#include "vector2d.h"
#include "matrix33.h"

#ifndef DpImage
typedef corecvs::AbstractBuffer<double> DpImage;
#endif

struct CirclePatternGeneratorParams
{
    int patternSize = 128;
    double stdevThreshold = 0.05;
    double corrThreshold = 0.5;

};

class CirclePatternGenerator : public CirclePatternGeneratorParams
{
public:
    CirclePatternGenerator(CirclePatternGeneratorParams params = CirclePatternGeneratorParams());
    void addToken(int token, double r, const std::vector<corecvs::Vector2dd> &centers);
    int getBestToken(DpImage &query, double &score, corecvs::Matrix33 &orientation) const;
    int getBestToken(const DpImage &query, const std::array<corecvs::Vector2dd, 4> &cell, double &score, corecvs::Matrix33 &orientation, corecvs::Matrix33 &homography) const;
    std::vector<std::vector<DpImage>> patterns;
    std::vector<int> tokens;
protected:
    void studentize(DpImage &image) const;
    void studentize(DpImage &image, double &mean, double &stdev) const;
    void createFlip(DpImage &source, DpImage &destination, int rotation, bool flip);
    corecvs::Matrix33 getFlipMatrix(int rotation, bool flip) const;
};

#endif
