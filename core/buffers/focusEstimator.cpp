#include <string.h> // NULL

#include "focusEstimator.h"

namespace corecvs {

int FocusEstimator::getScore(unsigned char* data, int h, int w, int stride, int)
{
    if (data == NULL || h <= 0 || w <= 0 || stride <= 0)
        return 0;

    double score = 0;
    unsigned char* line = data + stride;

    for (int i = 1; i < h - 1; ++i, line += stride)
    {
        unsigned char* pixel = line + 1;

        for (int j = 1; j < w - 1; ++j, ++pixel)
        {
            int sobelH = pixel[-1]      - pixel[1];
            int sobelV = pixel[-stride] - pixel[stride];

            int edgeScore = sobelH * sobelH + sobelV * sobelV;
            if (edgeScore > 10) {
                score += edgeScore;
            }
        }
    }

    score /= (w - 2) * (h - 2);

    return (int)(score + 0.5);
}

} // namespace corecvs
