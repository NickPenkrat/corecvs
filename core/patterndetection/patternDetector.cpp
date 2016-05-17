#include "patternDetector.h"

bool PatternDetector::detectPattern(corecvs::RGB24Buffer &rgbBuffer)
{
    corecvs::G8Buffer grayscale(rgbBuffer.h, rgbBuffer.w);
    grayscale.binaryOperationInPlace(rgbBuffer, [](const unsigned char & /*a*/, const RGBColor &b) { return b.y(); });
    return detectPattern(grayscale);
}

void PatternDetector::getPointData(corecvs::SelectableGeometryFeatures &features)
{
    corecvs::ObservationList list;
    getPointData(list);
    features.addAllLinesFromObservationList(list);
}


size_t PatternDetector::detectPatterns(corecvs::RGB24Buffer &/*buffer*/, std::vector<ObservationList> &patterns)
{
    patterns.clear();
    return 0;
}
