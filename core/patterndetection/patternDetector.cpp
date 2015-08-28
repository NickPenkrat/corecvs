#include "patternDetector.h"

bool PatternDetector::detectPattern(corecvs::RGB24Buffer &rgbBuffer)
{
    corecvs::G8Buffer grayscale(rgbBuffer.h, rgbBuffer.w);
    grayscale.binaryOperationInPlace(rgbBuffer, [](const unsigned char &a, const RGBColor &b) { return b.y(); });
    return detectPattern(grayscale);
}

void PatternDetector::getPointData(corecvs::SelectableGeometryFeatures &features)
{
    corecvs::ObservationList list;
    getPointData(list);
    features.addAllLinesFromObservationList(list);
}

void PatternDetector::drawDetected(corecvs::RGB24Buffer &buffer, bool drawSGF)
{
    if (drawSGF)
    {
        corecvs::SelectableGeometryFeatures features;
        getPointData(features);

        for (auto& p_ptr: features.mPaths)
        {
            SelectableGeometryFeatures::Vertex* prev = nullptr;
            for (auto& v_ptr: p_ptr->vertexes)
            {
                if (prev)
                {
                    corecvs::Vector2dd from = prev->position;
                    corecvs::Vector2dd to   = v_ptr->position;

                    auto diff = from - to;
                    int sx = std::abs(diff[0]) * 2 + 1;
                    int sy = std::abs(diff[1]) * 2 + 1;
                    int steps = std::max(sx, sy);

                    for (double alpha = 0.0; alpha < 1.0; alpha += 1.0 / steps)
                    {
                        auto p = from * alpha + (1.0 - alpha) * to;
                        int px = p[0] + 0.5, py = p[1] + 0.5;
                        for (int y = py - 1; y < py + 2; ++y)
                            for (int x = px - 1; x < px + 2; ++x)
                                if (std::abs(x - p[0]) < 0.5 && std::abs(y - p[1]) < 0.5 && x >= 0 && x < buffer.w && y >= 0 && y < buffer.h)
                                    buffer.element(y, x) = corecvs::RGBColor(0xff0000);
                    }
                }
                prev = v_ptr;
            }
        }
    }

    corecvs::ObservationList list;
    getPointData(list);

    for (auto& p: list)
    {
        int px = p.projection[0], py = p.projection[1];
        if (px >= 0 && px < buffer.w && py >= 0 && py < buffer.h)
            buffer.element(py, px) = corecvs::RGBColor(0x00ff00);
    }
}
