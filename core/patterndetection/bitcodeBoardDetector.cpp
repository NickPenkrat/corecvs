#include "bitcodeBoardDetector.h"
#include "homographyReconstructor.h"
#include "simpleRenderer.h"

BitcodeBoardDetector::BitcodeBoardDetector() :
    input(NULL),
    stats(NULL)
{
    result = false;
}

void BitcodeBoardDetector::setInput(RGB24Buffer *input)
{
    this->input = input;
}

void BitcodeBoardDetector::setObservations(ObservationList *observations)
{
    this->observations = observations;
}

void BitcodeBoardDetector::setParameters(const BitcodeBoardDetectorParameters &params)
{
    this->parameters = params;
}

BitcodeBoardDetectorParameters BitcodeBoardDetector::getParameters()
{
    return parameters;
}

void BitcodeBoardDetector::setStatistics(Statistics *stats)
{
    this->stats = stats;
}

Statistics *BitcodeBoardDetector::getStatistics()
{
    return stats;
}

bool BitcodeBoardDetector::operator ()()
{
    result = false;
    if (input == NULL || observations == NULL) {
        return result;
    }

    cellToMM = Vector2dd(parameters.checkerboardParams.cellSizeHor(), parameters.checkerboardParams.cellSizeVert());

    homography.reset();

    for (PointObservation &obs : *observations) {
        homography.addPoint2PointConstraint(obs.point.xy(), obs.projection);
    }

    if (!homography.hasEnoughtConstraints()) {
        return result;
    }

    transform = homography.getBestHomographyLSE();

    /* Code Translation */
    toCenter = Matrix33::ShiftProj(Vector2dd (
                            parameters.bitcodeParams.boardWidth () - 2,
                            parameters.bitcodeParams.boardHeight() - 2) / 2.0);


    Matrix33 centerToBitcode = Matrix33::ShiftProj(-parameters.bitcodeParams.boardWidth() / 2.0, parameters.bitcodeParams.boardHeight() / 2.0 + parameters.bitcodeParams.bitcodeIdentSize());

    orients[0] = toCenter *                            centerToBitcode;
    orients[1] = toCenter * Matrix33::RotationZ90()  * centerToBitcode;
    orients[2] = toCenter * Matrix33::RotationZ180() * centerToBitcode;
    orients[3] = toCenter * Matrix33::RotationZ270() * centerToBitcode;



    double bestScore = 0.0;
    bestMarker = -1;

    for (unsigned o = 0; o < CORE_COUNT_OF(orients); o++)
    {

        SYNC_PRINT(("BitcodeBoardDetector::(): checking orientation %d...\n", o ));
        marker[o] = detectMarker(transform, Matrix33::Scale2(cellToMM) * orients[o]);
        if (marker[o].detected && marker[o].score > bestScore) {
            bestScore = marker[o].score;
            bestMarker = o;
        }
    }

    if (bestMarker != -1) {
        result = true;
        bits = marker[bestMarker].bits;

        for (bool b : bits) {
            SYNC_PRINT(("%s ", b ? "1" : "0"));
        }
        SYNC_PRINT(("\n"));
    }
    return result;
}

void BitcodeBoardDetector::drawMarkerData(RGB24Buffer &buffer)
{

    int codeHeight = parameters.bitcodeParams.codeHeight();
    int codeWidth  = parameters.bitcodeParams.codeWidth ();

    for (int i = 0; i < parameters.bitcodeParams.boardHeight() - 1; i++)
    {
        for (int j = 0; j < parameters.bitcodeParams.boardWidth() - 1; j++)
        {
            Vector2dd pos(j, i);
            pos = cellToMM *  pos;
            Vector2dd output = transform * pos;

            RGBColor color = (i == 0 && j == 0) ? RGBColor::Blue() : RGBColor::Green();

            buffer.drawCrosshare1(output.x(), output.y(), color);
        }
    }

    for (unsigned o = 0; o < CORE_COUNT_OF(orients); o++)
    {
        for (int i = 0; i < codeHeight; i++)
        {
            for (int j = 0; j < codeWidth; j++)
            {
                Vector2dd pos(j + 0.5, i + 0.5);
                pos = orients[o] * pos;
                pos = cellToMM * pos;
                Vector2dd output = transform * pos;

                RGBColor color = (i == 0 && j == 0) ? RGBColor::Blue() : RGBColor::Green();

                if (marker[o].detected) {
                    color = marker[o].bits[i * codeWidth + j] ? RGBColor::Cyan() : RGBColor::Yellow();
                }
                buffer.drawCrosshare1(output.x(), output.y(), color);
            }
        }
    }

    /* Draw center*/
    Vector2dd pos(0.0, 0.0);
    pos = toCenter * pos;
    pos = cellToMM * pos;
    Vector2dd output = transform * pos;
    //SYNC_PRINT(("Center: %lf %lf\n", output.x(), output.y()));
    buffer.drawCrosshare1(output.x(), output.y(),  RGBColor::Cyan());

    if (bestMarker != -1) {
        /** Debug draw **/

        Vector2dd corners[4] = {
            Vector2dd(      0.0,        0.0),
            Vector2dd(codeWidth,        0.0),
            Vector2dd(codeWidth, codeHeight),
            Vector2dd(      0.0, codeHeight)
        };

        Vector2dd projected[4];

        for (size_t i = 0; i < CORE_COUNT_OF(corners); i++)
        {
            Vector2dd pos = orients[bestMarker] * corners[i];
            pos = cellToMM * pos;
            projected[i] = transform * pos;
        }

        for (size_t i = 0; i < CORE_COUNT_OF(corners); i++)
        {
            Vector2dd s = projected[i];
            Vector2dd e = projected[(i + 1) % CORE_COUNT_OF(corners)];
            buffer.drawLine(s.x(), s.y(), e.x(), e.y(), RGBColor::Red());
        }

    }



}

BitcodeBoardDetector::MarkerData BitcodeBoardDetector::detectMarker(Matrix33 homography, Matrix33 translation)
{
    MarkerData toReturn;
    toReturn.detected = false;

    double overallMean     = 0.0;
    double overallVariance = 0.0;
    int overallSamples     = 0;

    int codeHeight = parameters.bitcodeParams.codeHeight();
    int codeWidth  = parameters.bitcodeParams.codeWidth ();


    double delta = parameters.bitcodeParams.bitcodeConfidence();
    /* First pass. Compute statistics */

    for (int i = 0; i < codeHeight; i++)
    {
        for (int j = 0; j < codeWidth; j++)
        {

            double valuesum = 0.0;
            double valuesq  = 0.0;
            int samples = 0;


            Vector2dd corners[4] = {
                Vector2dd( j + 0.5 - delta, i + 0.5 - delta),
                Vector2dd( j + 0.5 + delta, i + 0.5 - delta),
                Vector2dd( j + 0.5 + delta, i + 0.5 + delta),
                Vector2dd( j + 0.5 - delta, i + 0.5 + delta)
            };

            Triangle2dd t1(corners[0], corners[1], corners[2]);
            Triangle2dd t2(corners[1], corners[2], corners[3]);

            TriangleSpanIterator it1(t1);
            TriangleSpanIterator it2(t2);

            /*loop over rectangle*/ {
                Vector2dd pos(j + 0.5, i + 0.5);
                Vector2dd output = homography * translation * pos;

                if (!input->isValidCoord(output.y(), output.x()))
                    return toReturn;

                double val = input->element(output.y(), output.x()).brightness();
                valuesum += val;
                valuesq  += val * val;
                samples++;
            }

            valuesum /= samples;
            valuesq /= samples;
            valuesq = valuesq - valuesum * valuesum;

            /* Add to overall stats */
            overallMean += valuesum;
            overallVariance += valuesum * valuesum;
            overallSamples++;
        }
    }

    overallMean     /= overallSamples;
    overallVariance /= overallSamples;
    overallVariance = overallVariance - overallMean * overallMean;


    SYNC_PRINT((" BitcodeBoardDetector::detectMarker(): mean %lf\n", overallMean ));
    /* Second pass */

    for (int i = 0; i < codeHeight; i++)
    {
        for (int j = 0; j < codeWidth; j++)
        {
            double valuesum = 0.0;
            double valuesq  = 0.0;
            int samples = 0;

            /*loop over rectangle*/ {
                Vector2dd pos(j + 0.5, i + 0.5);
                Vector2dd output = homography * translation * pos;

                if (!input->isValidCoord(output.y(), output.x()))
                    return toReturn;

                double val = input->element(output.y(), output.x()).brightness();
                valuesum += val;
                valuesq  += val * val;
                samples++;
            }

            valuesum /= samples;
            valuesq /= samples;
            valuesq = valuesq - valuesum * valuesum;

            if (parameters.produceDebug) {
                SYNC_PRINT(("    BitcodeBoardDetector::detectMarker(): value %lf -> (%s)\n", valuesum, (valuesum > overallMean) ? "1" : "0"));
            }

            if (valuesum > overallMean) {
                toReturn.bits.push_back(true);
            } else {
                toReturn.bits.push_back(false);
            }
        }
    }

    if (parameters.produceDebug) {
        SYNC_PRINT(("    BitcodeBoardDetector::detectMarker(): score %lf\n", overallVariance));
    }

    toReturn.detected = true;
    toReturn.score = overallVariance;
    return toReturn;
}
