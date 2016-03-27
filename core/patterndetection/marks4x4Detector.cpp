#include "marks4x4Detector.h"
#include "homographyReconstructor.h"

Marks4x4Detector::Marks4x4Detector() :
    input(NULL),
    stats(NULL)
{
    result = false;
}

void Marks4x4Detector::setInput(RGB24Buffer *input)
{
    this->input = input;
}

void Marks4x4Detector::setObservations(ObservationList *observations)
{
    this->observations = observations;
}

void Marks4x4Detector::setAlignerParams(const Marks4x4DetectorParameters &params)
{
    this->parameters = params;
}

Marks4x4DetectorParameters Marks4x4Detector::getAlignerParams()
{
    return parameters;
}

bool Marks4x4Detector::operator ()()
{
    result = false;
    if (input == NULL || observations == NULL) {
        return result;
    }

    Vector2dd cellToMM(parameters.checkerboardParams.cellSizeHor(), parameters.checkerboardParams.cellSizeVert());

    HomographyReconstructor homography;
    homography.reset();

    for (PointObservation &obs : *observations) {
        homography.addPoint2PointConstraint(obs.point.xy(), obs.projection);
    }

    if (!homography.hasEnoughtConstraints())
    {
        return result;
    }

    Matrix33 transform = homography.getBestHomographyLSE();


    /* Code Translation */
    Matrix33 toCenter = Matrix33::ShiftProj(1.5, 1.5);
    Matrix33 orients[4];
    orients[0] = toCenter *                            Matrix33::ShiftProj(-2.5, 3.0) ;
    orients[1] = toCenter * Matrix33::RotationZ90()  * Matrix33::ShiftProj(-2.5, 3.0) ;
    orients[2] = toCenter * Matrix33::RotationZ180() * Matrix33::ShiftProj(-2.5, 3.0) ;
    orients[3] = toCenter * Matrix33::RotationZ270() * Matrix33::ShiftProj(-2.5, 3.0) ;


    if (debug != NULL) {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Vector2dd pos(j, i);
                pos = cellToMM *  pos;
                Vector2dd output = transform * pos;

                RGBColor color = (i == 0 && j == 0) ? RGBColor::Blue() : RGBColor::Green();

                debug->drawCrosshare1(output.x(), output.y(), color);
            }
        }
    }

    /* Draw center*/
    Vector2dd pos(0.0, 0.0);
    pos = toCenter * pos;
    pos = cellToMM * pos;
    Vector2dd output = transform * pos;
    SYNC_PRINT(("Center: %lf %lf\n", output.x(), output.y()));
    debug->drawCrosshare1(output.x(), output.y(),  RGBColor::Cyan());


    MarkerData marker[4];
    double bestScore = 0.0;
    int bestMarker = -1;

    for (unsigned o = 0; o < CORE_COUNT_OF(orients); o++)
    {

        SYNC_PRINT(("Marks4x4Detector::(): checking orientation %d...\n", o ));
        marker[o] = detectMarker(transform, Matrix33::Scale2(cellToMM) * orients[o]);
        if (marker[o].detected && marker[o].score > bestScore) {
            bestScore = marker[o].score;
            bestMarker = o;
        }

        if (debug != NULL)
        {
            for (int i = 0; i < parameters.vertBits; i++)
            {
                for (int j = 0; j < parameters.horBits; j++)
                {
                    Vector2dd pos(j + 0.5, i + 0.5);
                    pos = orients[o] * pos;
                    pos = cellToMM * pos ;
                    Vector2dd output = transform * pos;

                    RGBColor color = (i == 0 && j == 0) ? RGBColor::Blue() : RGBColor::Green();

                    if (marker[o].detected) {
                        color = marker[o].bits[i * parameters.horBits + j] ? RGBColor::Cyan() : RGBColor::Yellow();
                    }
                    debug->drawCrosshare1(output.x(), output.y(), color);
                }
            }
        }
    }



    if (bestMarker != -1) {
        result = true;
        bits = marker[bestMarker].bits;

        for (bool b : bits) {
            SYNC_PRINT(("%s ", b ? "1" : "0"));
        }
        SYNC_PRINT(("/n"));
    }
    return result;
}

Marks4x4Detector::MarkerData Marks4x4Detector::detectMarker(Matrix33 homography, Matrix33 translation)
{
    MarkerData toReturn;
    toReturn.detected = false;

    double overallMean     = 0.0;
    double overallVariance = 0.0;
    int overallSamples     = 0;


    /* First pass. Compute statistics */

    for (int i = 0; i < parameters.vertBits; i++)
    {
        for (int j = 0; j < parameters.horBits; j++)
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

            /* Add to overall stats */
            overallMean += valuesum;
            overallVariance += valuesum * valuesum;
            overallSamples++;
        }
    }

    overallMean     /= overallSamples;
    overallVariance /= overallSamples;
    overallVariance = overallVariance - overallMean * overallMean;


    SYNC_PRINT((" Marks4x4Detector::detectMarker(): mean %lf\n", overallMean ));
    /* Second pass */

    for (int i = 0; i < parameters.vertBits; i++)
    {
        for (int j = 0; j < parameters.horBits; j++)
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

            SYNC_PRINT(("    Marks4x4Detector::detectMarker(): value %lf -> (%s)\n", valuesum, (valuesum > overallMean) ? "1" : "0"));

            if (valuesum > overallMean) {
                toReturn.bits.push_back(true);
            } else {
                toReturn.bits.push_back(false);
            }
        }
    }

    SYNC_PRINT(("    Marks4x4Detector::detectMarker(): score %lf\n", overallVariance));

    toReturn.detected = true;
    toReturn.score = overallVariance;
    return toReturn;
}
