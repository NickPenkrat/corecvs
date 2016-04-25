#include "bitcodeBoardDetector.h"
#include "homographyReconstructor.h"
#include "simpleRenderer.h"
#include "polygonPointIterator.h"
#include "ellipticalApproximation.h"

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

unsigned BitcodeBoardDetector::startOrientaion() {
    if (parameters.bitcodeParams.bitcodeOrientation() == BitcodeBoardOrientation::ANY)
        return 0;
    return parameters.bitcodeParams.bitcodeOrientation();
}

unsigned BitcodeBoardDetector::endOrientaion() {
    if (parameters.bitcodeParams.bitcodeOrientation() == BitcodeBoardOrientation::ANY)
        return BitcodeBoardOrientation::ANY;
    return parameters.bitcodeParams.bitcodeOrientation() + 1;
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

    transform = homography.getBestHomographyLSE2();

    /* Code Translation */
    toCenter = Matrix33::ShiftProj(Vector2dd (
                            parameters.bitcodeParams.boardWidth () - 2,
                            parameters.bitcodeParams.boardHeight() - 2) / 2.0);


    Matrix33 centerToBitcode = Matrix33::ShiftProj(-parameters.bitcodeParams.boardWidth() / 2.0, parameters.bitcodeParams.boardHeight() / 2.0 + parameters.bitcodeParams.bitcodeIdentSize());

    for (int i = 0; i < 4; i++)
    {
        orients[i] = toCenter * Matrix33::RotationZ(degToRad(90 * i)) * centerToBitcode;
    }

    double bestScore = 0.0;
    bestMarker = -1;

    for (unsigned o = startOrientaion(); o < endOrientaion(); o++)
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
        Vector2dd zeroPos = Vector2dd(
            parameters.bitcodeParams.centerToZeroX(),
            parameters.bitcodeParams.centerToZeroY());

        position = transform * Matrix33::Scale2(cellToMM) * toCenter * Matrix33::RotationZ(degToRad(90 * bestMarker)) * zeroPos;
        score = marker[bestMarker].score;
    }
    return result;
}

void BitcodeBoardDetector::drawBoardData(RGB24Buffer &buffer)
{
    Histogram whiteH(0, 255.0);
    Histogram blackH(0, 255.0);

    for (int i = -1; i < parameters.bitcodeParams.boardHeight() - 1; i++)
    {
        for (int j = -1; j < parameters.bitcodeParams.boardWidth() - 1; j++)
        {
            RGBColor white = RGBColor::Yellow();
            RGBColor black = RGBColor::Green();

            bool isWhite = (i + j) % 2;
            RGBColor color = isWhite ? white : black;
            /**/
            Polygon area = getRectImage(j,i, transform * Matrix33::Scale2(cellToMM));
            PolygonPointIterator it(area);
            for (Vector2d<int> p : it) {
                if (buffer.isValidCoord(p)) {
                    buffer.element(p) = RGBColor::lerpColor(buffer.element(p), color, 0.5);
                }
                if (input->isValidCoord(p)) {
                    if (isWhite)
                        whiteH.inc(input->element(p).brightness());
                    else
                        blackH.inc(input->element(p).brightness());

                }
            }
        }
    }

    Histogram all(0, 255.0);
    all.add(&whiteH);
    all.add(&blackH);

    int otsu = all.getOtsuThreshold();
    int whiteOk = whiteH.getIntervalSum(otsu, 255.0);
    int blackOk = blackH.getIntervalSum(0.0 ,  otsu);
    buffer.drawHistogram1024x512(&all, position.x(), position.y(), 0x1, 200, 200);
    //buffer.drawHistogram1024x512(&whiteH, position.x(), position.y() + 200, 0x1, 200, 200);
    //buffer.drawHistogram1024x512(&blackH, position.x(), position.y() + 400, 0x1, 200, 200);

    SYNC_PRINT(("BitcodeBoardDetector:: Otsu: %d white_ok %lf%% black_ok %lf%%\n",
                otsu,
                100.0 * whiteOk / whiteH.getTotalSum(),
                100.0 * blackOk / blackH.getTotalSum()));

}

void BitcodeBoardDetector::drawMarkerData(RGB24Buffer &buffer)
{

    int codeHeight = parameters.bitcodeParams.codeHeight();
    int codeWidth  = parameters.bitcodeParams.codeWidth ();

    buffer.drawCrosshare2(position.x(), position.y(), RGBColor::Violet());

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

    for (unsigned o = startOrientaion(); o < endOrientaion(); o++)
    {
        for (int i = 0; i < codeHeight; i++)
        {
            for (int j = 0; j < codeWidth; j++)
            {

                RGBColor color = (i == 0 && j == 0) ? RGBColor::Blue() : RGBColor::Green();

                if (marker[o].detected) {
                    color = marker[o].bits[i * codeWidth + j] ? RGBColor::Cyan() : RGBColor::Yellow();
                }

                Matrix33 t = transform * Matrix33::Scale2(cellToMM) * orients[o];
                Polygon area = getRectImage(j,i, t);
                PolygonPointIterator it(area);
                for (Vector2d<int> p : it) {
                    if (buffer.isValidCoord(p)) {
                        buffer.element(p) = RGBColor::lerpColor(buffer.element(p), color, 0.5);
                    }
                }
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


/* Stat collector */

Polygon BitcodeBoardDetector::getRectImage(int j, int i, Matrix33 transform)
{
    double delta = parameters.bitcodeParams.bitcodeConfidence();
    return Polygon::FromRectagle(Rectangled::SquareFromCenter(Vector2dd(j + 0.5, i + 0.5), delta)).transform(transform);
}

BitcodeBoardDetector::MarkerData BitcodeBoardDetector::detectMarker(Matrix33 homography, Matrix33 translation)
{
    MarkerData toReturn;
    toReturn.detected = false;

    SDevApproximation1d interSquare;

    int codeHeight = parameters.bitcodeParams.codeHeight();
    int codeWidth  = parameters.bitcodeParams.codeWidth ();

    vector<SDevApproximation1d> areaStats(codeHeight * codeWidth);

    /* First pass. Compute statistics */
    for (int i = 0; i < codeHeight; i++)
    {
        for (int j = 0; j < codeWidth; j++)
        {
            SDevApproximation1d areaStat;

            Polygon area = getRectImage(j,i, homography * translation);
            if (!area.isInsideBuffer(input->getSize())) {
                //SYNC_PRINT(("    BitcodeBoardDetector::detectMarker(): code not inside image\n"));
                return toReturn;
            }

            PolygonPointIterator it(area);
            for (Vector2d<int> p : it ) {
                if (!input->isValidCoord(p))
                {
                    SYNC_PRINT(("Interal error:\n"));
                    cout << area << endl;
                    cout << p << endl;
                    SYNC_PRINT(("\n"));

                }
                double val = input->element(p).brightness();
                areaStat.addPoint(val);
            }

            /* Add to overall stats */
            double mean = areaStat.getAverage();
            interSquare.addPoint(mean);

            areaStats[i * codeWidth + j] = areaStat;
        }
    }

    double overallMean = interSquare.getAverage();

    //SYNC_PRINT((" BitcodeBoardDetector::detectMarker(): mean %lf\n", overallMean ));
    /* Second pass */

    for (const SDevApproximation1d &stat : areaStats)
    {
        if (parameters.produceDebug) {
            // SYNC_PRINT(("    BitcodeBoardDetector::detectMarker(): value %lf -> (%s)\n", stat.getAverage(), (stat.getAverage() > overallMean) ? "1" : "0"));
        }

        if (stat.getAverage() > overallMean) {
            toReturn.bits.push_back(true);
        } else {
            toReturn.bits.push_back(false);
        }

    }

    if (parameters.produceDebug) {

        SYNC_PRINT(("    BitcodeBoardDetector::detectMarker():"));
        for (bool bit: toReturn.bits)
        {
            SYNC_PRINT(("%s", bit ? "1" : "0"));
        }
        SYNC_PRINT(("\n"));
        SYNC_PRINT(("    BitcodeBoardDetector::detectMarker(): score %lf\n", interSquare.getSDev()));
    }

    toReturn.detected = true;
    toReturn.score = interSquare.getSDev();
    return toReturn;
}
