#ifndef BITCODE_BOARD_DETECTOR_H
#define BITCODE_BOARD_DETECTOR_H

#include "rgb24Buffer.h"
#include "calculationStats.h"
#include "selectableGeometryFeatures.h"
#include "checkerboardDetectionParameters.h"
#include "generated/bitcodeBoardParamsBase.h"

namespace corecvs {

class BitcodeBoardDetectorParameters {
public:
    bool produceDebug;
    CheckerboardDetectionParameters checkerboardParams;
    BitcodeBoardParamsBase bitcodeParams;
};

class BitcodeBoardDetector
{
public:
    BitcodeBoardDetector();


    struct MarkerData {
        bool detected = false;
        vector<bool> bits;
        double score = 0.0;
    };


    /**
     * Inputs. This class will not own them
     **/
    RGB24Buffer *input;
    ObservationList *observations;

    /** Parameters. Owned **/
    BitcodeBoardDetectorParameters parameters;


    /* Outputs */
    bool result;
    vector<bool> bits;

    /* Misc */
    Statistics *stats;
    RGB24Buffer *debug;


    void setInput(RGB24Buffer *input);
    void setObservations(ObservationList *input);

    void setParameters(const BitcodeBoardDetectorParameters &params);
    BitcodeBoardDetectorParameters getParameters(void);


    bool operator ()();

    /* Some non standart helper functions */
private:
    MarkerData detectMarker(Matrix33 homography, Matrix33 translation);

    //drawMarker(RGB24Buffer *buffer, const MarkerData &data, orientation);


};

}


#endif // BITCODE_BOARD_DETECTOR_H
