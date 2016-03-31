#ifndef CHESSBOARD_CORNER_DETECTOR
#define CHESSBOARD_CORNER_DETECTOR

#include <vector>

#include "calculationStats.h"

#include "generated/chessBoardCornerDetectorParamsBase.h"

#include "abstractKernel.h"
#include "abstractBuffer.h"
#include "vector2d.h"
#include "matrix.h"
#include "reflection.h"
#include "convolver/convolver.h"


/* TODO: Time to add this file to corecvs namespace. So far just some relaxed usings */
using corecvs::DpImage;
using corecvs::DpKernel;
using corecvs::Vector2dd;

using std::vector;

/**
 * Structure that holds oriented corner
 **/

struct OrientedCorner
{
    OrientedCorner(
            Vector2dd pos = Vector2dd::Zero(),
            Vector2dd v1  = Vector2dd::Zero(),
            Vector2dd v2  = Vector2dd::Zero())
        : pos(pos), v1(v1), v2(v2), score(0.0)
    {}

    Vector2dd pos;
    Vector2dd v1;
    Vector2dd v2;
    double score;

    /**
     * Computes multi-scale score of corner based on image-intensity and gradient magnitude
     *
     * Gradient-wise score is correlation with cross of width = bandwidth oriented with corner edges
     * Intensity-wise score is correlation with corner-pattern created using edge orientations
     **/
    double scoreCorner(DpImage &img, DpImage &w, std::vector<double> &radius, double bandwidth = 3.0);

    /**
     *  Computes single scale score
     **/
    double scoreCorner(DpImage &img, DpImage &w, int r, double bandwidth = 3.0);


private:
    /**
     * Computes gradient magnitude correlation
     **/
    double scoreGradient(DpImage &w, int r, double bandwidth);

    /**
     * Computes correlation with corner-pattern
     **/
    double scoreIntensity(DpImage &img, int r);
};

/**
 * Here we implement first part (= oriented chessboard corner detection)
 * of algo presented in A. Geiger et. al Automatic Camera and Range Sensor Calibration using a single Shot http://www.cvlibs.net/publications/Geiger2012ICRA.pdf
 *
 * Basic explanation:
 * 1. Detect corner candidates using non-maximum supression over
 *    cost function defined as max-min of responses from convoluting
 *    image with set of kernels
 * 2. Estimate orientation of detected corners using meanshift algo for
 *    finding modes in gradient distribution near the corner
 * 3. Refine position and orientation solving LSQ-like problems
 **/


/*
 * Set of kernels for detecting chessboard corners
 * Set consists of 4 kernels; each kernel is an sector of gaussian;
 *
 * Radius defines gaussian stdev, alpha, psi - start angle and width of sector
 */
struct CornerKernelSet
{
    DpKernel A, B, C, D;

    CornerKernelSet(double r, double alpha, double psi, bool minify = false);
#ifdef USE_UNSAFE_CONVOLUTOR
    void unsafeConvolutor(DpImage &image, DpKernel &kernel, DpImage &dst);
#endif

    // Computes const function for entire image
    void computeCost(DpImage &img, DpImage &c, bool parallelable = true, bool new_style = true);

private:
    // Initialization routine
    void computeKernels(double r, double alpha, double psi, int w, int c, double threshold = 0.05);
    static void MinifyKernel(DpKernel &k);
};

class ChessBoardCornerDetectorParams : public ChessBoardCornerDetectorParamsBase
{
public:
    // Radius for multi-scale pattern detection
    // vector<double> patternRadius;
    // Radius for corner-scoring
    vector<double> cornerScores;
    // Angle for rotation-variant detection
    //vector<double> patternStartAngle;

    ChessBoardCornerDetectorParams(const ChessBoardCornerDetectorParamsBase &base = ChessBoardCornerDetectorParamsBase()) :
        ChessBoardCornerDetectorParamsBase(base)
    {
/*        patternStartAngle.push_back(0.0);
        patternStartAngle.push_back(degToRad(45));*/

/*        mPatternRadius.push_back(4.0);
        mPatternRadius.push_back(8.0);
        mPatternRadius.push_back(12.0);*/

/*        cornerScores.push_back(4.0);
        cornerScores.push_back(8.0);
        cornerScores.push_back(12.0);*/
    }

    void setMinAngle(double rad)
    {
        setMinAngleDeg(radToDeg(rad));
    }

    double minAngle() {
        return degToRad(minAngleDeg());
    }

    void setSectorSize(double rad)
    {
        setSectorSizeDeg(radToDeg(rad));
    }

    double sectorSize() {
        return degToRad(sectorSizeDeg());
    }

    double patternStartAngle(int index)
    {
        return degToRad(mPatternRadius[index]);
    }

    vector<double> patternStartAngle()
    {
        vector<double> toReturn;

        for (size_t i = 0; i < mPatternRadius.size(); i++)
        {
            toReturn.push_back(degToRad(mPatternRadius[i]));
        }
        return toReturn;
    }

#if 0
    template<typename VisitorType>
    void accept(VisitorType &visitor)
    {
        ChessBoardCornerDetectorParamsBase::accept(visitor);
//        corecvs::DoubleVectorField dvf(0, 0, "patternStartAngle");
//        visitor.visit(patternStartAngle, &dvf);
//        corecvs::DoubleVectorField dvf2(0, 0, 0, "patternRadius");
//        visitor.visit(patternRadius, &dvf2);
//        corecvs::DoubleVectorField dvf3(0, 0, "cornerScores");
//        visitor.visit(cornerScores, &dvf3);

/*        visitor.visit(patternStartAngle, "patternStartAngle");
        visitor.visit(patternRadius    , "patternRadius");
        visitor.visit(cornerScores     , "cornerScores");*/

    }
#endif
};



class ChessBoardCornerDetector : ChessBoardCornerDetectorParams
{
public:
    ChessBoardCornerDetector(ChessBoardCornerDetectorParams params = ChessBoardCornerDetectorParams());
    void detectCorners(DpImage &image, std::vector<OrientedCorner> &corners);
private:

    /**
     * Initalizes kernels for corner detection
     **/
    void prepareKernels();

    /**
     * Scales image to 0.05 .. 0.95 quantiles
     **/
    void scaleImage();

    /**
     *  first order derivative
     **/
    void prepareDiff(DpImage &diff, bool du);

    // computes angle and magnitude of gradients
    void prepareAngleWeight();
    // computes multi-scale corner cost function
    void computeCost();
    // extracts corner candidates from cost
    void runNms();
    // Initial computation of edge orientations (meanshift)
    bool edgeOrientationFromGradient(int top, int bottom, int left, int right, corecvs::Vector2dd &v1, corecvs::Vector2dd &v2);
    // Selects usable corners using orientation
    void filterByOrientation();
    // Adjusts corner orientation (LSQ-orthogonality)
    void adjustCornerOrientation();
    // Adjusts corner position (LSQ for weighted gradient-orthogonality to edges)
    void adjustCornerPosition();
    // Assigns final score
    void computeScores();


    /*
     * Some math-related stuff; probably we should move it outside if it is usefull
     */
    // mean-shift mode detector
    // TODO: do we need it outside detector?!
    void circularMeanShift(std::vector<double> &values, double bandwidth, std::vector<std::pair<int, double>> &modes);

#if DEPRICATED
    // Computes right eigen vectors and numbers for 2x2 matrix
    void eig22(corecvs::Matrix22 &A, double &lambda1, corecvs::Vector2dd &e1, double &lambda2, corecvs::Vector2dd &e2, double EIGTOLERANCE = 1e-9);
    // Checks if 2x2 matrix seems to be invertible
    bool invertable22(corecvs::Matrix &A);
    // Linear solver for 2x2 matrix
    void solve22(corecvs::Matrix &A, corecvs::Vector2dd &B, corecvs::Vector2dd &x);
#endif

    DpImage du, dv, w, phi, cost, img;
    std::vector<CornerKernelSet> kernels;
    std::vector<OrientedCorner> corners;

public:
    void setStatistics(corecvs::Statistics *stats);
    corecvs::Statistics *getStatistics();

    vector<std::string> debugBuffers();
    RGB24Buffer *getDebugBuffer(std::string name);


private:
    corecvs::Statistics *stats;

};

#endif // CHESSBOARD_CORNER_DETECTOR
