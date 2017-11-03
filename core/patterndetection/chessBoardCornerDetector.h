#ifndef CHESSBOARD_CORNER_DETECTOR
#define CHESSBOARD_CORNER_DETECTOR

#include <vector>

#include "core/xml/generated/chessBoardCornerDetectorParamsBase.h"

#include "core/buffers/abstractKernel.h"
#include "core/buffers/abstractBuffer.h"
#include "core/math/vector/vector2d.h"
#include "core/math/matrix/matrix.h"
#include "core/reflection/reflection.h"
#include "core/buffers/convolver/convolver.h"    // corecvs::DpImage

namespace corecvs {
    class Statistics;
}

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

    double& operator[](const int r)
    {
        return pos[r];
    }
    double operator[](const int r) const
    {
        return pos[r];
    }
    /**
     * Computes multi-scale score of corner based on image-intensity and gradient magnitude
     *
     * Gradient-wise score is correlation with cross of width = bandwidth oriented with corner edges
     * Intensity-wise score is correlation with corner-pattern created using edge orientations
     **/
    double scoreCorner(DpImage &img, DpImage &w, const std::vector<double> &radius, double bandwidth = 3.0);

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
    enum {
        KERNEL_A,
        KERNEL_B,
        KERNEL_C,
        KERNEL_D,
        KERNEL_LAST
    };
    corecvs::DpKernel  K[KERNEL_LAST];
    corecvs::FpKernel fK[KERNEL_LAST];

    CornerKernelSet(double r, double alpha, double psi, bool minify = false);

    /**
     * Computes const function for entire image
     **/
    void computeCost(corecvs::DpImage &img, corecvs::DpImage &c, bool parallelable = true, bool new_style = true);
    void computeCost(corecvs::FpImage &img, corecvs::DpImage &c);



private:
    // Initialization routine
    template<class KernelType>
    static void computeKernels(KernelType K[KERNEL_LAST], double r, double alpha, double psi, int w, int c, double threshold = 0.05);

    template<class KernelType>
    static void minifyKernel(KernelType &k);
};

class ChessBoardCornerDetectorParams : public ChessBoardCornerDetectorParamsBase
{
public:
    ChessBoardCornerDetectorParams(const ChessBoardCornerDetectorParamsBase &base = ChessBoardCornerDetectorParamsBase())
        : ChessBoardCornerDetectorParamsBase(base)
    {}

    void setMinAngle(double rad)
    {
        setMinAngleDeg(corecvs::radToDeg(rad));
    }

    double minAngle() const
    {
        return corecvs::degToRad(minAngleDeg());
    }

    void setSectorSize(double rad)
    {
        setSectorSizeDeg(corecvs::radToDeg(rad));
    }

    double sectorSize() const
    {
        return corecvs::degToRad(sectorSizeDeg());
    }

    vector<double> patternStartAngle() const
    {
        vector<double> toReturn = patternStartAngleDeg();
        for (size_t i = 0; i < toReturn.size(); i++)
        {
            toReturn[i] = corecvs::degToRad(toReturn[i]);
        }
        return toReturn;
    }
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
     * Scales image to (percLo, percHigh) quantiles
     *
     * // TODO: for Indoors it requires: [0.002, 0.998]
     **/
    void scaleImage(double percLow = 0.05, double percHigh = 0.95);

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

    DpImage du, dv;
    DpImage w, phi;
    DpImage cost, img;
    std::vector<CornerKernelSet> kernels;
    std::vector<OrientedCorner>  corners;

public:
    void setStatistics(corecvs::Statistics *stats)  { this->stats = stats; }
    corecvs::Statistics *getStatistics()            { return stats; }

    std::vector<std::string> debugBuffers() const;
    corecvs::RGB24Buffer *getDebugBuffer(const std::string& name) const;

private:
    corecvs::Statistics *stats = nullptr;
};

#endif // CHESSBOARD_CORNER_DETECTOR
