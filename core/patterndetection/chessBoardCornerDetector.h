#ifndef CHESSBOARDCORNERDETECTOR
#define CHESSBOARDCORNERDETECTOR

#include <vector>

#include "abstractKernel.h"
#include "abstractBuffer.h"
#include "vector2d.h"
#include "matrix.h"

typedef corecvs::AbstractBuffer<double> DpImage;
typedef corecvs::AbstractKernel<double> DpKernel;

// Structure that holds oriented corner
struct OrientedCorner
{
    OrientedCorner(corecvs::Vector2dd pos = corecvs::Vector2dd(0, 0), corecvs::Vector2dd v1 = corecvs::Vector2dd(0, 0), corecvs::Vector2dd v2 = corecvs::Vector2dd(0, 0))
        : pos(pos), v1(v1), v2(v2), score(0.0)
    {
    }

    corecvs::Vector2dd pos;
    corecvs::Vector2dd v1, v2;
    double score;

    /*
     * Computes multi-scale score of corner based on image-intensity and gradient magnitude
     *
     * Gradient-wise score is correlation with cross of width = bandwidth oriented with corner edges
     * Intensity-wise score is correlation with corner-pattern created using edge orientations
     */
    double scoreCorner(DpImage &img, DpImage &w, std::vector<double> &radius, double bandwidth = 3.0);

    // Computes single scale score
    double scoreCorner(DpImage &img, DpImage &w, int r, double bandwidth = 3.0);

private:
    // Computes gradient magnitude correlation
    double scoreGradient(DpImage &w, int r, double bandwidth);

    // Computes correlation with corner-pattern
    double scoreIntensity(DpImage &img, int r);
};

/*
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
 */


/*
 * Set of kernels for detecting chessboard corners
 * Set consists of 4 kernels; each kernel is an sector of gaussian;
 *
 * Radius defines gaussian stdev, alpha, psi - start angle and width of sector
 */
struct CornerKernelSet
{
    DpKernel A, B, C, D;

    CornerKernelSet(double r, double alpha, double psi);

    // Computes const function for entire image
    void computeCost(DpImage &img, DpImage &c, bool parallelable = true);

private:
    // One-dimension normal distribution PDF
    // TODO: find if we already have it somewhere
    static double pdf(double x, double sigma)
    {
        return exp(-x * x/(2.0 * sigma * sigma)) / (sigma * sqrt(2.0 * M_PI));
    }
    // Initialization routine
    void computeKernels(double r, double alpha, double psi, int w, int c, double threshold = 0.05);
};

struct ChessBoardCornerDetectorParams
{
    // Width of cross for corner gradient-score
    double gradientCrossWidth = 3.0;
#if __cplusplus >= 201103L // Our compiler is cool enough to support brace-initalizer-list for structure members
    // Radius for multi-scale pattern detection
    std::vector<double> patternRadius = {4.0, 8.0, 12.0};
    // Radius for corner-scoring
    std::vector<double> cornerScores = {4.0, 8.0, 12.0};
    // Angle for rotation-variant detection
    std::vector<double> patternStartAngle = { 0.0, M_PI / 4.0 };
#else
    std::vector<double> patternRadius;
    std::vector<double> cornerScores;
    std::vector<double> patternStartAngle;

    ChessBoardCornerDetectorParams()
    {
        patternStartAngle.push_back(0.0);
        patternStartAngle.push_back(M_PI / 4.0);

        patternradius.push_back(4.0);
        patternradius.push_back(8.0);
        patternradius.push_back(12.0);
        
        cornerScores.push_back(4.0);
        cornerScores.push_back(8.0);
        cornerScores.push_back(12.0);
    }
#endif
    // Sector size
    double sectorSize = M_PI / 2.0;
    // Number of bins for computing edge direction histogram
    int histogramBins = 32;
    // Minimal angle between edges
    double minAngle = M_PI / 10.0;
    // Typical radius for estimating edge-related data and refinig corner positions
    int neighborhood = 25;
    // Gradient magnitude threshold
    double gradThreshold = 0.1;
    // Gradient orientation inlier threshold
    double orientationInlierThreshold = 0.25;
    // Threshold for distance to edge
    double inlierDistanceThreshold = 5.0;
    // Threshold for maximal corner-position update
    double updateThreshold = 4.0;
    // Threshold for final score
    // FIXME: Check if score thresholding is usable
    double scoreThreshold = 0.0;
    // Number of orientation/position refinement rounds
    int nRounds = 3;
    // Meanshift smoothing stdev
    double meanshiftBandwidth = 1.0;
};

class ChessBoardCornerDetector : ChessBoardCornerDetectorParams
{
public:
    ChessBoardCornerDetector(ChessBoardCornerDetectorParams params = ChessBoardCornerDetectorParams());
    void detectCorners(DpImage &image, std::vector<OrientedCorner> &corners);
private:
    // initalizes kernels for corner detection
    void prepareKernels();

    // scales image to 0.05 .. 0.95 quantiles
    void scaleImage();
    // first order derivative
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
    // Computes right eigen vectors and numbers for 2x2 matrix
    void eig22(corecvs::Matrix &A, double &lambda1, corecvs::Vector2dd &e1, double &lambda2, corecvs::Vector2dd &e2);
    // Checks if 2x2 matrix seems to be invertible
    bool invertable22(corecvs::Matrix &A);
    // Linear solver for 2x2 matrix
    void solve22(corecvs::Matrix &A, corecvs::Vector2dd &B, corecvs::Vector2dd &x);


    DpImage du, dv, w, phi, cost, img;
    std::vector<CornerKernelSet> kernels;
    std::vector<OrientedCorner> corners;
};
#endif
