#include "global.h"

#include <vector>
#include <cassert>
#include <fstream>
#include <string>
#include <type_traits>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tbbWrapper.h"
#include "quaternion.h"
#include "homographyReconstructor.h"
#include "openCvCheckerboardDetector.h"
#include "levenmarq.h"

struct LocationData
{
    LocationData(corecvs::Vector3dd position = corecvs::Vector3dd(0.0, 0.0, 1.0), corecvs::Quaternion orientation = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0)) : position(position), orientation(orientation)
    {
    }

	corecvs::Vector3dd position;
	corecvs::Quaternion orientation;
};

struct CameraIntrinsics
{
    CameraIntrinsics(double fx = 1.0, double fy = 1.0, double cx = 0.0, double cy = 0.0, double skew = 0.0) : fx(fx), fy(fy), cx(cx), cy(cy), skew(skew)
    {
    }

	explicit operator corecvs::Matrix33() const
    {
        return corecvs::Matrix33(fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    }
	double fx, fy, cx, cy, skew;
};

struct Camera
{
	CameraIntrinsics intrinsics;
	LocationData extrinsics;
};

struct Photostation
{
	std::vector<Camera> cameras;
	LocationData location;
};

typedef std::vector<std::pair<Vector2dd, corecvs::Vector3dd>> PatternPoints3d;
typedef std::vector<PatternPoints3d> MultiCameraPatternPoints;

enum class CameraConstraints 
{
    NONE           =  0,
    ZERO_SKEW      =  1,
    LOCK_SKEW      =  2,
    EQUAL_FOCAL    =  4,
    LOCK_FOCAL     =  8,
    LOCK_PRINCIPAL = 16
};

template<typename E>
struct is_bitmask : std::false_type {};

template<typename E>
typename std::enable_if<is_bitmask<E>::value, E>::type operator& (const E &lhs, const E &rhs)
{
    typedef typename std::underlying_type<E>::type U;
    return static_cast<E>(static_cast<U>(lhs) & static_cast<U>(rhs));
}

template<typename E>
typename std::enable_if<is_bitmask<E>::value, E>::type operator| (const E &lhs, const E &rhs)
{
    typedef typename std::underlying_type<E>::type U;
    return static_cast<E>(static_cast<U>(lhs) | static_cast<U>(rhs));
}


template<typename E>
typename std::enable_if<is_bitmask<E>::value, bool>::type operator !(const E &lhs)
{
    typedef typename std::underlying_type<E>::type U;
    return !static_cast<bool>(static_cast<U>(lhs));
}

template<>
struct is_bitmask<CameraConstraints> : std::true_type {};


struct FlatPatternCalibrator
{
public:
    FlatPatternCalibrator(const CameraConstraints constraints = (CameraConstraints)0, const CameraIntrinsics lockParams = CameraIntrinsics()) : K(0), N(0), absoluteConic(6), lockParams(lockParams), constraints(constraints), forceZeroSkew(!!(constraints & CameraConstraints::ZERO_SKEW))

    {
    }

	void addPattern(const PatternPoints3d &patternPoints, const LocationData &position = LocationData())
    {
        ++N;
        locationData.push_back(position);
        points.push_back(patternPoints);
        K += patternPoints.size();
    }

	void solve(bool runPresolver = true,bool runLM = false)
    {
        if (runPresolver)
        {
            solveInitialIntrinsics();
            solveInitialExtrinsics();
            std::cout << std::endl << this << "RES NOP: " << getRmseReprojectionError() << std::endl;
            enforceParams();
        }
        std::cout << std::endl << this << "RES INIT: " << getRmseReprojectionError() << std::endl;

        if(runLM) refineGuess();
        std::cout << std::endl <<  this << "RES LM: " << getRmseReprojectionError() << std::endl;
    }

	CameraIntrinsics getIntrinsics()
    {
        return intrinsics;
    }

    std::vector<LocationData> getExtrinsics()
    {
        return locationData;
    }

    double getRmseReprojectionError()
    {
        std::vector<double> err(getOutputNum() - N);
        getFullReprojectionError(&err[0]);

        double sqs = 0.0;
        for (auto e: err) sqs += e * e;

        return sqrt(sqs / K);
    }

    void getFullReprojectionError(double out[])
    {
        corecvs::Matrix33 A = (corecvs::Matrix33)intrinsics;
        int idx = 0;

        for (size_t i = 0; i < N; ++i)
        {
            auto& R = locationData[i].orientation;
            R /= R.l2Metric();
            auto& T = locationData[i].position;
            auto& pt = points[i];
            for (auto& ptp: pt)
            {
                auto res = A * (R * (ptp.second - T));
                res /= res[2];

                auto diff = Vector2dd(res[0], res[1]) - ptp.first;

                out[idx++] = diff[0];
                out[idx++] = diff[1];
            }
        }
        assert(idx == getOutputNum() - N);
    }
private:
    size_t K, N;

    void enforceParams()
    {
#define FORCE(s, a, b) \
        if (!!(constraints & CameraConstraints::s)) intrinsics.a = b;
#define LOCK(s, a) \
        if (!!(constraints & CameraConstraints::s)) intrinsics.a = lockParams.a;

        FORCE(ZERO_SKEW, skew, 0.0);
        LOCK(LOCK_SKEW, skew);

        double f = (intrinsics.fx + intrinsics.fy) / 2.0;
        FORCE(EQUAL_FOCAL, fx, f);
        FORCE(EQUAL_FOCAL, fy, f);
        LOCK(LOCK_FOCAL, fx);
        LOCK(LOCK_FOCAL, fy);

        LOCK(LOCK_PRINCIPAL, cx);
        LOCK(LOCK_PRINCIPAL, cy);
#undef FORCE
#undef LOCK
    }

#define IFNOT(cond, expr) \
        if (!(constraints & CameraConstraints::cond)) \
        { \
            expr; \
        }
    int getInputNum() const
    {
        int input = 0;
        IFNOT(LOCK_FOCAL,
            input ++;
            IFNOT(EQUAL_FOCAL,
                input++));
        IFNOT(LOCK_PRINCIPAL, input += 2);
        IFNOT(LOCK_SKEW, IFNOT(ZERO_SKEW, input++));
        input += 7 * N;
        return input;
    }

    int getOutputNum() const
    {
        return K * 2 + N;
    }

    // Algorithm from 
    // Zhengyou Zhang A Flexible New Technique for Camera Calibration
	void solveInitialIntrinsics()
    {
        computeHomographies();
        computeAbsoluteConic();
        extractIntrinsics();
    }
	void solveInitialExtrinsics()
    {
        int n = homographies.size();

        auto A = (corecvs::Matrix33)intrinsics;
        auto Ai = A.inv();

        for (int i = 0; i < n; ++i)
        {
            auto H = homographies[i];
            corecvs::Vector3dd h1(H.a(0, 0), H.a(1, 0), H.a(2, 0));
            corecvs::Vector3dd h2(H.a(0, 1), H.a(1, 1), H.a(2, 1));
            corecvs::Vector3dd h3(H.a(0, 2), H.a(1, 2), H.a(2, 2));

            double lambda = (1.0 / !(Ai * h1) + 1.0 / !(Ai * h2)) / 2.0;

            auto T = lambda * Ai * h3;
            auto r1 = lambda * Ai * h1;
            auto r2 = lambda * Ai * h2;
            auto r3 = r1 ^ r2;

            corecvs::Matrix33 R(r1[0], r2[0], r3[0],
                       r1[1], r2[1], r3[1],
                       r1[2], r2[2], r3[2]), V;

            corecvs::Vector3dd W;
            corecvs::Matrix::svd(&R, &W, &V);
            
            corecvs::Matrix33 RO = R * V.transposed();
            auto C = -RO.transposed() * T;

            corecvs::Quaternion orientation = corecvs::Quaternion::FromMatrix(RO);
            locationData[i] = LocationData(C, orientation);
        }
    }

    void readParams(const double in[])
    {
#define GET_PARAM(ref) \
        ref = in[argin++];
#define IF_GET_PARAM(cond, ref) \
        if (!!(constraints & CameraConstraints::cond)) ref = in[argin++];
#define IFNOT_GET_PARAM(cond, ref) \
        if (!(constraints & CameraConstraints::cond)) ref = in[argin++];

        int argin = 0;
        IFNOT(LOCK_FOCAL,
            double f;
            GET_PARAM(f);
            intrinsics.fx = intrinsics.fy = f;
            IFNOT_GET_PARAM(EQUAL_FOCAL, intrinsics.fy));
        IFNOT(LOCK_PRINCIPAL,
            GET_PARAM(intrinsics.cx);
            GET_PARAM(intrinsics.cy));
        IFNOT(LOCK_SKEW,
            IFNOT_GET_PARAM(ZERO_SKEW, intrinsics.skew));

        for (size_t i = 0; i < N; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                GET_PARAM(locationData[i].position[j]);
            }
            for (int j = 0; j < 4; ++j)
            {
                GET_PARAM(locationData[i].orientation[j]);
            }
        }
        assert(argin == getInputNum());
#undef GET_PARAM
#undef IF_GET_PARAM
#undef IF_NOT_GET_PARAM
    }

#define SET_PARAM(ref) \
    out[argout++] = ref;
#define IF_SET_PARAM(cond, ref) \
    if (!!(constraints & CameraConstraints::cond)) out[argout++] = ref;
#define IFNOT_SET_PARAM(cond, ref) \
    if (!(constraints & CameraConstraints::cond)) out[argout++] = ref;

    void writeParams(double out[])
    {
        int argout = 0;
        IFNOT(LOCK_FOCAL,
            SET_PARAM(intrinsics.fx);
            IFNOT_SET_PARAM(EQUAL_FOCAL, intrinsics.fy));
        IFNOT(LOCK_PRINCIPAL,
            SET_PARAM(intrinsics.cx);
            SET_PARAM(intrinsics.cy));
        IFNOT(LOCK_SKEW,
            IFNOT_SET_PARAM(ZERO_SKEW, intrinsics.skew));

        for (size_t i = 0; i < N; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                SET_PARAM(locationData[i].position[j]);
            }
            for (int j = 0; j < 4; ++j)
            {
                SET_PARAM(locationData[i].orientation[j]);
            }
        }

        assert(argout == getInputNum());
    }
#undef SET_PARAM
#undef IF_SET_PARAM
#undef IFNOT_SET_PARAM
#undef IFNOT
    struct LMCostFunction : public corecvs::FunctionArgs
    {
        LMCostFunction(FlatPatternCalibrator *calibrator)
            : FunctionArgs(calibrator->getInputNum(), calibrator->getOutputNum()), calibrator(calibrator)
        {
        }
        void operator()(const double in[], double out[])
        {
            calibrator->readParams(in);
            calibrator->getFullReprojectionError(out);
            for (size_t i = 0; i < calibrator->N; ++i)
            {
                out[2 * calibrator->K + i] = 1.0 - calibrator->locationData[i].orientation.sumAllElementsSq();
            }
        }
        FlatPatternCalibrator *calibrator;
    };

    void refineGuess()
    {
        std::vector<double> in(getInputNum()), out(getOutputNum());
        writeParams(&in[0]);

        LevenbergMarquardt levmar(1000);
        levmar.f = new LMCostFunction(this);
        
        auto res = levmar.fit(in, out);
        readParams(&res[0]);
    }

    void computeHomographies()
    {
        homographies.clear();
        for (auto& pts: points)
        {
            if (!pts.size()) continue;

            std::vector<Vector2dd> ptsI, ptsP;

            for (auto& ptp: pts)
            {
                ptsI.push_back(ptp.first);
                ptsP.push_back(Vector2dd(ptp.second[0], ptp.second[1]));

            }

            HomographyReconstructor p2i;
            for (size_t i = 0; i < ptsI.size(); ++i)
            {
                p2i.addPoint2PointConstraint(ptsP[i], ptsI[i]);
            }

            corecvs::Matrix33 A, B;
            p2i.normalisePoints(A, B);
            auto res = p2i.getBestHomographyLSE();
            res = p2i.getBestHomographyLM(res);
            res = B.inv() * res * A;

            homographies.push_back(res);
        }
    }

    void computeAbsoluteConic()
    {
        absoluteConic = corecvs::Vector(6);
        
        int n = homographies.size();
        int n_equ = n * 2;
        if (n < 3) forceZeroSkew = true;

        if (forceZeroSkew) ++n_equ;
        int n_equ_actual = n_equ;
        if (n_equ < 6) n_equ = 6;

        corecvs::Matrix A(n_equ, 6);
        for (int i = 0; i < n_equ; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                A.a(i, j) = 0.0;
            }
        }

        for (int j = 0; j < n; ++j)
        {
            corecvs::Vector v00(6), v01(6), v11(6);
            auto& H = homographies[j];

#define V(I,J) \
            v ## I ## J[0] = H.a(0, I) * H.a(0, J); \
            v ## I ## J[1] = H.a(0, I) * H.a(1, J) + H.a(1, I) * H.a(0, J); \
            v ## I ## J[2] = H.a(1, I) * H.a(1, J); \
            v ## I ## J[3] = H.a(2, I) * H.a(0, J) + H.a(0, I) * H.a(2, J); \
            v ## I ## J[4] = H.a(2, I) * H.a(1, J) + H.a(1, I) * H.a(2, J); \
            v ## I ## J[5] = H.a(2, I) * H.a(2, J);

            V(0, 0);
            V(0, 1);
            V(1, 1);
#undef V

            auto v1 = v01;
            auto v2 = v00 - v11;

            for (int k = 0; k < 6; ++k)
            {
                A.a(2 * j, k) = v1[k];
                A.a(2 * j + 1, k) = v2[k];
            }
        }

        if (forceZeroSkew)
        {
            for (int i = 0; i < 6; ++i)
            {
                A.a(2 * n, i) = i == 1 ? 1.0 : 0.0;
            }
        }

        corecvs::Matrix V(6, 6), W(1, 6);
        corecvs::Matrix::svd(&A, &W, &V);

        double min_singular = 1e100;
        int id = -1;

        for (int j = 0; j < 6; ++j)
        {
            if (min_singular > W.a(0, j))
            {
                min_singular = W.a(0, j);
                id = j;
            }
        }

        for (int j = 0; j < 6; ++j)
        {
            absoluteConic[j] = V.a(j, id);
        }
    }

    void extractIntrinsics()
    {
        double b11, b12, b22, b13, b23, b33;
        b11 = absoluteConic[0];
        b12 = absoluteConic[1];
        b22 = absoluteConic[2];
        b13 = absoluteConic[3];
        b23 = absoluteConic[4];
        b33 = absoluteConic[5];

        double cx, cy, fx, fy, lambda, skew;
        cy = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 * b12);
        lambda = b33 - (b13 * b13 + cy * (b12 * b13 - b11 * b23)) / b11;
        fx = sqrt(lambda / b11);
        fy = sqrt(lambda * b11 / (b11 * b22 - b12 * b12));
        skew = -b12 * fx * fx * fy / lambda;
        cx = skew * cy / fx - b13 * fx * fx / lambda;

        intrinsics = CameraIntrinsics(fx, fy, cx, cy, skew);
    }

    corecvs::Vector absoluteConic;

    std::vector<corecvs::Matrix33> homographies;
	std::vector<PatternPoints3d> points;
	std::vector<LocationData> locationData;
	CameraIntrinsics intrinsics, lockParams;
	CameraConstraints constraints;
    bool forceZeroSkew;
};

struct PhotoStationCalibrator
{
public:
	void addCamera(CameraIntrinsics &intrinsics);
	void addCalibrationSetup(std::vector<int> &cameraIds, std::vector<LocationData> &cameraLocations, MultiCameraPatternPoints &points);

	void solve(bool fixIntrinsics = true);
	bool isSolvable();
	std::vector<LocationData> getCalibrationSetups();
	Photostation getPhotostation();
private:
	void solveInitialLocations();
	void refineGuess();

	std::vector<std::vector<LocationData>> locationData;
};

void usage()
{
    std::cout << "camera_calibration [N [M M_init M_increment [pattern]]]\n"
                 "  * N - number of cameras\n"
                 "  * M - number of calibration locations\n"
                 "  * M_init - initial filename postfix\n"
                 "  * M_increment - filename postfix increment\n"
                 "  * pattern - filename pattern (should have form *%d*%d*%s)" << std::endl;

}

bool parseArgs(int argc, char **argv, int &N, int &M, int &M_init, int &M_by, std::string &pattern)
{
    if (argc > 1) N = std::stoi(argv[1]);
    if (argc > 2)
    {
        if (argc < 5)
            return false;

        M      = std::stoi(argv[2]);
        M_init = std::stoi(argv[3]);
        M_by   = std::stoi(argv[4]);
    }
    if (argc > 5)
    {
        pattern = std::string(argv[5]);
    }
    return true;
}

bool checkIfExists(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);

    return (bool)ifs;
}

struct ParallelBoardDetector
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        auto& points = *pointsVector;
        for (int mc = r.begin(); mc < r.end(); ++mc)
        {
            auto& mc_vec = points[mc];
            for (int c = 0; c < N; ++c)
            {
                auto& cam_vec = mc_vec[c];
                // check if .csv exists
                //   load csv
                // else
                //   detect board
                //   produce csv
                
                char filename[1000], filename_csv[1000];
                sprintf(filename,     pattern, c, M_start + M_by * mc, "jpg");
                sprintf(filename_csv, pattern, c, M_start + M_by * mc, "csv");

                if (!checkIfExists(filename)) continue;

                if (checkIfExists(filename_csv))
                {
                    std::ifstream csv;
                    csv.open(filename_csv, std::ios_base::in);

                    cam_vec.clear();
                    do
                    {
                        double u, v, x, y, z;
                        csv >> u >> v >> x >> y >> z;
                        if ((bool)csv)
                        {
                            cam_vec.push_back(std::make_pair(Vector2dd(u, v), corecvs::Vector3dd(x, y, z)));
                        }
                    } while ((bool)csv);
                }
                else
                {
                    cv::Mat img = cv::imread(filename);

                    cam_vec = OpenCvCheckerboardDetector::GetPoints(img, 18, 11);

                    std::ofstream csv;
                    csv.open(filename_csv, std::ios_base::out);

                    for (auto p: cam_vec)
                    {
                        for (int i = 0; i < 2; ++i)
                        {
                            csv << p.first[i] << " ";
                        }
                        for (int i = 0; i < 3; ++i)
                        {
                            csv << p.second[i] << " ";
                        }
                        csv << std::endl;
                    }
                }
            }
        }
    }

    int N, M, M_start, M_by;
    const char *pattern;
    std::vector<MultiCameraPatternPoints> *pointsVector;

    ParallelBoardDetector(int N, int M, int M_start, int M_by, const char *pattern, std::vector<MultiCameraPatternPoints> *points) : N(N), M(M), M_start(M_start), M_by(M_by), pattern(pattern), pointsVector(points)
    {}
};

struct ParallelFlatPatternCalibrator
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int cam = r.begin(); cam < r.end(); ++cam)
        {
            auto& locations = (*locationsVector)[cam];
            auto& intrinsics = (*intrinsicsVector)[cam];
            auto& points = (*pointsVector);

            FlatPatternCalibrator calibrator(constraints, lockParams);
            
            for (auto& p: points)
            {
                if (p[cam].size())
                    calibrator.addPattern(p[cam]);
            }

            calibrator.solve(true, true);
//            calibrator.solve(false, true);

            locations = calibrator.getExtrinsics();
            intrinsics = calibrator.getIntrinsics();
        }
    }

    CameraConstraints constraints;
    CameraIntrinsics lockParams;
    std::vector<std::vector<LocationData>> *locationsVector;
    std::vector<CameraIntrinsics> *intrinsicsVector;
    std::vector<MultiCameraPatternPoints> *pointsVector;
    ParallelFlatPatternCalibrator(decltype(ParallelFlatPatternCalibrator::locationsVector) locations, std::vector<CameraIntrinsics> *intrinsics, decltype(pointsVector) points, CameraConstraints constraints = CameraConstraints::NONE, CameraIntrinsics lockParams = CameraIntrinsics()) : constraints(constraints), lockParams(lockParams), locationsVector(locations), intrinsicsVector(intrinsics), pointsVector(points)
    {
    }

};

void detectBoards(int N, int M, int M_start, int M_by, const char *pattern, std::vector<MultiCameraPatternPoints> &points)
{
    points.clear();
    points.resize(M);

    for (auto& v: points)
    {
        v.resize(N);
    }

    corecvs::parallelable_for (0, M, 1, ParallelBoardDetector(N, M, M_start, M_by, pattern, &points), true);
}

void calibrateCameras(int N, int M, std::vector<MultiCameraPatternPoints> &points, std::vector<CameraIntrinsics> intrinsics, std::vector<std::vector<LocationData>> &locationData)
{
    intrinsics.clear();
    intrinsics.resize(N);

    locationData.clear();
    locationData.resize(N);

    corecvs::parallelable_for (0, N, 1, ParallelFlatPatternCalibrator(&locationData, &intrinsics, &points, CameraConstraints::EQUAL_FOCAL | CameraConstraints::ZERO_SKEW | CameraConstraints::LOCK_SKEW ), true);
}

int main(int argc, char **argv)
{
    int N = 6, M = 24, M_start = 15, M_by = 15;
    std::string pattern = "distSPA%d_%ddeg.%s";
    if(!parseArgs(argc, argv, N, M, M_start, M_by, pattern))
    {
        usage();
        return 0;
    }

    std::vector<MultiCameraPatternPoints> points;
    detectBoards(N, M, M_start, M_by, pattern.c_str(), points);
    
    std::vector<CameraIntrinsics> intrinsics;
    std::vector<std::vector<LocationData>> locations;
    
    calibrateCameras(N, M, points, intrinsics, locations);

    for (auto& vc: locations)
    {
        for (auto& loc: vc)
        {
            std::cout << loc.position << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}
