#include <vector>
#include <cassert>
#include <string>

#include "calibrationJob.h"
#include "jsonGetter.h"

<<<<<<< HEAD
#include "flatPatternCalibrator.h"
#include "photoStationCalibrator.h"

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

void readImage(const std::string &filename, corecvs::RGB24Buffer &img)
{
    cv::Mat im = cv::imread(filename);
    im.convertTo(im, CV_64FC1, 1.0);
    img = corecvs::RGB24Buffer(im.rows, im.cols);
    for (int i = 0; i < im.rows; ++i)
    {
        for (int j = 0; j < im.cols; ++j)
        {
            img.element(i, j) = corecvs::RGBColor(im.at<double>(i, j * 3 + 2), im.at<double>(i, j * 3 + 1), im.at<double>(i, j * 3));
        }
    }
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
                        x *= (50.0 * 19.0 + 3.0) / (50.0 * 19.0);
                        if ((bool)csv)
                        {
                            cam_vec.push_back(std::make_pair(Vector2dd(u, v), corecvs::Vector3dd(x, y, z)));
                        }
                    } while ((bool)csv);
                }
                else
                {
                    RGB24Buffer img;
                    readImage(filename, img);

                    corecvs::ObservationList list;

                    CheckerboardDetectionParameters params;
                    params.setCellSizeHor (18);
                    params.setCellSizeVert(11);
                    params.setFitWidth (true );
                    params.setFitHeight(false);
                    ChessboardDetector detector(params);
                    bool found = detector.detectPattern(img);
                    if (found)
                    {
                        detector.getPointData(list);
                        cam_vec.clear();
                        for (auto& o: list)
                            cam_vec.emplace_back(o.projection, o.point);
                    }

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

            locations = calibrator.getExtrinsics();
            intrinsics = calibrator.getIntrinsics();
        }
    }

    CameraConstraints constraints;
    CameraIntrinsics_ lockParams;
    std::vector<std::vector<LocationData>> *locationsVector;
    std::vector<CameraIntrinsics_> *intrinsicsVector;
    std::vector<MultiCameraPatternPoints> *pointsVector;
    ParallelFlatPatternCalibrator(decltype(ParallelFlatPatternCalibrator::locationsVector) locations, std::vector<CameraIntrinsics_> *intrinsics, decltype(pointsVector) points, CameraConstraints constraints = CameraConstraints::NONE, CameraIntrinsics_ lockParams = CameraIntrinsics_()) : constraints(constraints), lockParams(lockParams), locationsVector(locations), intrinsicsVector(intrinsics), pointsVector(points)
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

void calibrateCameras(int N, int M, std::vector<MultiCameraPatternPoints> &points, std::vector<CameraIntrinsics_> &intrinsics, std::vector<std::vector<LocationData>> &locationData)
{
    intrinsics.clear();
    intrinsics.resize(N);

    locationData.clear();
    locationData.resize(N);

    corecvs::parallelable_for (0, N, 1, ParallelFlatPatternCalibrator(&locationData, &intrinsics, &points, CameraConstraints::ZERO_SKEW | CameraConstraints::LOCK_SKEW | CameraConstraints::EQUAL_FOCAL ), true);
}

// FIXME decide where we take image dimensions and other stuff
void drawPly(Photostation &ps, const std::string &name, int IW = 2592, int IH = 1944, int W = 18, int H = 11, double size = 50.0, double scale = 50.0)
{
    // Colorblind-safe palette
    RGBColor colors[] =
    {
        RGBColor(0x762a83u),
        RGBColor(0xaf8dc3u),
        RGBColor(0xe7d4e8u),
        RGBColor(0xd9f0d3u),
        RGBColor(0x7fbf7bu),
        RGBColor(0x1b7837u)
    };
    Mesh3D mesh;
    mesh.switchColor(true);
    

    auto cs = ps.location.position;
    auto qs = ps.location.orientation.conjugated();

    const int NSC = 9;
    Vector3dd center      = Vector3dd( 0,  0,  0), 
              center2     = Vector3dd( 0,  0,  1) * scale,
              topLeft     = Vector3dd( 0,  0,  1) * scale, 
              topRight    = Vector3dd(IW,  0,  1) * scale,
              bottomRight = Vector3dd(IW, IH,  1) * scale,
              bottomLeft  = Vector3dd( 0, IH,  1) * scale;
    Vector3dd pts[NSC * 2] =
    {
        center, center2,
        center, topLeft,
        center, topRight,
        center, bottomRight,
        center, bottomLeft,
        topLeft, topRight,
        topRight, bottomRight,
        bottomRight, bottomLeft,
        bottomLeft, topLeft,
    };
 
    int color = 0;
    for (auto& cam: ps.cameras)
    {
        auto cc = cam.extrinsics.position;
        auto qc = cam.extrinsics.orientation.conjugated();
        auto A = ((corecvs::Matrix33)cam.intrinsics).inv();

        mesh.currentColor = colors[color = (color + 1) % 6];

        for (int i = 0; i < NSC; ++i)
        {
            auto v1 = qs * (qc * (A * pts[i * 2]) + cc) + cs;
            auto v2 = qs * (qc * (A * pts[i * 2 + 1]) + cc) + cs;

            mesh.addLine(v1, v2);
        }

        auto ppv = qs * (qc * (A * Vector3dd(cam.intrinsics.cx, cam.intrinsics.cy, 1) * scale) + cc) + cs;

        mesh.addLine(ppv, qs * (qc * (A * center) + cc) + cs);
    }

    mesh.currentColor = RGBColor(~0u);
    for (int i = 0; i <= W; ++i)
    {
        Vector3dd v1(i * size,        0, 0);
        Vector3dd v2(i * size, H * size, 0);
        mesh.addLine(v1, v2);
    }
    for (int i = 0; i <= H; ++i)
    {
        Vector3dd v1(       0, i * size, 0);
        Vector3dd v2(W * size, i * size, 0);
        mesh.addLine(v1, v2);
    }

    std::ofstream meshof;
    meshof.open(name, std::ios_base::out);
    mesh.dumpPLY(meshof);
}

void calibratePhotostation(int N, int M, PhotoStationCalibrator &calibrator, std::vector<MultiCameraPatternPoints> &points, std::vector<CameraIntrinsics_> &intrinsics, std::vector<std::vector<LocationData>> &locations)
{
    for (auto& ci : intrinsics) {
        calibrator.addCamera(ci);
    }
	std::vector<int> cnt(N);
    for (auto& setup: points)
	{
		MultiCameraPatternPoints pts;
		std::vector<int> active;
		std::vector<LocationData> locs;
		for (int i = 0; i < N; ++i)
		{
			if (setup[i].size())
			{
				active.push_back(i);
				pts.push_back(setup[i]);
				locs.push_back(locations[i][cnt[i]++]);
			}
		}
		calibrator.addCalibrationSetup(active, locs, pts);
	}
	calibrator.solve(true, false);
    calibrator.recenter(); // Let us hope it'll speedup...
	calibrator.solve(false, true);
    calibrator.recenter();
}

=======
>>>>>>> official/dkorchemkin_mergestage
int main(int argc, char **argv)
{
    std::string name = "job.json";
    if (argc > 1)
        name = argv[1];
    CalibrationJob job;
    JSONGetter *get1 = new JSONGetter(name.c_str());
    get1->visit(job, "job");
    delete get1;
    auto ps = job.photostation;
    auto ss = job.calibrationSetupLocations;
    int N = ps.cameras.size();
    int M = ss.size();
    // Optical axis deviation from "vertical"
    Matrix T(N, 3);
    for (int i = 0; i < N; ++i)
    {
        Vector3dd v = ps.cameras[i].extrinsics.orientation.conjugated() * Vector3dd(0, 0, 1);
        for (int j = 0; j < 3; ++j)
        {
            T.a(i, j) = v[j];
        }
    }
    Matrix D(1, 3), V(3, 3);
    Matrix::svd(&T, &D, &V);
    double min_singular = D.a(0, 0);
    int id = 0;
    for (int i = 1; i < 3; ++i)
    {
        if (D.a(0, i) < min_singular)
        {
            min_singular = D.a(0, i);
            id = i;
        }
    }

    std::cout << "|_.Cam|_.angle|" << std::endl;
    Vector3dd m_axis(V.a(0, id), V.a(1, id), V.a(2, id));
    for (int i = 0; i < N; ++i)
    {
        std::cout << "|" << i << "|" << 180.0 * acos((ps.cameras[i].extrinsics.orientation.conjugated() * Vector3dd(0, 0, 1)) & m_axis) / M_PI - 90.0 << "|" << std::endl;
    }
    std::cout << std::endl;


    // Relative camera poses
    std::cout << "Relative camera poses" << std::endl;
	for (auto& cam: ps.cameras)
    {
        std::cout << cam.extrinsics.position << std::endl;
    }
#if 0
    for (int i = 0; i < M; ++i)
    {
        std::stringstream s;
        s << "mesh_" << M_start + i * M_by << "deg.ply";
        ps.location = ss[i];
        drawPly(ps, s.str());
    }
#endif
	// Cam2cam distances 
    std::cout << "Cam2cam distances" << std::endl;
    std::cout << "|_.Cam #1|_.Cam #2|_.Distance (mm)|" << std::endl;
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            std::cout << "|" << i << "|" << j << "|" << (ps.cameras[i].extrinsics.position - ps.cameras[j].extrinsics.position).l2Metric() << "|" << std::endl;
        }
    }
    std::cout << std::endl;

    // Camera intrinsics
    std::cout << "Final camera intrinsics" << std::endl;
    std::cout << "|_.Cam|_.fx|_.fy|_.fx/fy|_.skew|_.atan(skew/fy)|_.cx|_.cy|" << std::endl;
    for (int i = 0; i < N; ++i)
    {
        double fx = ps.cameras[i].intrinsics.fx;
        double fy = ps.cameras[i].intrinsics.fy;
        double cx = ps.cameras[i].intrinsics.cx;
        double cy = ps.cameras[i].intrinsics.cy;
        double skew=ps.cameras[i].intrinsics.skew;
        double aspect = fx / fy;
        double angle = atan(skew / fx) * 180.0 / M_PI;

        std::cout << "|" << i << "|" << fx << "|" << fy << "|" << aspect << "|" << skew << "|" << angle << "|" << cx << "|" << cy << "|" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Errors: " << std::endl;
    std::cout << "|_.Cam#|_.file|_.Distortion RMSE|_.Distortion max error|_.Calibration RMSE|_.Calibration max error|" << std::endl;
    for (auto &o: job.calibrationSetups)
    {
        for (auto &s: o)
        {
            auto& view = job.observations[s.cameraId][s.imageId];
            std::cout << "|" << s.cameraId << "|" << view.sourceFileName << "|" << view.distortionRmse << "|" << view.distortionMaxError << "|" << view.calibrationRmse << "|" << view.calibrationMaxError << "|" << std::endl;
        }
    }
    return 0;
}
