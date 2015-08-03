#include "global.h"

#include <QtCore/QString>
#include <QtCore/QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // imshow, waitkey

#include <string>

/// OpenCV wrapper
#include "OpenCVTools.h"

#include "bmpLoader.h"
#include "qtFileLoader.h"
#include "jsonGetter.h"
#include "jsonSetter.h"
#include "displacementBuffer.h"
#include "lmDistortionSolver.h"
#include "printerVisitor.h"
#include "openCvCheckerboardDetector.h"
#include "checkerboardDetectionParameters.h"
#include "selectableGeometryFeatures.h"

#include <vector>
#include <tuple>
#include <fstream>
#include <cassert>

#include "tbbWrapper.h"
#include "quaternion.h"

#include "homographyReconstructor.h"

struct location {
    Quaternion rotation;
    Vector3dd  position;
};

struct camera {
    double fx, fy, cx, cy;
    location extrinsic;
};

struct multi_camera {
    std::vector<camera> cameras;
    location mcamera_location;
};

std::vector<std::vector<std::vector<std::pair<Vector2dd, Vector3dd>>>> pts_reproj;
std::vector<Quaternion> ps_rotation;
std::vector<Vector3dd> ps_position;

multi_camera ps;
int N, M, K;

void init(double deg, double r, double deg2) {
    ps.cameras.resize(N);
    Vector3dd offset(0, 0, r);
    Quaternion rotation_cum(0, 0, 0, 1);
    Quaternion rotation = Quaternion::RotationX(deg);
    for(int i = 0; i < N; ++i)
    {
        ps.cameras[i].fx = ps.cameras[i].fy = 1000;
        ps.cameras[i].cx = 1296;
        ps.cameras[i].cy = 972;
        ps.cameras[i].extrinsic.rotation = rotation_cum;
        ps.cameras[i].extrinsic.position = Vector3dd(0, 0, -598) + rotation_cum * offset;
        rotation_cum = rotation_cum ^ rotation;
        std::cout << "Orientation: " << ps.cameras[i].extrinsic.position << " Rotation: " << ps.cameras[i].extrinsic.rotation << std::endl;
    }

    ps_rotation.resize(M);
    ps_position.resize(M);
    rotation_cum = Quaternion(0, 0, 0, 1);
    rotation = Quaternion::RotationX(deg2);

    for(int i = 0; i < M; ++i)
    {
        ps_rotation[i] = rotation_cum;
        rotation_cum = rotation_cum ^ rotation;
    }

}


int IW, IH;

struct ParallelBoards {
    void operator() (const corecvs::BlockedRange<size_t>& r) const
    {
        for(size_t i = r.begin(); i < r.end(); ++i) {
            pts_reproj[i].resize(N);
            char filename[1000];
            char csv_filename[1000];

            for(int j = 0; j < N; ++j) {
                sprintf(filename, "distSPA%d_%ddeg.jpg", j, i * 15);
                sprintf(csv_filename, "distSPA%d_%ddeg.csv", j, i * 15);
                std::ifstream ifs, ifsc;
                ifs.open(filename, std::ios_base::in);
                ifsc.open(csv_filename, std::ios_base::in);
                if(!ifs)
                    continue;
                std::cout << filename << std::endl;
                    cv::Mat img;
                    img = cv::imread(filename);
                    IH=img.rows*50;
                    IW = img.cols*50;
                if(!ifsc) {
                    std::cout << "COMPUTING" << std::endl;
                    std::vector<std::pair<Vector2dd, Vector3dd>> res = OpenCvCheckerboardDetector::GetPoints(img, 18, 11);
                    if(res.size()) {
                        pts_reproj[i][j] = res;
                        //                   K += res.size();
                    }
                    std::cout << filename << std::endl;
                    int len = strlen(filename);
                    for(int k = len - 1; k > 0; --k)
                        if(filename[k] == '.') {
                            filename[k + 1] = 'c';
                            filename[k + 2] = 's';
                            filename[k + 3] = 'v';
                        }
                    if(1){//res.size()) {
                        std::ofstream csvstream;
                        csvstream.open(filename, std::ios_base::out);

                        for(auto p: res) {
                            csvstream << p.first[0] << " " << p.first[1] << " " << p.second[0] << " " << p.second[1] << " " << p.second[2] << std::endl;
                        }
                    }
                } else 
                {
                    std::cout << "FROM CSV" << std::endl;
                    std::vector<std::pair<Vector2dd, Vector3dd>> res;
                    do
                    {
                        double u, v, x, y, z;
                        ifsc >> u >> v >> x >> y >> z;
                        if(ifsc) {
                            res.push_back(std::make_pair(Vector2dd(u, v), Vector3dd(x, y, z)));
                        }
                    } while(ifsc);
                    pts_reproj[i][j] = res;
                }

            }
        }
    }
};

void init_boards() {
    pts_reproj.resize(M+1);
    M = M+1;
    K = 0;
    corecvs::parallelable_for ((size_t)0, (size_t)(M), 1, ParallelBoards());
    for(int i = 0; i < M; ++i)
        for(int j = 0; j < N; ++j)
            K += pts_reproj[i][j].size();
}

void get_homographies() {
        std::vector<std::pair<Vector2dd, Vector3dd>> pairsGt;
        for(int i = 0; i < M && !pairsGt.size();  ++i)
            for(int j = 0; j < N; ++j)
                if(pts_reproj[i][j].size() == 11*18) {
                    for(auto p: pts_reproj[i][j])
                        pairsGt.push_back(p);
                    break;
                }
        for(auto& pp: pairsGt) {
            pp.first = Vector2dd(pp.second[0], pp.second[1]);
        }
    for(int i = 0; i < N; ++i) {
        std::vector<int> idx;
        for(int j = 0; j < M; ++j)
            if(pts_reproj[j][i].size())
                idx.push_back(j);
#if 0
        int mpts = 0;
        int midx = -1;
        for(int j = 0; j < idx.size(); ++j)
            if(mpts < pts_reproj[idx[j]][i].size()) {
                mpts = pts_reproj[idx[j]][i].size();
                midx = j;
            }

        if(mpts < 11 * 18) {
            std::cout << "ERROR: " << i << "th camera has no full image" << " (" << mpts << " max)" <<  std::endl;
            continue;
        }
#endif


        std::vector<Matrix33> homographies;
        for(int j = 0; j < idx.size(); ++j) {
            if(1) {// midx) {
                auto& repA = pts_reproj[idx[j]][i], repB = pairsGt;//pts_reproj[idx[midx]][i];
                int ptrA = 0, ptrB = 0;
                std::vector<Vector2dd> ptsA, ptsB;
                while(ptrA < repA.size() && ptrB < repB.size()) {
                    if(repA[ptrA].second == repB[ptrB].second) {
                        ptsA.push_back(repA[ptrA].first);
                        ptsB.push_back(repB[ptrB].first);
                        ptrA++; ptrB++;
                    } else ptrB++;
                }
                assert(ptsA.size() == repA.size());

                HomographyReconstructor b2a;
                for(int k = 0; k < ptsA.size(); ++k)
                    b2a.addPoint2PointConstraint(ptsB[k], ptsA[k]);

                corecvs::Matrix33 A, B;
                b2a.normalisePoints(A, B);
                auto res = b2a.getBestHomographyLSE();
                res = b2a.getBestHomographyLM(res);
                res = B.inv() * res * A;

                homographies.push_back(res);

                std::cout << "distSPA" << i << "_" << idx[j] * 15 << "deg.jpg -> GT:";// << std::endl << res << std::endl << std::endl;
                std::cout << "[";
                for(int ii = 0; ii < 3; ++ii) {
                    for(int jj = 0; jj < 3; ++jj)
                        std::cout << res.a(ii,jj) << (jj == 2 ? (ii == 2 ? "" : "; ") : ", ");
//                    std::cout << std::endl;
                }
                std::cout << "]" << std::endl;

            }
        }

        // Now we start next phase of zhang
        int n = homographies.size();
        Matrix m(2 * n, 6);
        for(int j = 0; j < homographies.size(); ++j) {
            Vector v00(6), v01(6), v11(6);
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

            auto v1 = (v01);
            auto v2 = (v00-v11);

            for(int k = 0; k < 6; ++k) {
                m.a(2 * j, k) = v1[k];
                m.a(2 * j + 1, k) = v2[k];
            }
        }
//        m.a(2*n,0)=m.a(2*n,2)=m.a(2*n,3)=m.a(2*n,4)=m.a(2*n,5) = 0.0;
//        m.a(2*n,1) = 1.0;

        Matrix V(6, 6), W(1, 6);
        Matrix::svd(&m, &W, &V);
        double min_singular = 1e100;
        Vector vec(6);

        for(int j = 0; j < 6; ++j) {
            if(min_singular > W.a(0,j)) {
                min_singular = W.a(0, j);
                for(int k = 0; k < 6; ++k)
                    vec[k] = V.a(k,j); //WTF?! is that correct?!
            }
        }
        double b11, b12, b22, b13, b23, b33;
        b11 = vec[0];
        b12 = vec[1];
        b22 = vec[2];
        b13 = vec[3];
        b23 = vec[4];
        b33 = vec[5];

        double cx, cy, fx, fy, lambda, skew;
        cy = (b12*b13-b11*b23)/(b11*b22-b12*b12);
        lambda = b33-(b13*b13 + cy *(b12*b13-b11*b23))/b11;
        fx = sqrt(lambda/b11);
        fy = sqrt(lambda*b11/(b11*b22-b12*b12));
        skew = -b12*fx*fx*fy/lambda;
        cx = skew*cy/fx-b13*fx*fx/lambda;

        std::cout << "SPA" << i << ": [" << fx << ", " << skew << ", " << cx << "; 0, " << fy << ", " << cy << "; " << "0, 0, 1]" << std::endl;
 

        std::ofstream of;
        char filename[1000];
        sprintf(filename, "SPA%d.m", i);
        of.open(filename, std::ios_base::out);

        std::ofstream ef, ef2, ef3;
        char filename2[1000];
        char filename3[1000];
        char filename4[1000];
        sprintf(filename2, "SPA%d.csv", i);
        sprintf(filename3, "SPA%d_ns.csv", i);
        sprintf(filename4, "SPA%d_r0.csv", i);
        ef.open(filename2, std::ios_base::out);
        ef2.open(filename3, std::ios_base::out);
        ef3.open(filename4, std::ios_base::out);


        Matrix33 A(fx, skew, cx, 0, fy, cy, 0, 0, 1);
        std::vector<Matrix> camCountors;
        std::vector<Vector3dd> camPos;
        std::vector<Vector3dd> dirs;
        for(int j = 0; j < n; ++j) {
            auto H = homographies[j];
            Vector3dd h1(H.a(0, 0), H.a(1, 0), H.a(2, 0));
            Vector3dd h2(H.a(0, 1), H.a(1, 1), H.a(2, 1));
            Vector3dd h3(H.a(0, 2), H.a(1, 2), H.a(2, 2));

            Matrix33 Ai = A.inv();
            double lambda = 1.0 / !(Ai * h1);
            auto T = lambda * Ai * h3;
            auto r1 = lambda * Ai * h1;
            auto r2 = lambda * Ai * h2;
            r1 /= !r1;
            r2 /= !r2;
            auto r3 = r1 ^ r2;
            Matrix33 R(r1[0], r2[0], r3[0], r1[1], r2[1], r3[1], r1[2], r2[2], r3[2]);
            Matrix33 RR = R;
            Matrix33 V;
            Vector3dd W;
            Matrix::svd(&R, &W, &V);

            std::cout << RR << std::endl << R * V.transposed() << std::endl << RR.transposed() * RR << std::endl << V * R.transposed() * R * V.transposed() << std::endl;

            Matrix33 RO = R * V.transposed();
            auto A2 = A;
            A2.a(0, 1) = 0.0;

            for(auto ptp: pts_reproj[idx[j]][i]) {
                auto p3 = ptp.second;
                auto res = A*(RO*p3+T);
                auto res2 = A2*(RO*p3+T);
                auto res3 = A*(R*p3+T);

                res /= res[2];
                res2 /= res2[2];
                res3 /= res3[2];
                Vector2dd r(res[0], res[1]), r2(res2[0], res2[1]), r3(res3[0], res3[1]);
                r -= ptp.first;
                r2 -= ptp.first;
                r3 -= ptp.first;
                ef << r[0] << ", " << r[1] << std::endl;
                ef2 << r2[0] << ", " << r2[1] << std::endl;
                ef3 << r3[0] << ", " << r3[1] << std::endl;
            }


            auto C  = -RO.transposed() * T;

            Matrix Aii(Ai);
            Matrix ROO(RO);
            Matrix m(3, 10);
            int pidx = 0;
#define MPUSH(x, y, z) \
            m.a(0, pidx) = x; m.a(1, pidx) = y; m.a(2, pidx) = z; pidx++;

            MPUSH( 0,  0, 50);
            MPUSH(IW,  0, 50);
            MPUSH(IW, IH, 50);
            MPUSH( 0, IH, 50);
            MPUSH( 0,  0, 50);
            MPUSH( 0,  0, 0);
            MPUSH( 0, IH, 50);
            MPUSH(IW, IH, 50);
            MPUSH( 0,  0, 0);
            MPUSH(IW,  0, 50);
            assert(pidx == 10);
            
            std::cout << "SPA-M" << "[";
            for(int ii = 0; ii < 3; ++ii)
                for(int jj = 0; jj < 10; ++jj)
                    std::cout <<  m.a(ii, jj)  << (jj == 9 ? (ii == 2 ? "" : "; ") : ", ");
            std::cout << "]" << std::endl;

            auto res = (Aii* m);
            Vector3dd eye(0, 0, 1);
            auto dir = RO.inv() * eye;
            dirs.push_back(dir);
#if 0
            for(int ii = 0; ii < 10; ++ii) {
                if(abs(res.a(ii, 2)) > 1e-7) {
                    double scale = 50/res.a(ii,2);
                    for(int jj = 0; jj < 3; ++jj)
                        res.a(ii, jj) *= scale;
                }
            }
#endif
            std::cout << "SPA-Ai" << "[";
            for(int ii = 0; ii < 3;++ii)
                for(int jj = 0; jj < 3; ++jj)
                    std::cout <<  Aii.a(ii, jj)  << (jj == 2 ? (ii == 2 ? "" : "; ") : ", ");
            std::cout << "]" << std::endl;
            std::cout << "SPA-R" << "[";
            for(int ii = 0; ii < 3;++ii)
                for(int jj = 0; jj < 10; ++jj)
                    std::cout << res.a(ii, jj)  << (jj == 9 ? (ii == 2 ? "" : "; ") : ", ");
            std::cout << "]" << std::endl;

            res = ROO.inv() * res;
            std::cout << "SPA-C" << "[";
            for(int ii = 0; ii < 3; ++ii)
                for(int jj = 0; jj < 10; ++jj)
                    std::cout << res.a(ii, jj) + C[ii]  << (jj == 9 ? (ii == 2 ? "" : "; ") : ", ");
            std::cout << "]" << std::endl;

            camCountors.push_back(res);
            camPos.push_back(C);
            std:: cout << "SPA" << i << "_" << idx[j] * 15 << "deg: " << C << std::endl;
        }
        of << "DSPA" << i << ".cams = {";
        for(int j = 0; j < camCountors.size(); ++j) {
            of << "[";
            for(int ii = 0; ii < 3; ++ii)
                for(int jj = 0; jj < 10; ++jj)
                    of << camCountors[j].a(ii, jj) + camPos[j][ii]  << (jj == 9 ? (ii == 2 ? "" : "; ") : ", ");
            of << "],";
        }
        of << "};"<< std::endl;
        of << "DSPA" << i << ".pos = ["; 
        for(int j = 0; j < camPos.size(); ++j)
            of << camPos[j] << ";" << std::endl;
        of << "];" << std::endl;

        of << "DSPA" << i << ".dir = [";
        for(int j = 0; j < dirs.size(); ++j) {
            auto v = dirs[j];
#define SQR(a) ((a)*(a))
            double norm = SQR(v[0]) + SQR(v[1]) + SQR(v[2]);
            norm = sqrt(norm);
            v /= norm;
            of << v[0] << " " << v[1] << " " << v[2] << "; ";
        }
        of << "];" << std::endl;
    }
    



    Matrix33 A(1, 2, 3, 0, 2, 3, 0, 0, 3);
    Matrix B(3, 10);
    for(int ii = 0; ii < 3; ++ii)
        for(int jj = 0; jj < 10; ++jj)
            B.a(ii, jj) = ii + jj;
    std::cout << A << std::endl << B << std::endl << (A * B) << std::endl;
    std::cout << A << std::endl << B << std::endl << (A * B).transposed() << std::endl;
}

#define SQR(a) ((a)*(a))

void input(const double args[]) {
    int arg_in = 0;
    for(int i = 0; i < N; ++i)
    {
        ps.cameras[i].fx = args[arg_in++];
        ps.cameras[i].fy = args[arg_in++];
        ps.cameras[i].cx = args[arg_in++];
        ps.cameras[i].cy = args[arg_in++];
        ps.cameras[i].extrinsic.position[0] = args[arg_in++];
        ps.cameras[i].extrinsic.position[1] = args[arg_in++];
        ps.cameras[i].extrinsic.position[2] = args[arg_in++];

        ps.cameras[i].extrinsic.rotation[0] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation[1] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation[2] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation[3] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation.normalise();
    }

    for(int i = 0; i < M; ++i)
    {
        if(i > 0) {
            for(int j = 0; j < 3; ++j)
                ps_position[i][j] = args[arg_in++];

            double err = 0.0;
            for(int j = 0; j < 4; ++j) 
                ps_rotation[i][j] = args[arg_in++]; 
        } else {
            ps_position[i] = Vector3dd(0, 0, 0);
            ps_rotation[i] = Quaternion(0, 0, 0, 1);
        }

        ps_rotation[i].normalise();
    }
    assert(arg_in == N * 11 + (M - 1) * 7);
}

void output(double args[]) {
    int arg_in = 0;
    for(int i = 0; i < N; ++i)
    {
        args[arg_in++] = ps.cameras[i].fx;
        args[arg_in++] = ps.cameras[i].fy;
        args[arg_in++] = ps.cameras[i].cx;
        args[arg_in++] = ps.cameras[i].cy;
        args[arg_in++] = ps.cameras[i].extrinsic.position[0];
        args[arg_in++] = ps.cameras[i].extrinsic.position[1];
        args[arg_in++] = ps.cameras[i].extrinsic.position[2];

        args[arg_in++] = ps.cameras[i].extrinsic.rotation[0];
        args[arg_in++] = ps.cameras[i].extrinsic.rotation[1];
        args[arg_in++] = ps.cameras[i].extrinsic.rotation[2];
        args[arg_in++] = ps.cameras[i].extrinsic.rotation[3];
    }

    for(int i = 0; i < M; ++i)
    {
        if(i != 0) {
            for(int j = 0; j < 3; ++j)
                args[arg_in++] = ps_position[i][j];

            double err = 0.0;
            for(int j = 0; j < 4; ++j) 
                args[arg_in++] =  ps_rotation[i][j];
        } else 
        {
        }
    }
    assert(arg_in == N * 11 + (M - 1) * 7);
}

double errg= 0.0;
double alpha = 60.0*M_PI/360.0;

void input_m(const double args[]) {
    int arg_in = 0;
    double f = args[arg_in++];
    double cx = args[arg_in++];
    double cy = args[arg_in++];
    double r = args[arg_in++];
    Vector3dd T;
    for(int i = 0; i < 3; ++i)
        T[i] = args[arg_in++];
    Quaternion Q;
    for(int i = 0; i < 4; ++i)
        Q[i] = args[arg_in++];
    Q.normalise();

    Vector3dd offset(0, 0, r);
    Quaternion single = Quaternion::RotationX(alpha), total(0, 0, 0, 1);




    for(int i = 0; i < N; ++i)
    {
        ps.cameras[i].fx = f;//args[arg_in++];
        ps.cameras[i].fy = f;//args[arg_in++];
        ps.cameras[i].cx = cx;//args[arg_in++];
        ps.cameras[i].cy = cy;//args[arg_in++];
        ps.cameras[i].extrinsic.position = T + Q * (total * offset);

        //        res[arg_out++] = 1.0 - (SQR(args[arg_in]) + SQR(args[arg_in + 1]) + SQR(args[arg_in + 2]) + SQR(args[arg_in + 3]));
        ps.cameras[i].extrinsic.rotation = Q^total;
        total = total ^ single;
        ps.cameras[i].extrinsic.rotation.normalise();
    }

    for(int i = 0; i < M; ++i)
    {
        for(int j = 0; j < 3; ++j)
            ps_position[i][j] = args[arg_in++];

        double err = 0.0;
        for(int j = 0; j < 4; ++j) {
            ps_rotation[i][j] = args[arg_in++];
            //            err += SQR(ps_rotation[i][j] = args[arg_in++]); 

        }
        ps_rotation[i].normalise();
    }
    assert(arg_in == 11 + M * 7);
}
void output_m(double args[]) {
    int arg_in = 0;
    //  double f = args[arg_in++];
    args[arg_in++] = ps.cameras[0].fx;
    args[arg_in++] = ps.cameras[0].cx;
    args[arg_in++] = ps.cameras[0].cy;
    args[arg_in++] = 120;
    for(int i = 0; i < 3; ++i);
    args[arg_in++] = 0;
    for(int i = 0; i < 3; ++i);
    args[arg_in++] = 0;
    args[arg_in++] = 1.0;


    for(int i = 0; i < M; ++i)
    {
        for(int j = 0; j < 3; ++j)
            args[arg_in++] = ps_position[i][j];

        double err = 0.0;
        for(int j = 0; j < 4; ++j) {
            args[arg_in++] = ps_rotation[i][j];
            //            err += SQR(ps_rotation[i][j] = args[arg_in++]); 

        }
        ps_rotation[i].normalise();
    }
    assert(arg_in == 11 + M * 7);
}

void error_m(const double args[], double res[]) {
    // Args:
    // < f cx cy tx ty tz qx qy qz qw >
    // M * < tx ty tz qx qy qz qw>
    //
    // Output:
    // K * < dx dy >
    int arg_in = 0, arg_out = 0;

    double f = args[arg_in++];
    double cx = args[arg_in++];
    double cy = args[arg_in++];
    double r = args[arg_in++];
    Vector3dd T;
    for(int i = 0; i < 3; ++i)
        T[i] = args[arg_in++];
    Quaternion Q;
    for(int i = 0; i < 4; ++i)
        Q[i] = args[arg_in++];
    Q.normalise();

    Vector3dd offset(0, 0, r);
    Quaternion single = Quaternion::RotationX(alpha), total(0, 0, 0, 1);




    for(int i = 0; i < N; ++i)
    {
        ps.cameras[i].fx = f;//args[arg_in++];
        ps.cameras[i].fy = f;//args[arg_in++];
        ps.cameras[i].cx = cx;//args[arg_in++];
        ps.cameras[i].cy = cy;//args[arg_in++];
        ps.cameras[i].extrinsic.position = T + Q * (total * offset);

        //        res[arg_out++] = 1.0 - (SQR(args[arg_in]) + SQR(args[arg_in + 1]) + SQR(args[arg_in + 2]) + SQR(args[arg_in + 3]));
        ps.cameras[i].extrinsic.rotation = Q^total;
        total = total ^ single;
        ps.cameras[i].extrinsic.rotation.normalise();
    }

    for(int i = 0; i < M; ++i)
    {
        for(int j = 0; j < 3; ++j)
            ps_position[i][j] = args[arg_in++];

        double err = 0.0;
        for(int j = 0; j < 4; ++j) {
            ps_rotation[i][j] = args[arg_in++];
            //            err += SQR(ps_rotation[i][j] = args[arg_in++]); 

        }
        ps_rotation[i].normalise();
    }
    assert(arg_in == 11 + M * 7);

    double norm = 0.0;

    //Reprojection section
    for(int i = 0; i < M; ++i)
    {
        for(int j = 0; j < N; ++j)
        {
            // Compute matricies
            Matrix33 Kc(ps.cameras[j].fx, 0.0,              ps.cameras[j].cx,
                    0.0,              ps.cameras[j].fy, ps.cameras[j].cy,
                    0.0,              0.0,              1.0);
            // Now we need to compute P, T
            Vector3dd C0 = ps.cameras[j].extrinsic.position;
            Quaternion Q0 = ps.cameras[j].extrinsic.rotation;

            Quaternion Q1 = ps_rotation[i];
            Vector3dd T1 = ps_position[i];
            // 1. Eliminate overall position
            // 2. Eliminate camera position


            for(int k = 0; k < pts_reproj[i][j].size(); ++k)
            {
                auto& ptp = pts_reproj[i][j][k];
                Vector2dd src = ptp.first, dst;
                Vector3dd prj = ptp.second;

                prj = prj - T1;
                prj = Q1.conjugated() * prj;
                prj = Q0.conjugated() * prj;
                prj = prj - C0;

                //                assert((Q1 * (Q0 * (prj + C0)) + T1 - ptp.second).norm() < 1e-5);
                prj = Kc * prj;
                prj /= prj[2];

                dst[0] = prj[0];
                dst[1] = prj[1];
                // Project
                Vector2dd diff = src - dst;
                res[arg_out++] = diff[0];
                res[arg_out++] = diff[1];
                norm += sqrt(SQR(diff[0])+SQR(diff[1]));
            }
        }
    }
    static int cnt = 0;
    errg = norm/K;
#if 1
    if((cnt++) % 1000 == 0) {
        std::cout << norm/K << std::endl;
    }
#endif
    assert(arg_out == 2*K);//N + M + 2 * K);
}

void error_f(const double args[], double res[]) {
    // Args:
    // N * < fx fy cx cy tx ty tz qx qy qz qw >
    // M * < tx ty tz qx qy qz qw>
    //
    // Output:
    // N * qnorm
    // M * qnorm
    // K * < dx dy >
    int arg_in = 0, arg_out = 0;
    for(int i = 0; i < N; ++i)
    {
        ps.cameras[i].fx = args[arg_in++];
        ps.cameras[i].fy = args[arg_in++];
        ps.cameras[i].cx = args[arg_in++];
        ps.cameras[i].cy = args[arg_in++];
        ps.cameras[i].extrinsic.position[0] = args[arg_in++];
        ps.cameras[i].extrinsic.position[1] = args[arg_in++];
        ps.cameras[i].extrinsic.position[2] = args[arg_in++];

        //        res[arg_out++] = 1.0 - (SQR(args[arg_in]) + SQR(args[arg_in + 1]) + SQR(args[arg_in + 2]) + SQR(args[arg_in + 3]));
        ps.cameras[i].extrinsic.rotation[0] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation[1] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation[2] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation[3] = args[arg_in++];
        ps.cameras[i].extrinsic.rotation.normalise();
    }

    for(int i = 0; i < M; ++i)
    {
        if(i > 0) {
            for(int j = 0; j < 3; ++j)
                ps_position[i][j] = args[arg_in++];

            double err = 0.0;
            for(int j = 0; j < 4; ++j) {
                ps_rotation[i][j] = args[arg_in++];
                err += SQR(ps_rotation[i][j]);
                //            err += SQR(ps_rotation[i][j] = args[arg_in++]); 

            }
            ps_rotation[i].normalise();
        } else {
            ps_position[i] = Vector3dd(0, 0, 0);
            ps_rotation[i] = Quaternion(0, 0, 0, 1);
        }
        //        res[arg_out++] = 1.0 - err;
    }

    double norm = 0.0;

    //Reprojection section
    for(int i = 0; i < M; ++i)
    {
        for(int j = 0; j < N; ++j)
        {
            // Compute matricies
            Matrix33 Kc(ps.cameras[j].fx, 0.0,              ps.cameras[j].cx,
                    0.0,              ps.cameras[j].fy, ps.cameras[j].cy,
                    0.0,              0.0,              1.0);
            // Now we need to compute P, T
            Vector3dd C0 = ps.cameras[j].extrinsic.position;
            Quaternion Q0 = ps.cameras[j].extrinsic.rotation;

            Quaternion Q1 = ps_rotation[i];
            Vector3dd T1 = ps_position[i];
            // 1. Eliminate overall position
            // 2. Eliminate camera position


            for(int k = 0; k < pts_reproj[i][j].size(); ++k)
            {
                auto& ptp = pts_reproj[i][j][k];
                Vector2dd src = ptp.first, dst;
                Vector3dd prj = ptp.second;

                prj = prj - T1;
                prj = Q1.conjugated() * prj;
                prj = Q0.conjugated() * prj;
                prj = prj - C0;

                //                assert((Q1 * (Q0 * (prj + C0)) + T1 - ptp.second).norm() < 1e-5);
                prj = Kc * prj;
                prj /= prj[2];

                dst[0] = prj[0];
                dst[1] = prj[1];
                // Project
                Vector2dd diff = src - dst;
                res[arg_out++] = diff[0];
                res[arg_out++] = diff[1];
                norm += sqrt(SQR(diff[0])+SQR(diff[1]));
            }
        }
    }
    static int cnt = 0;
    errg = norm/K;
#if 1
    if((cnt++) % 1000 == 0) {
        std::cout << norm/K << std::endl;
    }
#endif
    assert(arg_out == 2*K);//N + M + 2 * K);
}


struct Fun : public corecvs::FunctionArgs
{
    public:
        Fun() : FunctionArgs(N*11+(M-1)*7, 2*K) {}
        virtual void operator()(const double in[], double out[]) {
            error_f(in  , out);
        }
};
struct Fun_m : public corecvs::FunctionArgs
{
    public:
        Fun_m() : FunctionArgs(11+(M)*7, 2*K) {}
        virtual void operator()(const double in[], double out[]) {
            error_m(in  , out);
        }
};

int main() {
    N = 6;
    M = 360 / 15;
    init(-60.0*M_PI/180.0, 120.0, -15.0*M_PI/180.0);
    init_boards();

    get_homographies();
    return 0;

#if 0
    std::vector<double> in(N*11+(M-1)*7), out(2*K);
    output(&in[0]);
    LevenbergMarquardt levmar(4000);
    levmar.f = new Fun;
    std::vector<double> res = levmar.fit(in, out);
    input(&res[0]);
#else
    std::vector<double> in(11+(M)*7), out(2*K);
    output(&in[0]);
    LevenbergMarquardt levmar(4000);
    levmar.f = new Fun_m;
    std::vector<double> res = levmar.fit(in, out);
    input(&res[0]);
#endif
    for(int i = 0; i < N; ++i)
    {
        std::cout << "Position: " << ps.cameras[i].extrinsic.position << " Rotation: " << ps.cameras[i].extrinsic.rotation << " f: " << ps.cameras[i].fx << ", " << ps.cameras[i].fy << " C: " << ps.cameras[i].cx << ", " << ps.cameras[i].cy << std::endl;
    }

#if 0
    cv::Mat a = cv::imread("distSPA0_15deg.jpg");

    auto v =  OpenCvCheckerboardDetector::getPoints(a, 18, 11);
    for(auto p: v) {
        std::cout << p.first << " -> " << p.second << std::endl;
    }
#endif
    return 0;
}
