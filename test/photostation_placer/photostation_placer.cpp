/*
 * This test takes photostation model from calibrationJob and arranges copies
 * of this PS using GPS coordinates
 *
 * NOTE: this one is not finalized yet and is more "playground" than "application"
 */
#if 0
#include <vector>
#include <string>
#include <sstream>
#include <regex>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <random>

#include "calibrationHelpers.h"
#include "calibrationJob.h"
#include "calibrationLocation.h"
#include "mesh3d.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

#include "imageKeyPoints.h"
#include "reconstructionStructs.h"

#include "vector3d.h"
#include "vector2d.h"
#include "bufferReaderProvider.h"

#include "openCvFileReader.h"
#include "featureMatchingPipeline.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "multicameraTriangulator.h"
#include "undirectedGraph.h"
#include "multiPhotostationScene.h"

#define POI_ONLY
#define NOUPD
#define METERS
//#define Q_ONLY
//#define CANON_PS
#define ESTIMATE_FC
#define ESTIMATE_C
#define ESTIMATE_F
//#define ESTIMATE_RP

corecvs::Vector3dd convertVector(const corecvs::Vector3dd& geodesic)
{
    auto vec = geodesic;
    std::swap(vec[0], vec[1]);
#ifndef METERS
    return vec * 1e3;
#else
    return vec;
#endif
}

corecvs::Photostation GeneratePs(double r, int cams, corecvs::CameraModel camera)
{
    double step = 2.0 * M_PI / cams;
    Photostation ps;
    corecvs::Vector3dd posFirst(0.0, 0.0, r);
    for (int i = 0; i < cams; ++i)
    {
        corecvs::Quaternion q(sin(i * step / 2.0), 0.0, 0.0, cos(i * step / 2.0));
        corecvs::CameraModel cam = camera;
        cam.extrinsics.position = q * posFirst;
        cam.extrinsics.orientation = cam.extrinsics.orientation ^ q;
        ps.cameras.push_back(cam);
    }
//    ps.cameras.resize(1);
    return ps;
}

const int NPS =
#ifdef CANON_PS
    7
#else
    5
#endif
    ;

corecvs::Vector3dd locations[NPS] =
#ifdef CANON_PS
{
#ifndef METERS
corecvs::Vector3dd(135.296, 559.462, 164.410) * 1e3,
corecvs::Vector3dd(129.253, 564.869, 164.438) * 1e3,
corecvs::Vector3dd(134.741, 566.896, 164.356) * 1e3,
corecvs::Vector3dd(141.041, 569.568, 164.368) * 1e3,
corecvs::Vector3dd(138.042, 575.810, 164.375) * 1e3,
corecvs::Vector3dd(149.297, 579.770, 164.415) * 1e3,
corecvs::Vector3dd(153.354, 592.804, 164.408) * 1e3
#else
corecvs::Vector3dd(135.296, 559.462, 164.410),
corecvs::Vector3dd(129.253, 564.869, 164.438),
corecvs::Vector3dd(134.741, 566.896, 164.356),
corecvs::Vector3dd(141.041, 569.568, 164.368),
corecvs::Vector3dd(138.042, 575.810, 164.375),
corecvs::Vector3dd(149.297, 579.770, 164.415),
corecvs::Vector3dd(153.354, 592.804, 164.408)
#endif
};
#else
{
corecvs::Vector3dd(124.318, 564.418, 164.525),
corecvs::Vector3dd(135.331, 556.671, 164.553),
corecvs::Vector3dd(135.066, 568.676, 164.390),
corecvs::Vector3dd(134.149, 578.438, 164.433),
corecvs::Vector3dd(144.525, 570.309, 164.504)
};
#endif
corecvs::Quaternion orientation = corecvs::Quaternion(0.5, 0.5, 0.5, 0.5);

corecvs::CameraModel GenerateCamera(double F, double iW, double iH, bool imageFlip)
{
    corecvs::PinholeCameraIntrinsics in;
    in.focal = corecvs::Vector2dd(F, F);
    in.principal = corecvs::Vector2dd(iW / 2.0, iH / 2.0);
    in.size = corecvs::Vector2dd(iW, iH);
    in.skew = 0.0;
    corecvs::LocationData ex;
    ex.position = corecvs::Vector3dd(0.0, 0.0, 0.0);
    if (imageFlip)
    {
        ex.orientation = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    else
    {
        ex.orientation = corecvs::Quaternion(0.0, 0.0, sin(-M_PI / 4.0), cos(-M_PI / 4.0));
    }
    return CameraModel(in, ex); 
}

corecvs::Photostation GenerateModelPs()
{
#ifdef CANON_PS
    double F = 2250.0;
    double iW = 2896;
    double iH = 1944;
    
    auto cam = GenerateCamera(F, iW, iH, false);
    return GeneratePs(0.0, 24, cam);
#else
    double F = 1720;
    double iW = 2592;
    double iH = 1944;

    auto cam = GenerateCamera(F, iW, iH, true);
#ifndef METERS
    return GeneratePs(120.0, 6, cam);
#else
    return GeneratePs(0.12, 6, cam);
#endif
#endif
}

double angleOffset[] =
{
    atan((1150.0-1060.0)/1700.0),
    atan((1250.0-1187.0)/1700.0),
    atan((1250.0-1044.0)/1700.0),
    atan((1250.0-715.0)/1700.0),
    atan((1250.0-1021.0)/1700.0)
};


struct ReconstructionParameters
{
    DetectorType   detector   = "SURF";
    DescriptorType descriptor = "SURF";
    MatcherType    matcher    = "ANN";//"ANN";

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(detector,   "SURF", "detector");
        visitor.visit(descriptor, "SURF", "descriptor");
        visitor.visit(matcher,    "ANN",  "matcher");
    }
};

struct ReconstructionJob : ReconstructionParameters
{
    CalibrationJob         calibrationData;
    MultiPhotostationScene scene;

    int getOutputNum() const
    {
        std::vector<double> err;
        scene.computeReprojectionErrors(err);
        return err.size();
    }

    int getInputNum() const
    {
        int cams = 0;
#ifndef POI_ONLY
        return 7 * scene.photostations.size() - 6;
#else
        return 
#ifndef Q_ONLY
            7 
#else
            4
#endif
            * scene.photostations.size()
#ifdef ESTIMATE_FC
#ifdef ESTIMATE_F
            + 6
#endif
#ifdef ESTIMATE_C
            + 2 * 6
#endif
#endif
#ifdef ESTIMATE_RP
            + 7 * 5
#endif
            ;
#endif
    }
    void writeParams(double out[], corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0)) const
    {
        int N = scene.photostations.size();
        int argout = 0;
        for (int i = 0; i < N; ++i)
        {
            auto loc = scene.photostations[i].location;
            loc.orientation = loc.orientation.normalised();

#ifndef POI_ONLY
            if (i > 1)
#else
            if (1)
#endif
            {
#ifndef Q_ONLY
                for (int j = 0; j < 3; ++j)
                    out[argout++] = (loc.position[j] - mean[j]) / scale[j];
#endif
            }
            for (int j = 0; j < 4; ++j)
                out[argout++] = loc.orientation[j];
        }
#ifdef POI_ONLY
        for (int i = 0; i < 6; ++i)
        {
#ifdef ESTIMATE_FC
#ifdef ESTIMATE_F
            out[argout++] = scene.photostations[0].cameras[i].intrinsics.focal.x();
#endif
#ifdef ESTIMATE_C
            out[argout++] = scene.photostations[0].cameras[i].intrinsics.principal.x();
            out[argout++] = scene.photostations[0].cameras[i].intrinsics.principal.y();
#endif
#endif
#ifdef ESTIMATE_RP
            if (i > 0)
            {
            for (int j = 0; j < 3; ++j)
               out[argout++] = scene.photostations[0].cameras[i].extrinsics.position[j];
            for (int j = 0; j < 4; ++j)
               out[argout++] = scene.photostations[0].cameras[i].extrinsics.orientation[j];
            }
#endif

        }
#endif
        assert(argout == getInputNum());
    }

    void getScaler(corecvs::Vector3dd &mean, corecvs::Vector3dd &scale)
    {
        int N = scene.photostations.size();
        int argin = 0;
        mean = corecvs::Vector3dd(0.0, 0.0, 0.0);
        scale = corecvs::Vector3dd(0.0, 0.0, 0.0);
        for (int i = 0; i < N; ++i)
        {
            auto& loc = scene.photostations[i].location.position;
            mean += loc;
            scale += loc * loc;
        }
        mean /= N;
        scale = scale / N - mean * mean;
        if (!scale < 1e-6)
            scale = corecvs::Vector3dd(1.0, 1.0, 1.0);
        for (int j = 0; j < 3; ++j)
            scale[j] = std::sqrt(scale[j]);
    }

    void readParams(const double in[], corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0))
    {
        int N = scene.photostations.size();
        int argin = 0;
        for (int i = 0; i < N; ++i)
        {
            auto& loc = scene.photostations[i].location;

#ifndef POI_ONLY
            if (i > 1)
#else
            if (1)
#endif
            {
#ifndef Q_ONLY
//                std::cout << "MSX: " << in[argin] << ", " << in[argin + 1] << ", " << in[argin + 2] << std::endl;
                for (int j = 0; j < 3; ++j)
                    loc.position[j] = in[argin++] * scale[j] + mean[j];
#endif
            }
            for (int j = 0; j < 4; ++j)
                loc.orientation[j] = in[argin++];
            loc.orientation = loc.orientation.normalised();
        }
#ifdef POI_ONLY
        for (int kk = 0; kk < 6; ++kk)
        {
#ifdef ESTIMATE_FC
#ifdef ESTIMATE_F
            double f = in[argin++];
#endif
#ifdef ESTIMATE_C
            double cx = in[argin++];
            double cy = in[argin++];
#endif
            for (int i = 0; i < 5; ++i)
            {
#ifdef ESTIMATE_F
                scene.photostations[i].cameras[kk].intrinsics.focal = corecvs::Vector2dd(f, f);
#endif
#ifdef ESTIMATE_C
                scene.photostations[i].cameras[kk].intrinsics.principal = corecvs::Vector2dd(cx, cy);
#endif
            }
#endif
#ifdef ESTIMATE_RP
            corecvs::Vector3dd pos;
            corecvs::Quaternion orientation;
            if (kk > 0){
            for (int i = 0; i < 3; ++i)
                pos[i] = in[argin++];
            for (int j = 0; j < 4; ++j)
                orientation[j] = in[argin++];
            orientation.normalise();
            for (int i = 0; i < 5; ++i)
            {
             scene.photostations[i].cameras[kk].extrinsics.position = pos;
             scene.photostations[i].cameras[kk].extrinsics.orientation = orientation;
            }
            }
#endif
        }
#endif
        assert(argin == getInputNum());
    }
#if 1
    struct OptimizationFunctor : public corecvs::FunctionArgs
    {
    private:
        ReconstructionJob* rJob;
        bool saturated;
        double saturationThreshold;
        corecvs::Vector3dd mean, scale;
//        corecvs::Matrix22 covariation;
#if 0
        struct ParallelJacobianEvaluator
        {
            void operator() (const corecvs::BlockedRange<int> &r) const
            {
                for (int j = r.begin(); j < r.end(); ++j)
                {
                    for (int i = 0; i < inputCnt; ++i)
                    {
                        int  psId = i > 7 ? 2 + (i - 8) / 7 : i / 4;
                        int  inPs = i > 7 ? (i - 8) % 7 : i % 4;
                        bool quat = psId < 2 ? true : inPs < 4;
                        int  inVec = psId < 2 ? inPs : quat ? inPs - 4 : inPs;

                        auto& obs = rJob->scene.pointObservations[j];
                        bool isNotUsed = true;
                        for (auto& p: obs.projections)
                        {
                            if (p.photostationId == psId)
                            {
                                isNotUsed = false;
                                break;
                            }
                        }

                        if (isNotUsed) continue;
                        auto ps_copy = rJob->scene.photostations[psId];
                        if (quat)
                        {
                            ps_copy.location.orientation[inVec] += delta;
                            ps_copy.location.orientation.normalise();
                        }
                        else
                        {
                            ps_copy.location.position[inVec] += delta;
                        }

                        std::vector<std::pair<corecvs::Matrix44, corecvs::Vector2dd>> pairs;
                        for (auto& p: obs.projections)
                        {
                            if (p.photostationId == psId)
                            {
                                pairs.emplace_back(
                                        ps_copy.getKMatrix(p.cameraId),
                                        p.projection);
                            }
                            else
                            {
                                pairs.emplace_back(
                                        rJob->scene.photostations[p.photostationId].getKMatrix(p.cameraId),
                                        p.projection);
                            }
                        }
                        auto wp = MultiPhotostationScene::BackProject(pairs);

                        int id= firstError[j];
                        for (auto& p: obs.projections)
                        {
                            auto res = (p.photostationId == psId ? ps_copy : rJob->scene.photostations[p.photostationId]).project(wp, p.cameraId) - p.projection;
                            result->a(id++, i) = res[0];
                            result->a(id++, i) = res[1];
                        }

                    }
                }
            }

            ParallelJacobianEvaluator(decltype(rJob) rJob, Matrix* m, double delta = 1e-7) : rJob(rJob), result(m), delta(delta)
            {
                int N = rJob->scene.pointObservations.size();
                firstError.resize(N);
                int cum = 0;
                for (int i = 1; i < N; ++i)
                    firstError[i] = firstError[i - 1] + 2 * rJob->scene.pointObservations[i - 1].projections.size();
            }

            ReconstructionJob* rJob;
            std::vector<int> firstError;
            int inputCnt;
            Matrix *result;
            double delta;
        };
#endif
        bool angleError = true;
    public:
        OptimizationFunctor(decltype(rJob) rJob, bool saturated = false, double saturationThreshold = 10, corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0), bool angleError=true/*,corecvs::Matrix22 covariation = corecvs::Matrix22(1.0, 0.0, 0.0, 1.0)*/) : FunctionArgs(rJob->getInputNum(), saturated || angleError ? rJob->getOutputNum() / 2 : rJob->getOutputNum()), rJob(rJob), saturated(saturated), saturationThreshold(saturationThreshold), mean(mean), scale(scale), angleError(angleError) //, covariation(covariation)
        {
        }
#if 0
        static double saturatedError(double x, double y, double saturationThreshold)
        {
            double r = std::sqrt(x * x + y * y);
            double alpha = 2.0 * saturationThreshold * saturationThreshold;
            double beta = 4.0 / saturationThreshold;
            double gamma = -4.0;

            if (r > saturationThreshold)
            {
                double e = exp(r * beta + gamma);
                double s = e / (1 + e);
                return std::sqrt(s);
            }
            else
            {
                return r;
            }

        }
        // Our J is sparse!
        Matrix getJacobian(const double in[], double)
        {
            std::cout << "JBG" << std::endl;
            Matrix resultA(rJob->getOutputNum(), rJob->getInputNum());
            Matrix resultB(rJob->getOutputNum(), rJob->getInputNum());

            int N = rJob->scene.pointObservations.size();

            corecvs::parallelable_for(0, N, ParallelJacobianEvaluator(rJob, &resultA, 1e-6));
            corecvs::parallelable_for(0, N, ParallelJacobianEvaluator(rJob, &resultB, -1e-6));

            if (saturated)
            {
#if 1
                int K = rJob->getInputNum();
                Matrix result(outputs, rJob->getInputNum());
                ParallelJacobianEvaluator pje(rJob, 0, 0);
        //        std::vector<double> errors;
          //      rJob->computeReprojectionErrors(errors);
                for (int i = 0; i < N; ++i)
                {
                    auto& obs = rJob->scene.pointObservations[i];
                    
                    int id = pje.firstError[i];
                    int M = obs.projections.size();
                    for (int j = 0; j < M; ++j)
                    {
                        int id2 = id / 2 + j;
                        for (int k = 0; k < K; ++k)
                        {
                            result.a(id2, k) = 
                                saturatedError(
                                        resultA.a(id + j * 2, k),
                                        resultA.a(id + j * 2 + 1, k),
                                        saturationThreshold)-
                                saturatedError(
                                        resultB.a(id + j * 2, k),
                                        resultB.a(id + j * 2 + 1, k),
                                        saturationThreshold);
                            if (result.a(j, k) != 0.0)
                                result.a(j, k) /= 2e-7;
                            assert(!std::isnan(result.a(j, k)));
                            assert(!std::isinf(result.a(j, k)));
                        }
                    }
                }
                std::cout << "JBE" << std::endl;
                return result;
#else
                return 
#endif
            }
            else
            {
                return (resultA - resultB) * (1.0 / 2e-6);
            }
        }
#endif
        void operator() (const double in[], double out[])
        {
            static double mx = 0.0;
            std::vector<double> inputs(in, in + rJob->getInputNum());
            rJob->readParams(in, mean, scale);
            std::vector<double> errors;
    //        std::cout << "UPD..." << std::endl;
#ifndef NOUPD
            rJob->scene.updateBackProjections();
#endif
  //          std::cout << "UPD...OK" << std::endl;
            rJob->scene.computeReprojectionErrors(errors);

            int outputNum = FunctionArgs::outputs;
            int outputPtr = 0;
            double total = 0.0;
            double cnt  = 0.0;
            int inliers = 0;
            double inliers_total = 0.0;
            if (saturated)
            {
                double alpha = 2.0 * saturationThreshold * saturationThreshold;
                double beta = 4.0 / saturationThreshold;
                double gamma = -4.0;

                for (int i = 0; i < errors.size(); i += 2)
                {
                    double dx = errors[i];
                    double dy = errors[i + 1];
                    assert(!std::isnan(dx) && !std::isnan(dy));
                    double r = std::sqrt(dx * dx + dy * dy);

                    if (r > saturationThreshold)
                    {
                        double e = exp(r * beta + gamma);
                        double err = out[outputPtr++] = std::sqrt(alpha * e / (1.0 + e));
                        total += err * err;
                        if (err > mx)
                        {
                            mx = err;
                            std::cout << "ME: " << mx << std::endl;
                        }
                    }
                    else
                    {
                        double err = out[outputPtr++] = std::abs(r);
                        total += err * err;
                        inliers_total += err * err;
                        inliers++;
                    }
                    cnt++;
                }
            }
            else
            {
                int id = 0;
                auto& scene = rJob->scene;
                if (angleError)
                {
//f 1
                for (auto& po: scene.pointObservations)
                {
                    auto wp = po.worldPoint;
                    for (auto& proj: po.projections)
                    {
                        auto cam = scene.photostations[proj.photostationId].getRawCamera(proj.cameraId);
                        auto dp = (cam.rayFromPixel(proj.projection)).a;
                        auto realDir = wp - cam.extrinsics.position;
                        out[outputPtr++] = realDir.angleTo(dp); 
                 //     out[outputPtr++] = realDir.angleTo(dp);
                        total += out[outputPtr - 1] * out[outputPtr - 1];
                        inliers_total += out[outputPtr - 1] * out[outputPtr - 1];
                        cnt += 1;
                        inliers += 1;
                    }
                }
//lse
                } else {
                for (int i = 0; i < errors.size(); ++i)
                {
                    double err = out[outputPtr++] = (errors[i]);
                    total += err * err;
                    cnt += 0.5;
                    inliers_total += err * err;
                    if (i % 2 == 0)
                        inliers++;
                }
                }
//ndif
            }
            static double minAng = 1e100, minRep = 1e100;
            if (angleError)
            {
                if (total < minAng * 0.5)
                {
            std::cout << "LMFA: " << std::sqrt(total / cnt) << " | " << std::sqrt(inliers_total / inliers) << " (" << cnt << " | " << inliers << ")" << std::endl;
            minAng = total;
                }
            } else
            {
                if (total < minRep * 0.5)
                {
            std::cout << "LMFR: " << std::sqrt(total / cnt) << " | " << std::sqrt(inliers_total / inliers) << " (" << cnt << " | " << inliers << ")" << std::endl;
            minRep = total;
                }

            }
//            std::cout << "ESZ:" << errors.size() << std::endl;
//          assert(outputNum == outputPtr);
        }


    };
#endif

    void undistortAll(bool singleDistortion = true);
    void detectAll();
    void descriptAll();
    void solve(bool angleError = true)
    {
        corecvs::LevenbergMarquardt LM(10000);
        corecvs::Vector3dd mean, scale;
        getScaler(mean, scale);
//        mean = 0.0;
//        scale = 1.0;
        std::cout << "MV: " << mean << " " << scale << std::endl;
        LM.f = new OptimizationFunctor(this,false,30.0,mean,scale,angleError);
        //LM.trace = true;
        std::vector<double> in(getInputNum()), out(LM.f->outputs);
        writeParams(&in[0], mean, scale);
        auto res = LM.fit(in, out);
        readParams(&res[0], mean, scale);
        return;
#if 0
    scene.computeReprojectionErrors(errors);
    std::vector<double> errors;
#else
    getScaler(mean, scale);
    OptimizationFunctor f(this, false, 30.0, mean, scale, angleError);
    std::vector<double> inf(getInputNum()), errors(f.outputs);
    writeParams(&inf[0], mean, scale);
    f(&inf[0], &errors[0]);
#endif
#if 0
    std::vector<double> terrors, xerrors, yerrors;
    for (int i = 0; i < errors.size(); i += 2)
    {
        double fin = std::sqrt(errors[i] * errors[i] + errors[i + 1] * errors[i + 1]);
        terrors.push_back(fin);
        xerrors.push_back(errors[i]);
        yerrors.push_back(errors[i + 1]);
    }
    std::sort(terrors.begin(), terrors.end());
    std::sort(xerrors.begin(), xerrors.end());
    std::sort(yerrors.begin(), yerrors.end());
    int NP = terrors.size();
#else
#endif
    double sum = 0.0;
    double inlier_sum = 0.0;
    int inliers = 0;
    for (auto& v: errors)
    {
        sum += v * v;
        if (v < 30.0)
        {
            inliers++;
            inlier_sum += v * v;
        }
    }
    sum /= errors.size();// / 2.0;
    inlier_sum /= inliers;
    std::ofstream er;
    std::vector<int> camid, psid;
    camid.resize(errors.size() / 2);
    psid.resize(errors.size() / 2);
    int idx = 0;
    for (auto& p: scene.pointObservations)
        for (auto& pp: p.projections)
        {
            camid[idx] = pp.cameraId;
            psid[idx++] = pp.photostationId;
        }
#if 0
    er.open("errors.csv", std::ios_base::out);
    er << "X, Y, cam, ps" << std::endl;
    for (int i = 0; i < errors.size(); i += 2)
    {
        er << errors[i] << ", " << errors[i + 1] << ", "
            << camid[i / 2] << ", " << ('A' + psid[i / 2]) << std::endl;
    }
#endif
    sort(errors.begin(), errors.end());


    

    int NN = errors.size();

    std::cout << "PTE: " << std::sqrt(sum) << " # " << inlier_sum << " | " << std::sqrt(inlier_sum)  << " (" << errors.size() <<  " | " << inliers << ")" <<
#if 0
        " 0.025Q: " << terrors[0.025 * NP] <<
        " 0.25Q: " << terrors[0.25 * NP] <<
        " 0.50Q: " << terrors[0.5  * NP] <<
        " 0.75Q: " << terrors[0.75 * NP] <<
        " 0.975Q: " << terrors[0.975 * NP] <<
        std::endl <<
        "XTE: " <<
        " 0.025Q: " << xerrors[0.025 * NP] <<
        " 0.25Q: " << xerrors[0.25 * NP] <<
        " 0.50Q: " << xerrors[0.5  * NP] <<
        " 0.75Q: " << xerrors[0.75 * NP] <<
        " 0.975Q: " << xerrors[0.975 * NP] <<
        std::endl <<
        "YTE: " <<
        " 0.025Q: " << yerrors[0.025 * NP] <<
        " 0.25Q: " << yerrors[0.25 * NP] <<
        " 0.50Q: " << yerrors[0.5  * NP] <<
        " 0.75Q: " << yerrors[0.75 * NP] <<
        " 0.975Q: " << yerrors[0.975 * NP] <<
#else
        " 0.025Q: " << errors[0.025 * NN] <<
        " 0.25Q: " << errors[0.25 * NN] <<
        " 0.50Q: " << errors[0.5  * NN] <<
        " 0.75Q: " << errors[0.75 * NN] <<
        " 0.975Q: " << errors[0.975 * NN] <<
#endif   
        std::endl;
    }
    void fill(std::unordered_map<std::string, LocationData> &data, bool, bool)
    {
        int id = 0;
        for (int id = 0; id < 5; ++id)
        {
            std::stringstream ss;
            ss << "SP" << (char)('A' + id);
            std::string prefix = ss.str();

            scene.photostations.push_back(calibrationData.photostation);
            scene.photostations.rbegin()->location = data[prefix];
            scene.photostations.rbegin()->location.orientation = scene.photostations.rbegin()->location.orientation ^ corecvs::Quaternion(.0, .0, sin(angleOffset[id]/2.0), cos(angleOffset[id]/2.0));
            std::cout << "CHECKL: " << data[prefix].position << std::endl;

            std::vector<CameraObservation> observations;
            for (int i = 0; i < 6; ++i)
            {
                CameraObservation observation;
                std::stringstream ss;
                ss << prefix << i << "_0deg.jpg";

                ss.str("");
                ss << prefix << i << "_undist.jpg";
                observation.undistortedFileName = ss.str();
                observation.sourceFileName = ss.str();

                observations.push_back(observation);
            }
            scene.cameraObservations.push_back(observations);
        }
    }
    void fill(std::unordered_map<std::string, LocationData> &data, bool)
    {
        for (int id = 0; id < NPS; ++id)
        {
            auto ps = GenerateModelPs();
            std::stringstream ss;
            ss << "SP" << (char)('A' + id);
            std::string prefix = ss.str();

            scene.photostations.push_back(ps);
            scene.photostations.rbegin()->location = data[prefix];
            //scene.photostations.rbegin()->location.orientation = scene.photostations.rbegin()->location.orientation ^ corecvs::Quaternion(.0, .0, sin(angleOffset[id]/2.0), cos(angleOffset[id]/2.0));

            std::vector<CameraObservation> observations;
            for (int i = 1; i < 25; ++i)
            {
                CameraObservation observation;
                std::stringstream ss;
                ss << prefix << std::setfill('0') << std::setw(2) << i << ".jpg";
                observation.sourceFileName = ss.str();
                observation.undistortedFileName = ss.str();

                observations.push_back(observation);
            }
            scene.cameraObservations.push_back(observations);
        }
    }
    void fill(std::unordered_map<std::string, LocationData> &data)
    {
        int id = 0;
        for (int id = 0; id < 5; ++id)
        {
            std::stringstream ss;
            ss << "SP" << (char)('A' + id);
            std::string prefix = ss.str();

            scene.photostations.push_back(calibrationData.photostation);
            scene.photostations.rbegin()->location = data[prefix];
            scene.photostations.rbegin()->location.orientation = scene.photostations.rbegin()->location.orientation ^ corecvs::Quaternion(.0, .0, sin(angleOffset[id]/2.0), cos(angleOffset[id]/2.0));

            std::vector<CameraObservation> observations;
            for (int i = 0; i < 6; ++i)
            {
                CameraObservation observation;
                std::stringstream ss;
                ss << prefix << i << "_0deg.jpg";
                observation.sourceFileName = ss.str();

                ss.str("");
                ss << prefix << i << "_undist.jpg";
                observation.undistortedFileName = ss.str();

                observations.push_back(observation);
            }
            scene.cameraObservations.push_back(observations);
        }
    }

    template <typename V>
    void accept(V &visitor)
    {
        ReconstructionParameters::accept(visitor);
        visitor.visit(scene,           MultiPhotostationScene(), "scene");
        visitor.visit(calibrationData, CalibrationJob(), "calibrationData");
    }

    struct ParallelUndistortionMapEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int camId = r.begin(); camId < r.end(); ++camId)
            {
                auto& cam = (*photostation).cameras[camId];
                calibrationJob->prepareUndistortionTransformation(cam.distortion, cam.intrinsics.distortedSize[0], cam.intrinsics.distortedSize[1], (*transformations)[camId], cam.intrinsics.size[0], cam.intrinsics.size[1]);
            }
        }


        std::vector<corecvs::DisplacementBuffer> *transformations;
        CalibrationJob *calibrationJob;
        Photostation *photostation;
        
        ParallelUndistortionMapEstimator(decltype(transformations) transformations, CalibrationJob* calibrationJob, Photostation* photostation) : transformations(transformations), calibrationJob(calibrationJob), photostation(photostation)
        {
        }
    };

    struct ParallelUndistortionCalculator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int camId = r.begin(); camId < r.end(); ++camId)
            {
                auto& o = (*observations)[camId];
                auto& t = (*transformations)[camId];
                auto& s = (*photostation).cameras[camId].intrinsics.size;

                // Read file
                corecvs::RGB24Buffer src = BufferReaderProvider::readRgb(o.sourceFileName);
                // Apply undistortion
                auto* res = src.doReverseDeformationBlTyped<corecvs::DisplacementBuffer>
                    (&t, s[1], s[0]);
                // Write file
                BufferReaderProvider::writeRgb(*res, o.undistortedFileName);
                delete res;
            }
        }


        std::vector<corecvs::DisplacementBuffer> *transformations;
        std::vector<CameraObservation> *observations;
        Photostation *photostation;
        
        ParallelUndistortionCalculator(decltype(observations) observations, decltype(transformations) transformations, Photostation* photostation)
            : transformations(transformations), observations(observations), photostation(photostation)
        {
        }
    };
};

class PhotostationFilenameMatchingPlanComputationStage : public FeatureMatchingPipelineStage
{
public:
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename)
    {
        pipeline->matchPlan.load(filename);
    }
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const
    {
        pipeline->matchPlan.save(filename);
    }
	void run(FeatureMatchingPipeline *pipeline)
    {
        pipeline->tic();
        MatchPlan &matchPlan = pipeline->matchPlan;
        std::vector<Image> &images = pipeline->images;
        size_t N = images.size();

        matchPlan.plan.clear();

        for (size_t i = 0; i < N; ++i)
        {
            std::deque<uint16_t> query(images[i].keyPoints.keyPoints.size());
            for (size_t j = 0; j < images[i].keyPoints.keyPoints.size(); ++j)
            {
                query[j] = (uint16_t)j;
            }

            for (size_t j = 0; j < N; ++j)
            {
                if (i == j)
                    continue;
                std::string& name1 = images[i].filename;
                std::string& name2 = images[j].filename;
                int idx1 = (name1[3]-'0');
                int idx2 = (name2[3]-'0');
                int diff  = (6 * 100 + idx1 - idx2) % 6;
                int diff2 = (6 * 100 + idx2 - idx1) % 6;
                diff = CORE_MIN(diff, diff2);
                if (diff > 0)
                {
                    continue;
                }
                std::deque<uint16_t> train(images[j].keyPoints.keyPoints.size());
                for (size_t k = 0; k < images[j].keyPoints.keyPoints.size(); ++k)
                {
                    train[k] = (uint16_t)k;
                }

                MatchPlanEntry entry = { (uint16_t)i, (uint16_t)j, query, train };
                matchPlan.plan.push_back(entry);
            }
        }
        pipeline->toc("Preparing matching plan", "");

    }
	~PhotostationFilenameMatchingPlanComputationStage() {}
};


void ReconstructionJob::detectAll()
{
    std::vector<std::string> filenames;
    int M = scene.getPhotostationCount();

    std::unordered_map<int, std::pair<int, int>> img_map;

    int idx = 0;
    for (int i = 0; i < M; ++i)
    {
        int N = scene.photostations[i].cameras.size();
        for (int j = 0; j < N; ++j)
        {
            filenames.push_back(scene.cameraObservations[i][j].undistortedFileName);
            std::cout << "Adding: " << *filenames.rbegin() << std::endl;
            img_map[idx++] = std::make_pair(i, j);
        }
    }

    FeatureMatchingPipeline pipeline(filenames);
    pipeline.add(new KeyPointDetectionStage(detector), true);
    pipeline.add(new DescriptorExtractionStage(descriptor), true);
//    pipeline.add(new PhotostationFilenameMatchingPlanComputationStage(), true);
    pipeline.add(new FileNameRefinedMatchingPlanComputationStage(), true);
    pipeline.add(new MatchAndRefineStage(descriptor, matcher), true);

    pipeline.run();

    struct err
    {
        double err_x,
               err_y;
        int psA, psB;
        int camA, camB;
        double f;
    };
    std::vector<err> errors_f;


    int N = 0;
    for (auto& set : pipeline.refinedMatches.matchSets)
    {
        for (auto& match: set.matches)
            N = std::max(N, 36 * std::max(match.featureA, match.featureB));
    }

    UndirectedGraph uag(N+10);

    std::unordered_map<std::pair<int, int>, Match> graphToMatches;
    std::ofstream ofcm;
    ofcm.open("matches.csv", std::ios_base::out);
    ofcm << "Ax, Ay, As, Ao, Bx, By, Bs, Bo" << std::endl;


    for (auto& set: pipeline.refinedMatches.matchSets)
    {
        for (auto& match: set.matches)
        {
            auto id1 = img_map[match.imgA];
            auto id2 = img_map[match.imgB];

            auto p1 = pipeline.images[match.imgA].keyPoints.keyPoints[match.featureA];
            auto p2 = pipeline.images[match.imgB].keyPoints.keyPoints[match.featureB];
            auto bp = scene.backProject({{id1, corecvs::Vector2dd(p1.x, p1.y)}, {id2, corecvs::Vector2dd(p2.x, p2.y)}});
            auto pp1 = scene.project(id1.first, id1.second, bp);
            auto pp2 = scene.project(id2.first, id2.second, bp);

            auto ppg1 = corecvs::Vector2dd(p1.x, p1.y);
            auto ppg2 = corecvs::Vector2dd(p2.x, p2.y);

            double diff = std::sqrt(std::max(!(pp1-ppg1), !(pp2-ppg2)));
            ofcm << p1.x << ", " <<
                    p1.y << ", " <<
                    p1.size << ", " <<
                    p1.octave << ", " <<
                    p2.x << ", " <<
                    p2.y << ", " <<
                    p2.size << ", " <<
                    p2.octave << std::endl;
            ofcm << p2.x << ", " <<
                    p2.y << ", " <<
                    p2.size << ", " <<
                    p2.octave << ", " <<
                    p1.x << ", " <<
                    p1.y << ", " <<
                    p1.size << ", " <<
                    p1.octave << std::endl;


            if (scene.photostations[id1.first].isVisible(bp, id1.second) && scene.photostations[id2.first].isVisible(bp, id2.second) && diff < 1e20)
            {
                int mid1 = 1 + 36 * match.featureA + 6 * id1.first + id1.second;
                int mid2 = 1 + 36 * match.featureB + 6 * id2.first + id2.second;

                uag.addEdge(mid1, mid2);

                graphToMatches[std::make_pair(mid1, mid2)] = match;
                graphToMatches[std::make_pair(mid2, mid1)] = match;
                std::cout <<
                    "PT:BOTH" << std::endl;
                auto cam1 = scene.photostations[id1.first].getRawCamera(id1.second);
                auto cam2 = scene.photostations[id2.first].getRawCamera(id2.second);
                corecvs::Matrix33 K1 = (corecvs::Matrix33)cam1.intrinsics,
                                  K2 = (corecvs::Matrix33)cam2.intrinsics,
                                  R1 = cam1.extrinsics.orientation.toMatrix(),
                                  R2 = cam2.extrinsics.orientation.toMatrix();
                corecvs::Vector3dd C1 = cam1.extrinsics.position,
                                   C2 = cam2.extrinsics.position;

                corecvs::Matrix33 R = R2 * R1.inv();
                corecvs::Vector3dd T = R2 * R1.inv() * C1 - R2 * C2;
                corecvs::Matrix33 F = K1.inv().transposed() * corecvs::Matrix33::CrossProductLeft(T) * R * K2.inv();
                corecvs::Vector3dd line = corecvs::Vector3dd(p1.x, p1.y, 1.0) * F;
                double a = std::sqrt(line[0] * line[0] + line[1] * line[1]);
                double dot = (line & corecvs::Vector3dd(p2.x, p2.y, 1.0)) / a;
                corecvs::Vector3dd line2 = F * corecvs::Vector3dd(p2.x, p2.y, 1.0);
                double a2 = std::sqrt(line2[0] * line2[0] + line2[1] * line2[1]);
                double dot2 = (line2 & corecvs::Vector3dd(p1.x, p1.y, 1.0)) / a2;

                err error;
                error.err_x = (pp1 - ppg1)[0];
                error.err_y = (pp1 - ppg1)[1];
                error.f     = a2;
                error.psA = id1.first;
                error.psB = id2.first;
                error.camA = id1.second;
                error.camB = id2.second;
                errors_f.push_back(error);
                error.err_x = (pp2 - ppg2)[0];
                error.err_y = (pp2 - ppg2)[1];
                error.f     = a;
                error.psA = id2.first;
                error.psB = id1.first;
                error.camA = id2.second;
                error.camB = id1.second;
                errors_f.push_back(error);

                if (std::abs(dot) > 20.0)
                {
                    std::cout << "PT:FAILF" << std::endl;
        //            continue;
                }


                PointProjection pr1, pr2;
                pr1.projection = corecvs::Vector2dd(p1.x, p1.y);
                pr1.photostationId = id1.first;
                pr1.cameraId = id1.second;
                pr1.featureId = match.featureA;

                pr2.projection = corecvs::Vector2dd(p2.x, p2.y);
                pr2.photostationId = id2.first;
                pr2.cameraId = id2.second;
                pr2.featureId = match.featureB;

                PointObservation__ obs;
                obs.worldPoint = bp;
                obs.projections = { pr1, pr2 };

                scene.pointObservations.push_back(obs);
            }
            else
            {
                std::cout <<
                    "PT:FAIL" << std::endl;
            }

                
        }
    }
    auto cl = uag.maximalCliques();
    std::unordered_map<int, int> cc;
    std::unordered_map<int, int> psc;
    for (auto& c: cl)
        cc[c.size()]++;

    for(auto& p: cc)
        std::cout << "CL: " << p.first << " : " << p.second << std::endl;
    std::string boo;
    std::cin >> boo;

    scene.pointObservations.clear();
    for (auto& c: cl)
    {
        if (c.size() < 3)
            continue;
        int Nx = c.size();
        std::unordered_set<std::pair<int, int>> imgKeyPoints;
        for (int i = 0; i < Nx; ++i)
            for (int j = i + 1; j < Nx; ++j)
            {
                auto& match = graphToMatches[std::make_pair(c[i], c[j])];
                imgKeyPoints.insert(std::make_pair(match.imgA, match.featureA));
                imgKeyPoints.insert(std::make_pair(match.imgB, match.featureB));
            }
        std::vector<std::pair<std::pair<int, int>, corecvs::Vector2dd>> projection;
        std::vector<PointProjection> projections;
        for (auto& p: imgKeyPoints)
        {
            corecvs::Vector2dd proj = corecvs::Vector2dd(pipeline.images[p.first].keyPoints.keyPoints[p.second].x, pipeline.images[p.first].keyPoints.keyPoints[p.second].y);
            projection.emplace_back(img_map[p.first], proj);
            PointProjection pr;
            pr.projection = proj;
            pr.photostationId = img_map[p.first].first;
            pr.cameraId = img_map[p.first].second;
            pr.featureId = p.second;
            projections.push_back(pr);
        }
        auto res = scene.backProject(projection);
        bool ok = true;
        for (auto& p: imgKeyPoints)
        {
            if (!scene.photostations[img_map[p.first].first].isVisible(res, img_map[p.first].second))
            {
                ok = false;
                break;
            }
        }
        int Nxx = projections.size();
        for (int i = 0; i < Nxx; ++i)
        {
            for (int j = i + 1; j < Nxx; ++j)
            {
                auto id1 = projection[i].first;
                auto id2 = projection[j].first;
                auto p1 = projection[i].second;
                auto p2 = projection[j].second;

                auto cam1 = scene.photostations[id1.first].getRawCamera(id1.second);
                auto cam2 = scene.photostations[id2.first].getRawCamera(id2.second);
                corecvs::Matrix33 K1 = (corecvs::Matrix33)cam1.intrinsics,
                                  K2 = (corecvs::Matrix33)cam2.intrinsics,
                                  R1 = cam1.extrinsics.orientation.toMatrix(),
                                  R2 = cam2.extrinsics.orientation.toMatrix();
                corecvs::Vector3dd C1 = cam1.extrinsics.position,
                                   C2 = cam2.extrinsics.position;

                corecvs::Matrix33 R = R2 * R1.inv();
                corecvs::Vector3dd T = R2 * R1.inv() * C1 - R2 * C2;
                corecvs::Matrix33 F = K1.inv().transposed() * corecvs::Matrix33::CrossProductLeft(T) * R * K2.inv();
                corecvs::Vector3dd line = corecvs::Vector3dd(p1.x(), p1.y(), 1.0) * F;
                double a = std::sqrt(line[0] * line[0] + line[1] * line[1]);
                double dot = (line & corecvs::Vector3dd(p2.x(), p2.y(), 1.0)) / a;
                corecvs::Vector3dd line2 = F * corecvs::Vector3dd(p2.x(), p2.y(), 1.0);
                double a2 = std::sqrt(line2[0] * line2[0] + line2[1] * line2[1]);
                double dot2 = (line2 & corecvs::Vector3dd(p1.x(), p1.y(), 1.0)) / a2;

        //      if (std::max(std::abs(dot), std::abs(dot2)) > 100.0)
//                  ok = false;
            }
        }
        if (!ok)
            continue;
        PointObservation__ obs;
        obs.worldPoint = res;
        obs.projections = projections;
        scene.pointObservations.push_back(obs);
    }
    std::cout << "TOTAL: " << scene.pointObservations.size() << std::endl;

    std::ofstream ofc;
    ofc.open("errors_data.csv", std::ios_base::out);
    ofc << "dx, dy, psA, psB, camA, camB, f" << std::endl;
    for (auto& e: errors_f)
    {
        ofc << 
            e.err_x << ", " <<
            e.err_y << ", " <<
            e.psA   << ", " <<
            e.psB   << ", " <<
            e.camA  << ", " <<
            e.camB  << ", " <<
            e.f << std::endl;
    }


    std::ofstream ofk;
    ofk.open("keypoints_stats.csv", std::ios_base::out);
    ofk << "x, y, sz, octave" << std::endl;

    idx = 0;
    for (int i = 0; i < M; ++i)
    {
        int N = scene.photostations[i].cameras.size();
        for (int j = 0; j < N; ++j)
        {
            scene.cameraObservations[i][j].keyPoints = pipeline.images[idx++].keyPoints.keyPoints;
            for (auto& kp: scene.cameraObservations[i][j].keyPoints)
                ofk << kp.x << ", " << kp.y << ", " << kp.size << ", " << kp.octave << std::endl;
        }
    }

#if 0
    std::vector<double> errors;
    scene.computeReprojectionErrors(errors);
#else
    OptimizationFunctor f(this, true, 30.0);
    std::vector<double> inf(getInputNum()), errors(f.outputs);
    writeParams(&inf[0]);
    f(&inf[0], &errors[0]);
#endif
#if 1
#if 0
    std::vector<double> terrors, xerrors, yerrors;
    for (int i = 0; i < errors.size(); i += 2)
    {
        double fin = std::sqrt(errors[i] * errors[i] + errors[i + 1] * errors[i + 1]);
        terrors.push_back(fin);
        xerrors.push_back(errors[i]);
        yerrors.push_back(errors[i + 1]);
    }
    std::sort(terrors.begin(), terrors.end());
    std::sort(xerrors.begin(), xerrors.end());
    std::sort(yerrors.begin(), yerrors.end());
    int NP = terrors.size();
#else
#endif
    double sum = 0.0, inlier_sum = 0.0;
    int inliers = 0;
    for (auto& v: errors)
    {
        sum += v * v;
        if (v < 30.0)
        {
            inliers++;
            inlier_sum += v * v;
        }
    }
    sum /= errors.size();// / 2.0;
    inlier_sum /= inliers;
    sort(errors.begin(), errors.end());


    

    std::ofstream er;
    er.open("errors.csv", std::ios_base::out);
    for (int i = 0; i < errors.size(); i += 2)
    {
        er << errors[i] << ", " << errors[i + 1] << std::endl;
    }
    int NN = errors.size();

    std::cout << "PTE: " << sum << "|" << std::sqrt(sum) << " # " << inlier_sum << " | " << std::sqrt(inlier_sum) << " (" << errors.size() << " | " << inliers <<  ")" <<
#if 0
        " 0.025Q: " << terrors[0.025 * NP] <<
        " 0.25Q: " << terrors[0.25 * NP] <<
        " 0.50Q: " << terrors[0.5  * NP] <<
        " 0.75Q: " << terrors[0.75 * NP] <<
        " 0.975Q: " << terrors[0.975 * NP] <<
        std::endl <<
        "XTE: " <<
        " 0.025Q: " << xerrors[0.025 * NP] <<
        " 0.25Q: " << xerrors[0.25 * NP] <<
        " 0.50Q: " << xerrors[0.5  * NP] <<
        " 0.75Q: " << xerrors[0.75 * NP] <<
        " 0.975Q: " << xerrors[0.975 * NP] <<
        std::endl <<
        "YTE: " <<
        " 0.025Q: " << yerrors[0.025 * NP] <<
        " 0.25Q: " << yerrors[0.25 * NP] <<
        " 0.50Q: " << yerrors[0.5  * NP] <<
        " 0.75Q: " << yerrors[0.75 * NP] <<
        " 0.975Q: " << yerrors[0.975 * NP] <<
#else
        " 0.025Q: " << errors[0.025 * NN] <<
        " 0.25Q: " << errors[0.25 * NN] <<
        " 0.50Q: " << errors[0.5  * NN] <<
        " 0.75Q: " << errors[0.75 * NN] <<
        " 0.975Q: " << errors[0.975 * NN] <<
#endif   
        std::endl;

#else
    std::vector<double> terrors, xerrors, yerrors;
    for (int i = 0; i < errors.size(); i += 2)
    {
        double fin = std::sqrt(errors[i] * errors[i] + errors[i + 1] * errors[i + 1]);
        terrors.push_back(fin);
        xerrors.push_back(errors[i]);
        yerrors.push_back(errors[i + 1]);
    }
    std::sort(terrors.begin(), terrors.end());
    std::sort(xerrors.begin(), xerrors.end());
    std::sort(yerrors.begin(), yerrors.end());
    int NP = terrors.size();
    double sum = 0.0;
    for (auto& v: errors)
        sum += v * v;
    sum /= errors.size();// / 2.0;


    

    std::ofstream er;
    er.open("errors.csv", std::ios_base::out);
    for (int i = 0; i < errors.size(); i += 2)
    {
        er << errors[i] << ", " << errors[i + 1] << std::endl;
    }

    std::cout << "PTE: " << std::sqrt(sum) << " (" << errors.size() << ")" <<
        " 0.025Q: " << terrors[0.025 * NP] <<
        " 0.25Q: " << terrors[0.25 * NP] <<
        " 0.50Q: " << terrors[0.5  * NP] <<
        " 0.75Q: " << terrors[0.75 * NP] <<
        " 0.975Q: " << terrors[0.975 * NP] <<
        std::endl <<
        "XTE: " <<
        " 0.025Q: " << xerrors[0.025 * NP] <<
        " 0.25Q: " << xerrors[0.25 * NP] <<
        " 0.50Q: " << xerrors[0.5  * NP] <<
        " 0.75Q: " << xerrors[0.75 * NP] <<
        " 0.975Q: " << xerrors[0.975 * NP] <<
        std::endl <<
        "YTE: " <<
        " 0.025Q: " << yerrors[0.025 * NP] <<
        " 0.25Q: " << yerrors[0.25 * NP] <<
        " 0.50Q: " << yerrors[0.5  * NP] <<
        " 0.75Q: " << yerrors[0.75 * NP] <<
        " 0.975Q: " << yerrors[0.975 * NP] <<
        
        std::endl;
#endif
}


void ReconstructionJob::undistortAll(bool singleDistortion)
{
    int M = scene.getPhotostationCount();
    std::vector<corecvs::DisplacementBuffer> transformations;
    for (int i = 0; i < M; ++i)
    {
        int N = scene.photostations[i].cameras.size();
        if (i == 0 || !singleDistortion)
        {
            transformations.resize(N);
            corecvs::parallelable_for(0, N, ParallelUndistortionMapEstimator(&transformations, &calibrationData, &scene.photostations[i]));
        }
        corecvs::parallelable_for(0, N, ParallelUndistortionCalculator(&scene.cameraObservations[i], &transformations, &scene.photostations[i]));
    }

    
}

std::vector<PointObservation__> parsePois(const std::string &filename, bool m15 = true)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);

    std::vector<PointObservation__> res;

    int N;
    ifs >> N;
    for (int i = 0; i < N; ++i)
    {
        std::string label;
        corecvs::Vector3dd pos;
        int M;
        std::string lstr;
        while (!lstr.size() && (bool)ifs)
        {
            std::getline(ifs, lstr);
        }
        assert(lstr.size());
        std::stringstream ss(lstr);
    
        ss >> label;
        if (label.size() < lstr.size())
        {
            ss >> pos[0] >> pos[1] >> pos[2];
        }
        ifs >> M;

        std::vector<PointProjection> projections;
        PointObservation__ observation;
        observation.worldPoint = convertVector(pos);
        observation.updateable = label.size() == lstr.size();
        observation.label = label;

        std::cout << "POI: LABEL: " << label << ": ";
        if (!observation.updateable) std::cout << observation.worldPoint;
        std::cout << std::endl;

        for (int j = 0; j < M; ++j)
        {
            std::string filename;
            double x, y;

            ifs >> filename >> x >> y;
            if (x < 0 || y < 0) continue;

            int psId = filename[2] - 'A';

            int camId = m15 ?
                (filename[3] - '0')
                :((filename[3] - '0') * 10 + filename[4] - '0') - 1;
            PointProjection proj;
            proj.projection = corecvs::Vector2dd(x, y)-corecvs::Vector2dd(0.5, 0.5);
            proj.cameraId = camId;
            proj.photostationId = psId;
 //         if (camId == 0)            
            projections.push_back(proj);
             std::cout << "POI: LABEL: " << label << ": CAM: " << filename << " (" << camId << "|" << psId << ")" << proj.projection << std::endl;
            
        }
        observation.projections = projections;
//      if (projections.size())
        if (!observation.updateable && projections.size() > 1)
        res.push_back(observation);
    }

    return res;

}

// XXX: here we parse only position part; angle data is rejected
//      we also make prefix uppercase since it is partially lower-case
//      in Measure_15
std::unordered_map<std::string, LocationData>  parseGps(const std::string &filename, const corecvs::Matrix33 &axis = corecvs::Matrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0) )
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);
    if (!ifs)
    {
        std::cout << "GPS data file " << filename << " cannot be opened!" << std::endl;
        exit(0);
    }

    std::unordered_map<std::string, LocationData> locations;
    std::string data;
    // Just prefix and 3 numbers delimited by spaces
    std::string prefix_regex = "([A-Za-z]+)",
                float_regex  = "([+-]?[0-9]+\\.?[0-9]*)",
                sep_regex    = "\\s+";
    std::regex re("^" + prefix_regex + sep_regex + float_regex + sep_regex + float_regex + sep_regex + float_regex +  "((\\s.*)?|$)");
    do
    {
        std::getline(ifs, data);
        if (data.size() && *data.rbegin() == '\r')
            data.resize(data.size() - 1);
        if (!data.size()) continue;

        std::smatch m;
        if (!std::regex_match(data, m, re) || m.size() < 5) // 4 tokens + final
        {
            std::cout << "REJECTED: " << data << std::endl;
            continue;
        }

        std::string key;
        double n, e, h;
        std::stringstream ss(data);
        ss >> key >> n >> e >> h;
        for (auto& c: key) c = toupper(c);
        std::cout << "Key: " << key << " geodesic stuff: " << n << " " << e << " " << h << std::endl;

        // XXX: note, that stuff valid only for current proto, somewhere we should add switch for it
        LocationData location;
#ifndef METERS
        location.position[0] = e * 1e3;
        location.position[1] = n * 1e3;
        location.position[2] = h * 1e3;
#else
        location.position[0] = e;
        location.position[1] = n;
        location.position[2] = h;
#endif
//      location.position = axis * location.position;
        double phi=0.0;//-0.22*M_PI / 180.0;

        location.orientation = 
#if 0
            corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);//corecvs::Quaternion(0.5, 0.5, 0.5, 0.5);//corecvs::Quaternion(sin(M_PI / 4.0), 0.0, 0.0, cos(M_PI / 4.0));
#else
        corecvs::Quaternion(0,0,sin(phi),cos(phi)) ^ corecvs::Quaternion(0.5, 0.5, 0.5, 0.5);
#endif

        locations[key] = location;

        std::cout << "Key: " << key << " our stuff: " << location.position << " " << location.orientation << std::endl;
    } while(ifs);
    return locations;
}

corecvs::Vector3dd generateSpherePoint(const corecvs::Vector3dd &center, const double r, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> rngZ(-1, 1);
    std::uniform_real_distribution<double> rngPhi(0.0, 2.0 * M_PI);
    double phi = rngPhi(rng);
    double z = rngZ(rng);
    double zi = std::sqrt(1.0 - z * z);
    corecvs::Vector3dd sp(cos(phi) * zi, sin(phi) * zi, z);
    assert(std::abs((!sp) - 1.0) < 1e-5);
    return sp * r + center;
}

corecvs::Vector3dd generateBallPoint(const corecvs::Vector3dd &center, const double R, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> rngR;
    auto s = generateSpherePoint(corecvs::Vector3dd(0, 0, 0), 1.0, rng);
    double r = (rngR(rng)) * R;
    assert((!(r * s)) <= R);
    return center + r * s;
}

std::vector<corecvs::Vector3dd> generatePois(const std::vector<corecvs::Photostation> &pss, double r, size_t N, std::mt19937 &rng, bool sphere = true)
{
    corecvs::Vector3dd center(0, 0, 0);
    double cnt = 0.0;
    for (auto& ps: pss)
    {
        center += ps.location.position;
        cnt += 1.0;
    }
    center = center * (1.0 / cnt);

    std::vector<corecvs::Vector3dd> pois;
    while (pois.size() < N)
    {
        auto vec = sphere ? generateSpherePoint(center, r, rng) : generateBallPoint(center, r, rng);
        int isVisible = 0;
        for (auto& ps: pss)
            if (ps.isVisible(vec))
            {
                isVisible++;
            } 
        if (isVisible < 5)
            continue;
        pois.push_back(vec);
    }
    return pois;
}

corecvs::Vector2dd generateNormalLimited(double sigma, double maxDelta, std::mt19937 &rng)
{
    corecvs::Vector2dd res;
    std::normal_distribution<double> rnn(0.0, sigma);
    do
    {
        res[0] = rnn(rng);
        res[1] = rnn(rng);
    } while (!res > maxDelta);
    return res;
}

corecvs::Quaternion generateQuaternionDelta(double sigma, std::mt19937 &rng)
{
    auto v = generateSpherePoint(corecvs::Vector3dd(0, 0, 0), 1.0, rng);
    double phi = generateNormalLimited(sigma * M_PI / 180.0, M_PI / 2.0, rng)[0];
    double s = sin(phi / 2.0);
    double c = cos(phi / 2.0);
    return corecvs::Quaternion(s * v[0], s * v[1], s * v[2], c);
}

corecvs::Vector3dd generateVectorDelta(double sigma, std::mt19937 &rng)
{
    corecvs::Vector3dd vec(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; ++i)
        vec[i] = generateNormalLimited(sigma, sigma * 100, rng)[0];
    return vec;
}

std::vector<PointObservation__> generateObservations(const std::vector<corecvs::Photostation> &pss, const std::vector<corecvs::Vector3dd> &pois, double sigma, double maxDelta, std::mt19937 &rng)
{
    std::vector<PointObservation__> observations;
    for (auto& poi: pois)
    {
        PointObservation__ observation;
        observation.updateable = false;
        observation.worldPoint = poi;

        for (size_t i = 0; i < pss.size(); ++i)
        {
            for (size_t j = 0; j < pss[i].cameras.size(); ++j)
            {
                if (pss[i].isVisible(poi, j))
                {
                    PointProjection proj;
                    proj.photostationId = i;
                    proj.cameraId = j;
                    proj.projection = pss[i].project(poi, j) + generateNormalLimited(sigma, maxDelta, rng);
                    observation.projections.push_back(proj);
                }
            }
        }

        observations.push_back(observation);
    }
    return observations;
}

std::vector<std::vector<double>> runExp(int N, double R, double sigma, double maxDelta, double sigmaPos, double sigmaAngle, int M, int N_checkPoints, double R_checkPoints, std::vector<std::vector<double>> &errCheck)
{
    std::vector<std::vector<double>> results;
    std::random_device hw;
    std::mt19937 rng(hw());

    std::chrono::time_point<std::chrono::high_resolution_clock> 
        start = std::chrono::high_resolution_clock::now(),
        stop;
    for (int jj = 0; jj < M; ++jj)
    {
        ReconstructionJob job;
        auto ps = GenerateModelPs();
        std::vector<Photostation> pss;
        corecvs::Vector3dd vec(0, 0, 0);
        for (int i = 0; i < NPS; ++i)
        {
            ps.location.position = locations[i];
            ps.location.orientation = orientation;
            pss.push_back(ps);
            vec = vec + locations[i];
        }
        vec = vec * (1.0 / NPS);
//        std::cout << "MCo: POI" << std::endl;
        std::cout << "MCSTAT: MODELP" << std::endl;
        auto pois = generatePois(pss, R, N, rng, true);
        auto pois_check = generatePois(pss, R_checkPoints, N_checkPoints, rng, false);

        for (int kk = 0; kk < 1; ++kk)
        {
            std::cout << "MCSTAT: MODELO" << std::endl;
//            std::cout << "MCi: " << kk << " / " << M << " (" << jj << " / " << M << ")" << std::endl;
//            std::cout << "MCi: GO" << std::endl;
            auto obs = generateObservations(pss, pois, sigma, maxDelta, rng);
            job.scene.photostations = pss;
            job.scene.pointObservations = obs;
 
            for (int i = 0; i < NPS; ++i)
            {
                job.scene.photostations[i].location.position += generateVectorDelta(sigmaPos, rng);
                job.scene.photostations[i].location.orientation = job.scene.photostations[i].location.orientation ^ generateQuaternionDelta(sigmaAngle, rng);
            }
//            std::cout << "MCi: ERR" << std::endl;
            std::vector<double> errors;
            job.scene.computeReprojectionErrors(errors);
            double ssq = 0.0;
            double mxr = 0.0;
            for (int i = 0; i < errors.size(); i += 2)
            {
                double r2 = errors[i] * errors[i] + errors[i + 1] * errors[i + 1];
                double r = std::sqrt(r2);
                if (r > mxr) mxr = r;
                ssq += r2;
            }
            double rmseI = std::sqrt(ssq / (errors.size() / 2.0));
            double mxrI = mxr;
            
//            std::cout << "MCi: SOL" << std::endl;
            std::cout << "MCSTAT: SOLVING" << std::endl;
            start = std::chrono::high_resolution_clock::now();
          job.solve();
          stop = std::chrono::high_resolution_clock::now();
            std::cout << "MCSTAT: SOLVED " << std::chrono::duration<double>(stop - start).count() << "s " << std::endl;

            std::vector<double> output = { R, N, sigma, sigmaPos, sigmaAngle, jj};
            for (int i = 0; i < pss.size(); ++i)
            {
                for (int j = 0; j < 3; ++j)
                    output.push_back((pss[i].location.position[j] - job.scene.photostations[i].location.position[j]));
                for (int j = 0; j < 4; ++j)
                    output.push_back((pss[i].location.orientation.conjugated() ^job.scene.photostations[i].location.orientation)[j]);
            }

//            std::cout << "MCi: ERR" << std::endl;
            errors.clear();
            job.scene.computeReprojectionErrors(errors);
            ssq = 0.0;
            mxr = 0.0;
            for (int i = 0; i < errors.size(); i += 2)
            {
                double r2 = errors[i] * errors[i] + errors[i + 1] * errors[i + 1];
                double r = std::sqrt(r2);
                if (r > mxr) mxr = r;
                ssq += r2;
            }
            double rmse = std::sqrt(ssq / (errors.size() / 2.0));
            output.push_back(rmse);
            output.push_back(mxr);
            output.push_back(rmseI);
            output.push_back(mxrI);
            results.push_back(output);

            std::cout << "MCSTAT: MODELO2" << std::endl;
            start = std::chrono::high_resolution_clock::now();
            job.scene.cameraObservations.resize(NPS);
            for (int i = 0; i < NPS; ++i)
            {
                job.scene.cameraObservations[i].resize(job.scene.photostations[i].cameras.size());
            }

            auto obs_check = generateObservations(pss, pois_check, sigma, maxDelta, rng);
            for (auto &o: obs_check)
            {
                o.updateable = true;
                for (auto& p: o.projections)
                {
                    KeyPoint kp;
                    kp.x = p.projection.x();
                    kp.y = p.projection.y();
                    p.featureId = job.scene.cameraObservations[p.photostationId][p.cameraId].keyPoints.size();
                    job.scene.cameraObservations[p.photostationId][p.cameraId].keyPoints.push_back(kp);
                }
            }
            stop = std::chrono::high_resolution_clock::now();
            std::cout << "MCSTAT: MODELO2ED " << std::chrono::duration<double>(stop - start).count() << "s " << std::endl;

            std::cout << "MCSTAT: UPD " << std::endl;
            start = std::chrono::high_resolution_clock::now();
            job.scene.pointObservations = obs_check;
            job.scene.updateBackProjections();
            stop = std::chrono::high_resolution_clock::now();
            std::cout << "MCSTAT: UPDED " << std::chrono::duration<double>(stop - start).count() << "s " << std::endl;


            for (size_t i = 0; i < obs_check.size(); ++i)
            {
                auto pos = obs_check[i].worldPoint;
                auto diff = job.scene.pointObservations[i].worldPoint - pos;
                pos = pos - vec;
                double d = !pos;
                assert(!pos < 
#ifdef METERS
                        60
#else
                        60*1e3
#endif
                      );
                double e = !diff;

                std::vector<double> row(9);
                for (int j = 0; j < 3; ++j)
                {
                    row[j] = pos[j];
                    row[3 + j] = diff[j];
                }
                row[6] = e;
                row[7] = d;
                int cntOk = 0;
                for (int iii = 0; iii < obs_check[i].projections.size(); ++iii)
                {
                    auto wp = job.scene.pointObservations[i].worldPoint;
                    auto pp = job.scene.pointObservations[i].projections[iii];
                    if (job.scene.photostations[pp.photostationId].isVisible(wp, pp.cameraId))
                        cntOk++;
                            
                }
                row[8] = cntOk - (int)obs_check[i].projections.size();
             errCheck.push_back(row);
            }
        }
    }
    return results;
}

struct Desc
{
    int N;
    double sigma;
    double R;
    double maxDelta;
    double sigmaPos;
    double sigmaAngle;
    int M;
    int N_checkPoints;
    double R_checkPoints;
    int confId;
};

#define FULL_FILE "errors_mc.csv"
tbb::mutex out_mutex;

struct ParallelSimulator
{
public:
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto& d = descriptions[i];
            if (!((d.confId >= 11 && d.confId <= 15) || (d.confId != 8 && d.confId != 4 && d.confId != 6 && d.confId >= 3 && d.confId <= 9)))
                continue;
            std::vector<std::vector<double>> errors;
            std::cout << "MCST: Starting #" << i << std::endl; // N = " << d.N << " R = " << d.R << " sigma = " << d.sigma << std::endl;
            (*results)[i] = runExp(d.N, d.R, d.sigma, d.maxDelta, d.sigmaPos, d.sigmaAngle, d.M, d.N_checkPoints, d.R_checkPoints, errors);
            std::cout << "MCST: Ending #" << i << std::endl; // N = " << d.N << " R = " << d.R << " sigma = " << d.sigma << std::endl;

            {
            tbb::mutex::scoped_lock lock(out_mutex);

            std::ofstream of_all, of_curr;
            of_all.open(FULL_FILE, std::ios_base::app);
            for (auto& p: (*results)[i])
            {
                for (auto& v: p)
                of_all << std::setprecision(15) << v << ", ";
                of_all << std::endl;
            }
            std::stringstream ss;
            ss << "errors_mc_" << d.confId << ".csv";
            of_curr.open(ss.str(), std::ios_base::app);
            for (auto& p: errors)
            {
                for (auto& v: p)
                    of_curr << std::setprecision(15) << v << ", ";
                of_curr << std::endl;
            }
            of_all.flush();
            of_curr.flush();
            }
        }
    
    }
    std::vector<Desc> descriptions;
    std::vector<std::vector<std::vector<double>>> *results;
    ParallelSimulator(decltype(results) results, const std::vector<Desc> &desc) :
        descriptions(desc), results(results)
    {
    }
};

#define NEL(A) ((sizeof(A)) / sizeof(A[0]))

void run()
{
    int N[] = {8, 16, 128, 1024};
    double R_checkPoints =
#ifdef METERS
        50
#else
        50.0 * 1e3
#endif
        ;
    int N_checkPoints = 65536;
    double sigma[] = {0.1, 0.5, 1.0, 2.0};
    double sigma_pos[] = {
#ifndef METERS
//    0.001,
 //     0.5 * 1e3,
      1.0  * 1e2
#else
//    0.001 * 1e-3,
//      0.5,
      0.1
#endif
    };
    double sigma_angle[] = {
//    0.001,
//      2.0,
        5.0
//      5.0
    };
    double R[] = { 
#ifndef METERS
        1e3 * 
#endif
            30.0 
    };
    double maxDelta = 6.0;
    int M = 1024;
    int Nn = NEL(N), sigman = NEL(sigma), Rn = NEL(R), sigmaPn = NEL(sigma_pos), sigmaAn = NEL(sigma_angle);

    std::ofstream log;
    log.open(FULL_FILE, std::ios_base::out);
    log << "R, N, sigma, sigmaPos, sigmaAngle, PointSet, ";
    log.flush();
    for (int i = 0; i < NPS; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            log << "d" << (char)('X' + j) << i << ", ";
        }
        for (int j = 0; j < 4; ++j)
        {
            log << "dQ" << (j < 3 ? (char)('X' + j) : 'W') << i << (i == 6 && j == 3 ? "" : ", ");
        }
#ifdef Q_ONLY
//        log << ", w";
#endif
    }
    log << ", rmse, max, rmseInitial, maxInitial";
    log << std::endl;

    int id = 0;
    std::vector<Desc> descriptions;
    for (int i = 0; i < Nn; ++i)
        for (int j = 0; j < sigman; ++j)
            for (int k = 0; k < Rn; ++k)
                for (int l = 0; l < sigmaPn; ++l)
                    for (int m = 0; m < sigmaAn; ++m)
            {
                std::cout << "MC: Creating N = " << N[i] << " R = " << R[k] << " sigma = " << sigma[j] << std::endl;
#if 0
                auto res = runExp(N[i], R[k], sigma[j], maxDelta, M);
                for (auto& r: res)
                {
                    for (int ii = 0; ii < r.size(); ++ii)
                        log << r[ii] << (ii + 1 == r.size() ? "" : ", ");
                    log << std::endl;
                }
#else
                std::ofstream of_curr;
            std::stringstream ss;
            ss << "errors_mc_" << id << ".csv";
            of_curr.open(ss.str(), std::ios_base::out);
            of_curr << "# N=" << N[i] << ", sigma = " << sigma[j] << ", R = " << R[k] << ", sigmaPos = " << sigma_pos[l] << ", sigmaAngle = " << sigma_angle[m] << ", R check = " << R_checkPoints << std::endl;
            of_curr << "x, y, z, dx, dy, dz, e, d, cntOk," << std::endl;
            of_curr.flush();
                for (int ii = 0; ii < M; ++ii)
                descriptions.push_back({ N[i], sigma[j], R[k], maxDelta, sigma_pos[l], sigma_angle[m], 1, N_checkPoints, R_checkPoints, id});
#endif
                id++;
            }
    int NN = descriptions.size();
    std::vector<std::vector<std::vector<double>>> results(NN);
    corecvs::parallelable_for(0, NN, ParallelSimulator(&results, descriptions));
#if 0
    for (auto& res: results)
                for (auto& r: res)
                {
                    for (int ii = 0; ii < r.size(); ++ii)
                        log << std::setprecision(15) <<  r[ii] << (ii + 1 == r.size() ? "" : ", ");
                    log << std::endl;
                }
#endif
            
}

void run_m15_pois()
{
    // 1. Read pois
    // 2. Read GT
    // 3. Run solver
    // 4. Compute reprojection & distance error
    
    auto map = parseGps("ps_coords_m15.txt");
    CalibrationJob jobC;
    JSONGetter getter("calibration.json");
    getter.visit(jobC, "job");
    auto pps = jobC.photostation;

    ReconstructionJob rec;
    rec.calibrationData = jobC;
    rec.fill(map, false, false);
    for (auto& p: rec.scene.photostations)
    {
        std::cout << "CHECKPS: " << p.location.position << " : " << p.location.orientation << std::endl;
    }
    rec.scene.pointObservations = parsePois("pois_m15.txt", true);

 //   exit(0);

    rec.solve(true);
    std::vector<double> errors;
    rec.scene.computeReprojectionErrors(errors);
    double ssq = 0.0;
    for (int i = 0; i < errors.size(); ++i)
        ssq += errors[i] * errors[i];
    std::cout << "ERROR: " << std::sqrt(ssq / (errors.size() / 2)) << std::endl;
    corecvs::Vector3dd meanPos(0.0, 0.0, 0.0);
    for (auto&p : rec.scene.photostations)
    {
        meanPos += p.location.position;
    }
    meanPos = meanPos * (1.0 / rec.scene.photostations.size());
    for (auto&o : rec.scene.pointObservations)
    {
        auto wp = o.worldPoint;
        auto wp2 = rec.scene.backProject(o.projections, o.updateable);
        std::cout << o.label << " GT: " << wp << " OBS: " << wp2 << " (" << (!(wp - wp2)/1e3) << ")" <<  ": " << (!(wp-wp2)/!(wp - meanPos)) * 100.0 << "% " << std::endl;
    }
    std::cout << "GT:" << std::endl;
    rec.solve(false);
    rec.scene.computeReprojectionErrors(errors);
    ssq = 0.0;
    for (int i = 0; i < errors.size(); ++i)
        ssq += errors[i] * errors[i];
    std::cout << "ERROR: " << std::sqrt(ssq / (errors.size() / 2)) << std::endl;
    meanPos = corecvs::Vector3dd(0.0, 0.0, 0.0);
    for (auto&o : rec.scene.pointObservations)
    {
        std::cout << "RE: " <<  o.label << ": ";
        for (auto&p : o.projections)
        {
            auto diff = rec.scene.computeReprojectionError(p.photostationId, p.cameraId, o.worldPoint, p.projection);
            std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << diff << " (" << !diff << ")]";
        }
        std::cout << std::endl;
    }
    for (auto&p : rec.scene.photostations)
    {
        meanPos += p.location.position;
    }
    meanPos = meanPos * (1.0 / rec.scene.photostations.size());
    double meanGt = 0.0, gtCnt = 0.0;
    for (auto&o : rec.scene.pointObservations)
    {
        auto wp = o.worldPoint;
        auto wp2 = rec.scene.backProject(o.projections, o.updateable);
        std::cout << o.label << " GT: " << wp << " OBS: " << wp2 << " (" << (!(wp - wp2)/1e3) << ")" <<  ": " << "[" << !(wp - meanPos) / 1e3 << " | " << (!(wp-wp2)/!(wp - meanPos)) * 100.0 << "% ]" << std::endl;
        meanGt += (!(wp-wp2)/!(wp - meanPos));
        gtCnt += 1.0;
    }
    std::cout << "GT:" << std::endl;
    double meanGtr = 0.0, gtrCnt = 0.0;
    for (auto&o2 : rec.scene.pointObservations)
    for (auto&o : rec.scene.pointObservations)
    {
        auto wp1 = o.worldPoint;
        auto wp2 = o2.worldPoint;
    if (wp1==wp2) continue;

        auto wp12 = rec.scene.backProject(o.projections, o.updateable);
        auto wp22 = rec.scene.backProject(o2.projections, o2.updateable);
        //std::cout << o.label << ":" << o2.label << " GT: " << wp1 << " OBS: " << wp2 << " (" << (!(wp12 - wp22)/1e3) << ")" <<  ": " << "[" << !(wp1 - wp2) / 1e3 << " | " << (!(wp12-wp22)/!(wp1 - wp2)) * 100.0 << "% ]" << std::endl;
        std::cout << "GTR: " << std::abs(!(wp12 - wp22)-!(wp1 - wp2))/!(wp1/2.0 + wp2/2.0 - meanPos)  << std::endl;
        meanGtr += std::abs(!(wp12 - wp22)-!(wp1 - wp2))/!(wp1/2.0 + wp2/2.0 - meanPos);
        gtrCnt += 1.0;
    }
    std::cout << "GTRM: " << meanGtr / gtrCnt * 100.0 << std::endl;
    std::cout << "GTM: " << meanGt / gtCnt * 100.0 << std::endl;
    std::cout << "GT:" << std::endl;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "CC" << i << " : ";
        std::cout << "focal :" << rec.scene.photostations[0].cameras[i].intrinsics.focal << " focal diff: " << (jobC.photostation.cameras[i].intrinsics.focal - rec.scene.photostations[0].cameras[i].intrinsics.focal) << " ";
        std::cout << "cxcy: " << rec.scene.photostations[0].cameras[i].intrinsics.principal << " cxcy  diff: " << (jobC.photostation.cameras[i].intrinsics.principal - rec.scene.photostations[0].cameras[i].intrinsics.principal) << " ";
        std::cout << "pos: " << rec.scene.photostations[0].cameras[i].extrinsics.position << " pos   diff: " << (jobC.photostation.cameras[i].extrinsics.position  - rec.scene.photostations[0].cameras[i].extrinsics.position) << " ";
        corecvs::Quaternion Q = jobC.photostation.cameras[i].extrinsics.orientation ^ rec.scene.photostations[0].cameras[i].extrinsics.orientation.conjugated();
        double phi = std::acos(Q[3]) * 2.0 * 180.0 / M_PI;
        corecvs::Vector3dd axis(Q[0], Q[1], Q[2]);
        axis.normalise();
        if (phi < 0)
        {
            phi = -phi;
            axis = -axis;
        }
        std::cout << "angle diff: " << axis << " | " << phi << std::endl;
    }
}

int main(int argc, char **argv)
{
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();


    std::string filenameCalibration = "calibration.json";
    std::string filenameGPS         = "gps.txt";
    std::string filenamePly         = "mesh.ply";

    if (argc > 1)
    {
        filenameCalibration = std::string(argv[1]);
    }
    if (argc > 2)
    {
        filenameGPS = std::string(argv[2]);
    }
    if (argc > 3)
    {
        filenamePly = std::string(argv[3]);
    }

    std::cout << "Reading calibration from " << filenameCalibration << std::endl <<
                 "Reading GPS data from " << filenameGPS << std::endl <<
                 "Saving mesh to " << filenamePly << std::endl;


#if 1
#if 1
    run();
#else
    run_m15_pois();
#endif
#else
    std::vector<int> p = {0, 1, 2};
    corecvs::Vector3dd arr[] =
    {
        corecvs::Vector3dd(1.0, 0.0, 0.0),
        corecvs::Vector3dd(0.0, 1.0, 0.0),
        corecvs::Vector3dd(0.0, 0.0, 1.0)
    };

    auto map = parseGps(filenameGPS);
    auto loc = map["SPA"];
    std::cout << "PS location: " << loc.position << std::endl;
    auto pps = GenerateModelPs();
    pps.location = loc;
    corecvs::Vector3dd poi1 = convertVector(corecvs::Vector3dd(507.97, 163.182, 175.785));
    corecvs::Vector2dd truth[] = 
    {
        corecvs::Vector2dd(2108, 508),
        corecvs::Vector2dd(1503, 527),
        corecvs::Vector2dd(906, 515),
    };
    int truth_id[] = 
    {
        10, 11, 12
    };
    double avg = 0.0;
    double mx = 0.0;
    int cnt = 0;
    for (int i = 0; i < pps.cameras.size(); ++i)
    {
        if (pps.isVisible(poi1, i))
        {
            std::cout << "SPA" << std::setw(2) << std::setfill('0') << (i + 1) << ": " << pps.project(poi1, i) << std::endl;
            for (int j = 0; j < 3; ++j)
            {
                if (truth_id[j] == i + 1)
                {
                    auto v = truth[j] - pps.project(poi1, i);
                    std:: cout << "DIFF: " << v << std::endl;
                    if (!v > mx)
                        mx = !v;
                    avg += v[0] * v[0] + v[1] * v[1];
                    cnt++;
                }
            }
        }
        else
        {
            std::cout << "SPA" << std::setw(2) << std::setfill('0') << (i + 1) << ": not visible" << std::endl;
        }
    }
    std::cout << "RMSE err: " << std::sqrt(avg / cnt) << std::endl;
    std::cout << "Max err: " << mx << std::endl;
    Mesh3D meshh;
    meshh.switchColor(true);
    pps.location = corecvs::LocationData();
    CalibrationHelpers().drawPly(meshh, pps, 1000.0);
    std::ofstream mofs;
    mofs.open("mesh_n.ply", std::ios_base::out);
    meshh.dumpPLY(mofs);

    ReconstructionJob rec;
    rec.fill(map, false);
    rec.scene.pointObservations = parsePois("pois_new.txt");
    std::vector<double> errors;
    rec.scene.computeReprojectionErrors(errors);
    double ssq = 0.0;
    mx = 0.0;
    for (int i = 0; i < errors.size(); i += 2)
    {
        ssq += errors[i]     * errors[i];
        ssq += errors[i + 1] * errors[i + 1];
        if (mx < std::abs(errors[i]))
            mx = std::abs(errors[i]);
        if (mx < std::abs(errors[i + 1]))
            mx = std::abs(errors[i + 1]);
    }
    std::cout << "Mean: " << std::sqrt(ssq / (errors.size() / 2)) << " max: " <<  mx << std::endl;
    rec.detectAll();
    rec.solve();
    std::cout << "X,Y,Z,Xe,Ye,Ze" << std::endl;
    for (int i = 0; i < rec.scene.photostations.size(); ++i)
    {
        std::stringstream pref;
        pref << "SP" << (char)('A' + (char)i);
        std::cout << pref.str() << ", ";
        std::cout <<
            rec.scene.photostations[i].location.position[0] << ", " <<
            rec.scene.photostations[i].location.position[1] << ", " <<
            rec.scene.photostations[i].location.position[2] << ", " <<
            map[pref.str()].position[0] << ", " << 
            map[pref.str()].position[1] << ", " <<
            map[pref.str()].position[2] << std::endl;
    }
    auto cc= rec.scene.photostations[0].location.orientation;
    for (int i = 0; i < rec.scene.photostations.size(); ++i)
    {
        auto o = cc.conjugated() ^ rec.scene.photostations[i].location.orientation;
        corecvs::Vector3dd axis(o[0], o[1], o[2]);
        axis.normalise();
        double phi = acos(o[3]) * 2.0 * 180.0 / M_PI;
        std::cout << axis << " " << phi << std::endl;
    }
#endif
    return 0;
#if 0
    CalibrationJob job;
    JSONGetter getter(filenameCalibration.c_str());
    getter.visit(job, "job");

    Photostation ps = job.photostation;

    Mesh3D mesh;
    mesh.switchColor(true);

    corecvs::Vector3dd sum(0.0, 0.0, 0.0), sumSq;
    for (auto& kp: map)
    {
        sum += kp.second.position;
        for (int i = 0; i < 3; ++i)
            sumSq[i] += kp.second.position[i]*kp.second.position[i];
    }
    sum /= (double)map.size();
    for (auto& kp: map)
    {
        std::cout << "KP: " << kp.second.position << " " << kp.second.orientation << std::endl;
        ps.location = kp.second;
        ps.location.position -= sum; // only for mesh output purposes
        std::cout << "Placing " << kp.first <<  ps.location.position << " " << ps.location.orientation << std::endl;
        CalibrationHelpers().drawPly(mesh, ps);
    }
    
    std::ofstream of;
    of.open(filenamePly, std::ios_base::out);
    mesh.dumpPLY(of);



    ReconstructionJob jobR;
    jobR.calibrationData = job;
    jobR.fill(map, false);
    //jobR.undistortAll();
    jobR.detectAll();
    jobR.solve();

    mesh = jobR.scene.drawPly();
    std::ofstream ofd;
    ofd.open("fitted.ply", std::ios_base::out);
    mesh.dumpPLY(ofd);
    
    JSONSetter setter("output.json");
    setter.visit(jobR.scene, "scene");
    return 0;
#endif
}
#else
int main(int, char**) { return 0; }
#endif
