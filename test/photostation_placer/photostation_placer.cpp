/*
 * This test takes photostation model from calibrationJob and arranges copies
 * of this PS using GPS coordinates
 *
 * NOTE: this one is not finalized yet and is more "playground" than "application"
 */
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

#include "calibrationHelpers.h"
#include "calibrationJob.h"
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
    return ps;
}

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
        ex.orientation = corecvs::Quaternion(0.0, 0.0, sin(M_PI / 4.0), cos(M_PI / 4.0));
    }
    return CameraModel(in, ex); 
}

corecvs::Photostation GenerateCanonPs()
{
    double F = 2250.0;
    double iW = 2896;
    double iH = 1944;
    
    auto cam = GenerateCamera(F, iW, iH, false);
    return GeneratePs(0.0, 24, cam);
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
        return 7 * scene.photostations.size() - 6;
    }
    void writeParams(double out[], corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0)) const
    {
        int N = scene.photostations.size();
        int argout = 0;
        for (int i = 0; i < N; ++i)
        {
            auto loc = scene.photostations[i].location;
            loc.orientation = loc.orientation.normalised();

            if (i > 1)
            {
                for (int j = 0; j < 3; ++j)
                    out[argout++] = (loc.position[j] - mean[j]) / scale[j];
            }
            for (int j = 0; j < 4; ++j)
                out[argout++] = loc.orientation[j];
        }
        CORE_ASSERT_TRUE_S(argout == getInputNum());
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
    }

    void readParams(const double in[], corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0))
    {
        int N = scene.photostations.size();
        int argin = 0;
        for (int i = 0; i < N; ++i)
        {
            auto& loc = scene.photostations[i].location;

            if (i > 1)
            {
                for (int j = 0; j < 3; ++j)
                    loc.position[j] = in[argin++] * scale[j] + mean[j];
            }
            for (int j = 0; j < 4; ++j)
                loc.orientation[j] = in[argin++];
            loc.orientation = loc.orientation.normalised();
        }
        CORE_ASSERT_TRUE_S(argin == getInputNum());
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
#if 10
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
    public:
        OptimizationFunctor(decltype(rJob) rJob, bool saturated = false, double saturationThreshold = 10, corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0)/*,corecvs::Matrix22 covariation = corecvs::Matrix22(1.0, 0.0, 0.0, 1.0)*/) : FunctionArgs(rJob->getInputNum(), saturated ? rJob->getOutputNum() / 2 : rJob->getOutputNum()), rJob(rJob), saturated(saturated), saturationThreshold(saturationThreshold), mean(mean), scale(scale) //, covariation(covariation)
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
                            CORE_ASSERT_TRUE_S(!std::isnan(result.a(j, k)));
                            CORE_ASSERT_TRUE_S(!std::isinf(result.a(j, k)));
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
            std::cout << "UPD..." << std::endl;
            rJob->scene.updateBackProjections();
            std::cout << "UPD...OK" << std::endl;
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
                    CORE_ASSERT_TRUE_S(!std::isnan(dx) && !std::isnan(dy));
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
                for (int i = 0; i < errors.size(); ++i)
                {
                    double err = out[outputPtr++] = std::abs(errors[i]);
                    total += err * err;
                    cnt += 0.5;
                    inliers_total += err * err;
                    if (i % 2 == 0)
                        inliers++;
                }
            }
            std::cout << "LMF: " << std::sqrt(total / cnt) << " | " << std::sqrt(inliers_total / inliers) << " (" << cnt << " | " << inliers << ")" << std::endl;
            CORE_ASSERT_TRUE_S(outputNum == outputPtr);
        }
    };
#endif

    void undistortAll(bool singleDistortion = true);
    void detectAll();
    void descriptAll();
    void solve()
    {
        corecvs::LevenbergMarquardt LM(10000000);
        corecvs::Vector3dd mean, scale;
        getScaler(mean, scale);
//        mean = 0.0;
//        scale = 1.0;
        std::cout << "MV: " << mean << " " << scale << std::endl;
        LM.f = new OptimizationFunctor(this, true,30.0,mean,scale);
        LM.trace = true;
        std::vector<double> in(getInputNum()), out(LM.f->outputs);
        writeParams(&in[0], mean, scale);
        auto res = LM.fit(in, out);
        readParams(&res[0], mean, scale);
#if 0
    scene.computeReprojectionErrors(errors);
    std::vector<double> errors;
#else
    OptimizationFunctor f(this, true, 30.0);
    std::vector<double> inf(getInputNum()), errors(f.outputs);
    writeParams(&inf[0]);
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

    sort(errors.begin(), errors.end());


    

    std::ofstream er;
    er.open("errors.csv", std::ios_base::out);
    for (int i = 0; i < errors.size(); i += 2)
    {
        er << errors[i] << ", " << errors[i + 1] << std::endl;
    }
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
    void fill(std::unordered_map<std::string, LocationData> &data, bool)
    {
        for (int id = 0; id < 7; ++id)
        {
            auto ps = GenerateCanonPs();
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
        location.position[0] = e * 1e3;
        location.position[1] = n * 1e3;
        location.position[2] = h * 1e3;
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



    std::vector<int> p = {0, 1, 2};
    corecvs::Vector3dd arr[] =
    {
        corecvs::Vector3dd(1.0, 0.0, 0.0),
        corecvs::Vector3dd(0.0, 1.0, 0.0),
        corecvs::Vector3dd(0.0, 0.0, 1.0)
    };

    auto map = parseGps(filenameGPS);

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
}
