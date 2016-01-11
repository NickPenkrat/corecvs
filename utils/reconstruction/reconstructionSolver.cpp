#include <vector>
#include <string>
#include <sstream>
#include <random>

#include "reconstructionSolver.h"
#include "levenmarq.h"
#include "pnpSolver.h"

//#define  METERS

#ifdef WITH_OPENCV
# include "openCvFileReader.h"  // inside it requires BufferReaderProvider
# include "openCvDescriptorExtractorWrapper.h"
# include "openCvFeatureDetectorWrapper.h"
# include "openCvDescriptorMatcherWrapper.h"
#else
# include "bufferReaderProvider.h"
#endif

//#define ESTIMATE_C
//#define ESTIMATE_FC
//#define ESTIMATE_F
//#define ESTIMATE_RP

int ReconstructionJob::getOutputNum() const
{
    std::vector<double> err;
    scene.computeReprojectionErrors(err);
    return (int)err.size();
}

int ReconstructionJob::getInputNum() const
{
    //int cams = 0;
#ifndef POI_ONLY
    return 7 * (int)scene.photostations.size() - 6;
#else
    return 
#ifndef Q_ONLY
        7 
#else
        4
#endif
        * (int)scene.photostations.size()
#ifdef ESTIMATE_FC
#ifdef ESTIMATE_F
        + scene.photostations[0].cameras.size()
#endif
#ifdef ESTIMATE_C
        + 2 * scene.photostations[0].cameras.size()
#endif
#endif
#ifdef ESTIMATE_RP
        + 7 * (scene.photostations[0].cameras.size() - 1)
#endif
        ;
#endif
}
    
void ReconstructionJob::writeParams(double out[], corecvs::Vector3dd mean, corecvs::Vector3dd scale) const
{
    int N = (int)scene.photostations.size();
    int argout = 0;
    for (int i = 0; i < N; ++i)
    {
        auto loc = scene.photostations[i].location;
        loc.rotor = loc.rotor.normalised();

#ifndef POI_ONLY
        if (i > 1)
#else
        if (1)
#endif
        {
#ifndef Q_ONLY
            for (int j = 0; j < 3; ++j)
                out[argout++] = (loc.shift[j] - mean[j]) / scale[j];
#endif
        }
        for (int j = 0; j < 4; ++j)
            out[argout++] = loc.rotor[j];
    }
#ifdef POI_ONLY
    for (int i = 0; i < scene.photostations[0].cameras.size(); ++i)
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
    
void ReconstructionJob::getScaler(corecvs::Vector3dd &mean, corecvs::Vector3dd &scale)
{
    int N = (int)scene.photostations.size();
    //int argin = 0;
    mean = corecvs::Vector3dd(0.0, 0.0, 0.0);
    scale = corecvs::Vector3dd(0.0, 0.0, 0.0);
    for (int i = 0; i < N; ++i)
    {
        auto& loc = scene.photostations[i].location.shift;
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
    
void ReconstructionJob::readParams(const double in[], corecvs::Vector3dd mean, corecvs::Vector3dd scale)
{
    int N = (int)scene.photostations.size();
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
                loc.shift[j] = in[argin++] * scale[j] + mean[j];
#endif
        }
        for (int j = 0; j < 4; ++j)
            loc.rotor[j] = in[argin++];
        loc.rotor = loc.rotor.normalised();
    }
#ifdef POI_ONLY
    for (int kk = 0; kk < scene.photostations[0].cameras.size(); ++kk)
    {
#ifdef ESTIMATE_FC
#ifdef ESTIMATE_F
        double f = in[argin++];
#endif
#ifdef ESTIMATE_C
        double cx = in[argin++];
        double cy = in[argin++];
#endif
        for (int i = 0; i < scene.photostations.size(); ++i)
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
        for (int i = 0; i < scene.photostations.size(); ++i)
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
        
void ReconstructionJob::OptimizationFunctor::operator() (const double in[], double out[])
{
    static double mx = 0.0;
    std::vector<double> inputs(in, in + rJob->getInputNum());
    rJob->readParams(in, mean, scale);
    std::vector<double> errors;
#ifndef NOUPD
    rJob->scene.updateBackProjections();
#endif
    rJob->scene.computeReprojectionErrors(errors);

    //int outputNum = FunctionArgs::outputs;
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

        for (uint i = 0; i < errors.size(); i += 2)
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
        //int id = 0;
        auto& scene = rJob->scene;
        if (angleError)
        {
            for (auto& po: scene.pointObservations)
            {
                auto wp = po.worldPoint;
                for (auto& proj: po.projections)
                {
                    auto cam = scene.photostations[proj.photostationId].getRawCamera(proj.cameraId);
                    auto dp = (cam.rayFromPixel(proj.projection)).a;
                    auto realDir = wp - cam.extrinsics.position;
                    out[outputPtr++] = realDir.angleTo(dp); 
                    total += out[outputPtr - 1] * out[outputPtr - 1];
                    inliers_total += out[outputPtr - 1] * out[outputPtr - 1];
                    cnt += 1;
                    inliers += 1;
                }
            }
        } else {
            for (uint i = 0; i < errors.size(); ++i)
            {
                double err = out[outputPtr++] = (errors[i]);
                total += err * err;
                cnt += 0.5;
                inliers_total += err * err;
                if (i % 2 == 0)
                    inliers++;
            }
        }
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
}

void ReconstructionJob::solve(bool angleError)
{
    corecvs::LevenbergMarquardt LM(10000);
    corecvs::Vector3dd mean, scale;
    getScaler(mean, scale);
//        mean = 0.0;
//        scale = 1.0;
    std::cout << "MV: " << mean << " " << scale << std::endl;
    LM.f = new OptimizationFunctor(this,false,30.0,mean,scale,angleError);
    LM.normalisation = new NormalizationFunctor(this, mean, scale);
    //LM.trace = true;
    std::vector<double> in(getInputNum()), out(LM.f->outputs);
    writeParams(&in[0], mean, scale);
    auto res = LM.fit(in, out);
    readParams(&res[0], mean, scale);
    return;
}    

void ReconstructionJob::undistortAll(bool singleDistortion)
{
    int M = scene.getPhotostationCount();
    std::vector<corecvs::DisplacementBuffer> transformations;
    for (int i = 0; i < M; ++i)
    {
        int N = (int)scene.photostations[i].cameras.size();
        if (i == 0 || !singleDistortion)
        {
            transformations.resize(N);
            corecvs::parallelable_for(0, N, ParallelUndistortionMapEstimator(&transformations, &calibrationData, &scene.photostations[i]));
        }
        corecvs::parallelable_for(0, N, ParallelUndistortionCalculator(&scene.cameraObservations[i], &transformations, &scene.photostations[i]));
    }
}

void ReconstructionJob::ParallelUndistortionCalculator::operator() (const corecvs::BlockedRange<int> &r) const
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
        
void ReconstructionJob::ParallelUndistortionMapEstimator::operator() (const corecvs::BlockedRange<int> &r) const
{
    for (int camId = r.begin(); camId < r.end(); ++camId)
    {
        auto& cam = (*photostation).cameras[camId];
        calibrationJob->prepareUndistortionTransformation(camId, (*transformations)[camId]);
    }
}
    
void ReconstructionJob::fill(std::unordered_map<std::string, corecvs::Affine3DQ> &data, size_t psLocationCnt)
{
    std::cout << "psLocationCnt: " << psLocationCnt << std::endl;

    for (int id = 0; id < psLocationCnt; ++id)
    {
        std::stringstream ss;
        ss << "SP" << (char)('A' + id);
        std::string prefix = ss.str();

        scene.photostations.push_back(calibrationData.photostation);
        scene.photostations.rbegin()->location = data[prefix];
 //       scene.photostations.rbegin()->location.orientation = scene.photostations.rbegin()->location.orientation;

        std::vector<CameraObservation> observations;
        for (int i = 0; i < (int)scene.photostations[id].cameras.size(); ++i)
        {
            CameraObservation observation;
            std::stringstream ss;
            ss << prefix << i << "_0deg.jpg";
            observation.sourceFileName = ss.str();

            ss.str("");
            ss << prefix << i << "_undist.jpg";
            observation.undistortedFileName = ss.str();

            std::cout << "Added: " << observation.undistortedFileName << std::endl;

            observations.push_back(observation);
        }
        scene.cameraObservations.push_back(observations);
    }
}

void ReconstructionJob::solveWithBadPOI(bool filter, bool forceGps)
{
    // 1. Read GT
    // 2. Run solver
    // 3. Compute reprojection & distance error

    double maxAngleError = 1.0;

    std::cout << "Solve: " << scene.photostations.size() << " " << scene.pointObservations.size() << std::endl;

    if (filter)
    {
        std::mt19937 rng;
        for (int i = 0; i < scene.photostations.size(); ++i)
        {
            std::vector<std::tuple<int, corecvs::Vector2dd, corecvs::Vector3dd>> pspa;
            for (auto &o: scene.pointObservations)
            {
                for (auto&p: o.projections)
                {
                    if (p.photostationId == i && !o.updateable)
                    {
                        pspa.emplace_back(p.cameraId, p.projection, o.worldPoint);
                    }
                }
            }

            double bhScore = 1e100;
            corecvs::Affine3DQ bh;
            for (int iii = 0; iii < 10000; ++iii)
            {
                decltype(pspa) psp;
                int N = (int)pspa.size();
                psp.resize(4);
                int ids[4];
                int idx = 0;
                ids[0] = rng() % N;
                while (idx < 4)
                {
                    int idd = rng() % N;
                    bool isValid = true;
                    for (int ii = 0; ii < idx; ++ii)
                        if (ids[ii] == idd)
                            isValid = false;
                    if (isValid)
                        ids[idx++] = idd;
                }
                for (int kk = 0; kk < 4; ++kk)
                {
                    psp[kk] = pspa[ids[kk]];
                }

            std::vector<corecvs::Vector3dd> pts, dirs,oris;
            for (auto& t: psp)
            {
                corecvs::Vector3dd dir = calibrationData.photostation.getRawCamera(std::get<0>(t)).rayFromPixel(std::get<1>(t)).a.normalised();
                corecvs::Vector3dd ori = calibrationData.photostation.getRawCamera(std::get<0>(t)).extrinsics.position;
                corecvs::Vector3dd pt  = std::get<2>(t);
                pts.push_back(pt);
                oris.push_back(ori);
                dirs.push_back(dir);
            }
            auto hyp = corecvs::PNPSolver::solvePNP(oris, dirs, pts);
            for (auto& h: hyp)
            {
                auto ps = calibrationData.photostation;
                ps.location = h;
                double score = 0.0;
                for (auto& t: pspa)
                {
                    auto cam = ps.getRawCamera(std::get<0>(t));
                    auto dp = (cam.rayFromPixel(std::get<1>(t)).a).normalised();
                    double angle = std::abs((std::get<2>(t) - cam.extrinsics.position).angleTo(dp)) * 180.0 / M_PI;
                    if (angle < maxAngleError && cam.isVisible(std::get<2>(t)))
                        score -= 1.0;// diff & diff;
                }
                if (score < bhScore)
                {
                    bhScore = score;
                    bh = h;
                }
            }
            }
            if (bhScore <= -3)
            {
                if (!forceGps)
                    scene.photostations[i].location = bh;
                else
                    scene.photostations[i].location.rotor = bh.rotor;
            }
        }
        std::cout << "ANGLES: " << std::endl;
        for (auto&o : scene.pointObservations)
        {
            std::cout << "AE: " <<  o.label << ": ";
            for (auto&p : o.projections)
            {
                auto cam = scene.photostations[p.photostationId].getRawCamera(p.cameraId);
                auto dp = (cam.rayFromPixel(p.projection)).a;
                double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
                std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
            }
            std::cout << std::endl;
        }
        decltype(scene.pointObservations) obsNew;
        for (auto&o : scene.pointObservations)
        {
            std::cout << "AE: " <<  o.label << ": ";
            auto on = o;
            on.projections.clear();
            for (auto&p : o.projections)
            {
                auto cam = scene.photostations[p.photostationId].getRawCamera(p.cameraId);
                auto dp = (cam.rayFromPixel(p.projection)).a;
                double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
                std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
                if (std::abs(angle) < maxAngleError && cam.isVisible(o.worldPoint))
                    on.projections.push_back(p);
            }
            if (on.projections.size() > 1)
                obsNew.push_back(on);
            std::cout << std::endl;
        }
        scene.pointObservations = obsNew;
    }
    // And refine positions
#if 1
    std::cout << "Solve: " << scene.photostations.size() << " " << scene.pointObservations.size() << std::endl;

    for (int i = 0; i < scene.photostations.size(); ++i)
    {
        std::vector<std::tuple<int, corecvs::Vector2dd, corecvs::Vector3dd>> pspa;
        for (auto &o: scene.pointObservations)
        {
            for (auto&p: o.projections)
            {
                if (p.photostationId == i && !o.updateable)
                {
                    pspa.emplace_back(p.cameraId, p.projection, o.worldPoint);
                }
            }
        }
        if (pspa.size() < 3)
            continue;

        double bhScore = 1e100;
        corecvs::Affine3DQ bh;
        std::vector<corecvs::Vector3dd> pts, dirs,oris;
        auto psp = pspa;
        for (auto& t: psp)
        {
            corecvs::Vector3dd dir = calibrationData.photostation.getRawCamera(std::get<0>(t)).rayFromPixel(std::get<1>(t)).a.normalised();
            corecvs::Vector3dd ori = calibrationData.photostation.getRawCamera(std::get<0>(t)).extrinsics.position;
            corecvs::Vector3dd pt  = std::get<2>(t);
            pts.push_back(pt);
            oris.push_back(ori);
            dirs.push_back(dir);
        }
        auto hyp = corecvs::PNPSolver::solvePNP(oris, dirs, pts);
        for (auto& h: hyp)
        {
            auto ps = calibrationData.photostation;
            ps.location = h;
            double score = 0.0;
            for (auto& t: pspa)
            {
                auto proj = ps.project(std::get<2>(t), std::get<0>(t));
                auto cam = ps.getRawCamera(std::get<0>(t));
                auto dp = (cam.rayFromPixel(std::get<1>(t)).a).normalised();
                auto diff = std::get<1>(t) - proj;
                score += diff & diff;
            }
            std::cout << "SP" << (char)('A' + i) << " : " << h.shift;
            h.rotor.printAxisAndAngle();
            std::cout << "SC: " << std::sqrt(score / pspa.size()) << std::endl;
            if (score < bhScore)
            {
                bhScore = score;
                bh = h;
            }
        }
        if (pspa.size() > 2)
        {
            if (!forceGps)
               scene.photostations[i].location = bh;
            else
                scene.photostations[i].location.rotor = bh.rotor;
        }
    }
#endif
    /*
     * Now we run solver 2 times: with geometrical error,
     * and continue optimization with graphical (e.g. reprojection)
     * error
     */
    // Geometrical error optimization

    solve(true);
    std::vector<double> errors;
    scene.computeReprojectionErrors(errors);
    double ssq = 0.0;
    for (uint i = 0; i < errors.size(); ++i)
        ssq += errors[i] * errors[i];
    std::cout << "ERROR: " << std::sqrt(ssq / (errors.size() / 2)) << std::endl;
    corecvs::Vector3dd meanPos(0.0, 0.0, 0.0);
    for (auto&p : scene.photostations)
    {
        meanPos += p.location.shift;
    }
    meanPos = meanPos * (1.0 / scene.photostations.size());
    for (auto&o : scene.pointObservations)
    {
        auto wp = o.worldPoint;
        auto wp2 = scene.backProject(o.projections, o.updateable);
        std::cout << o.label << " GT: " << wp << " OBS: " << wp2 << " (" << (!(wp - wp2)/1e3) << ")" <<  ": " << (!(wp-wp2)/!(wp - meanPos)) * 100.0 << "% " << std::endl;
    }
    std::cout << "ANGLES: " << std::endl;
    for (auto&o : scene.pointObservations)
    {
        std::cout << "AE: " <<  o.label << ": ";
        for (auto&p : o.projections)
        {
            auto cam = scene.photostations[p.photostationId].getRawCamera(p.cameraId);
            auto dp = (cam.rayFromPixel(p.projection)).a;
            double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
            std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
        }
        std::cout << std::endl;
    }

    std::cout << "GT:" << std::endl;
    solve(false);
}


