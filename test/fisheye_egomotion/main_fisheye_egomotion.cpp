/*
    Bayer to PPM converter
 */


#include <QApplication>
#include <avEncoder.h>
#include <iostream>
#include <time.h>

#include "core/utils/global.h"
#include "core/rectification/ransacEstimator.h"
#include "core/geometry/mesh3d.h"
#include "core/cameracalibration/calibrationDrawHelpers.h"

#include <core/buffers/rgb24/abstractPainter.h>

#include "core/stereointerface/processor6D.h"
#include "core/buffers/bufferFactory.h"
#include "core/cameracalibration/cameraModel.h"
#include "core/buffers/converters/errorMetrics.h"
#include "core/reflection/commandLineSetter.h"
#include "core/utils/utils.h"
#include "qtFileLoader.h"
#include "core/cameracalibration/ilFormat.h"
#include "core/buffers/remapBuffer.h"

#include "core/camerafixture/fixtureScene.h"
#include "core/camerafixture/cameraFixture.h"
#include "xml/generated/fisheyeEgomotionParameters.h"

#ifdef WITH_OPENCV
#include "openCVTools.h"
#include "moc-flow/openCVMovingObjectFlow.h"
#endif
#include "core/framesources/imageCaptureInterface.h"


#include "aviCapture.h"
#include "modCostFunction.h"

#include <core/reflection/jsonPrinter.h>
#include <core/reflection/usageVisitor.h>


using namespace std;
using namespace corecvs;



#ifdef WITH_CVS_FEATURES
extern "C"
{
    extern void init_cvs_stereo_provider();
    extern void init_cvs_detectors_provider();
    extern void init_cvs_descriptors_provider();
    extern void init_cvs_matchers_provider();
}
#endif

class MOCProcessor : public ImageInterfaceReceiver
{
public:
    virtual void newFrameReadyCallback(frame_data_t frameData) override
    {
        SYNC_PRINT(("MOCProcessor::newFrameReadyCallback(frame_data_t): called\n"));
    }

    virtual void newImageReadyCallback() override
    {
        SYNC_PRINT(("MOCProcessor::newImageReadyCallback(): called\n"));
    }

    virtual void newStatisticsReadyCallback(CaptureStatistics stats) override
    {
        SYNC_PRINT(("MOCProcessor::newStatisticsReadyCallback(CaptureStatistics): called\n"));
    }

    virtual void streamPausedCallback() override
    {
        SYNC_PRINT(("MOCProcessor::streamPausedCallback(): called\n"));
    }
};



#if 0
int main1(int argc, char **argv)
{
    AVEncoder encoder;
    RGB24Buffer buffer(100,100);

    AVEncoder::printCaps();

    encoder.startEncoding("out.avi", buffer.h, buffer.w);
    for (int i = 0; i < 250; i++)
    {
        buffer.checkerBoard(i % 99, RGBColor::Yellow(), RGBColor::Brown());
        AbstractPainter<RGB24Buffer> p(&buffer);
        p.drawFormat(0,0, RGBColor::Navy(), 2, "%d", i);
        encoder.addFrame(&buffer);
    }
    encoder.endEncoding();
    SYNC_PRINT(("Done."));
}
#endif

CameraModel getModel(int h, int w, std::string filename)
{
    std::string input =
    "omnidirectional\n"
    "604 1.58474 1.07055 3 1.14549 -0.233596 -0.125904\n";

    std::istringstream ss(input);

    OmnidirectionalProjection projection;

    projection.setSizeX(w);
    projection.setSizeY(h);
    projection.setDistortedSizeX(w);
    projection.setDistortedSizeY(h);

    projection.setPrincipalX(957.28999999999996);
    projection.setPrincipalY(648.84000000000003);
    projection.setFocal(689.58076000000005);
    projection.setN({
         -0.23202500000000001,
         -0.472026,
          0.52235399999999998,
          -0.25007699999999999});


    CameraModel model;
    if (filename.empty()) {
        model.intrinsics.reset(ILFormat::loadIntrisics(ss));
    } else {
        model.intrinsics.reset(ILFormat::loadIntrisics(filename));
    }
    model.setLocation(Affine3DQ::Identity());

    return model;
}

class Unwrapper {
public:
    PlaneFrame frame;
    Vector2dd unwrapSize;
    CameraModel model;

    RemapBuffer *remapCache = NULL;

    Unwrapper() {
        frame.e1 = Vector3dd::OrtX() * 3;
        frame.e2 = Vector3dd::OrtY() * 3;
        frame.p1 = Vector3dd(-1.5, -1.5, 1.0);

        unwrapSize = Vector2dd(1000, 1000);
    }

    ~Unwrapper() {
        delete_safe(remapCache);
    }

    RGB24Buffer* unwrap(RGB24Buffer* input, bool cache = true)
    {
        RGB24Buffer* unwrap = new RGB24Buffer(unwrapSize.y(), unwrapSize.x());

        if (cache && remapCache != NULL)
        {
            SYNC_PRINT(("RGB24Buffer* unwrap(): using cache\n"));
            for (int i = 0; i < unwrap->h; i++)
            {
                for (int j = 0; j < unwrap->w; j++)
                {
                    Vector2dd p = remapCache->element(i,j);

                    if (input->isValidCoordBl(p)) {
                        unwrap->element(i,j) = input->elementBl(p);
                    }
                }
            }
            return unwrap;
        }

        if (cache && (remapCache == NULL))
        {
            SYNC_PRINT(("RGB24Buffer* unwrap(): Creating cache buffer\n"));
            remapCache = new RemapBuffer(unwrap->getSize());
        }

        for (int i = 0; i < unwrap->h; i++)
        {
            for (int j = 0; j < unwrap->w; j++)
            {
                Vector2dd c((double)j / unwrap->w, (double)i / unwrap->h);
                Vector3dd r = frame.getPoint(c);
                Vector2dd p = model.intrinsics->project(r);

                if (input->isValidCoordBl(p)) {
                    unwrap->element(i,j) = input->elementBl(p);
                    if (cache)
                    {
                        remapCache->element(i, j) = p;
                    }
                }
            }
        }

        return unwrap;
    }

    void drawUnwrap(RGB24Buffer *canvas, const Vector2dd &p0, const Vector2dd &p1, const RGBColor &color)
    {
        Vector3dd rd0 = model.intrinsics->reverse(p0);
        Vector3dd rd1 = model.intrinsics->reverse(p1);

        Vector2dd pf0 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd0, Vector3dd::Zero())) * unwrapSize;
        Vector2dd pf1 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd1, Vector3dd::Zero())) * unwrapSize;

        canvas->drawLine(pf0, pf1, color);
        canvas->drawCrosshare1(pf1, color);
    }

};

/* RANSAC*/
void jointOptimisationRANSAC(MODCostFunctionData &data)
{
    RansacParameters ransacParams;
    MODJointOptimisation context;

    Ransac<MODJointOptimisation> ransac(100, ransacParams);
    context.baseData = &data;
    vector<Correspondence *> dataPtr;
    ransac.data = &dataPtr;
    ransac.data->reserve(data.obseravtions.size());
    for (size_t i = 0; i < data.obseravtions.size(); i++)
    {
        ransac.data->push_back(&data.obseravtions[i]);
    }
    ransac.trace = true;
    ransac.setUseMedian(true);
    ransac.setInlierThreshold(0.0001);
    ransac.setIterationsNumber(40);

    MODModel result = ransac.getModelRansac(&context);

    cout << "jointOptimisationRANSAC() finishing" << result.transform[0] << endl;
    cout << "Polinom after before" << endl;
    data.projection.mN.resize(MODCostFunctionData::OMNIDIR_POWER);
    for (int i = 0; i < MODCostFunctionData::OMNIDIR_POWER; i++)
    {
        cout << "res " << result.projectionN[i] << " <-> " << data.projection.mN[i] << endl;
        data.projection.mN[i] = result.projectionN[i];
    }

    /* Output test scene */
    FixtureScene scene;

    CameraFixture *frameFixture = scene.createCameraFixture();

    FixtureCamera *zero = scene.createCamera();

    zero->intrinsics.reset(data.projection.clone());
    zero->setLocation(Affine3DQ::Identity());

    scene.addCameraToFixture(zero, frameFixture);

    Affine3DQ totalTransform = Affine3DQ::Identity();

    for (size_t i = 0; i < data.transform.size(); i++)
    {
        totalTransform = data.transform[i] * totalTransform;
        FixtureCamera *cam = scene.createCamera();
        cam->copyModelFrom(data.projection.clone());
        cam->setLocation(totalTransform);
        scene.addCameraToFixture(cam, frameFixture);
    }

#if 0
    for (size_t corrId = 0; corrId < flowInput.size(); corrId++) {
        SceneFeaturePoint *point = scene.createFeaturePoint();

        SceneObservation observation1(zero, point, Vector2dd::Zero(), frameFixture);
        point->observations[zero] = observation1;
        point->observations[zero].setUndist(flowInput[corrId]->start);

        SceneObservation observation2(cam2, point, Vector2dd::Zero(), frameFixture);
        point->observations[cam2] = observation2;
        point->observations[cam2].setUndist(flowInput[corrId]->end);
    }
#endif


    {
        JSONPrinter printer("joint.json");
        printer.visit(scene, "scene");
    }


}

void jointOptimisation(MODCostFunctionData &data)
{

    MODCostFunction          cost(&data);
    MODCostFunctionNormalise normalise(&data);


    LevenbergMarquardt LMfit;
    LMfit.f = &cost;
    LMfit.normalisation = &normalise;
    LMfit.maxIterations = 250;
    LMfit.trace = true;
    LMfit.traceProgress = true;
    vector<double> input;
    vector<double> output;
    vector<double> result;


    input.resize (cost.inputs);
    output.resize(cost.outputs, 0);

    for (int i = 0; i < MODCostFunctionData::OMNIDIR_POWER; i++)
        input[i] = data.projection.mN[i];

    for (size_t i = 0; i < data.transform.size(); i++)
    {
        Affine3DQ guess = data.transform[i];
        guess.shift.storeTo(&input[i * MODCostFunctionData::MOVE_MODEL + MODCostFunctionData::OMNIDIR_POWER]);
        guess.rotor.storeTo(&input[i * MODCostFunctionData::MOVE_MODEL + MODCostFunctionData::OMNIDIR_POWER + 3]);
    }

    result = LMfit.fit(input, output);

    for (int i = 0; i < MODCostFunctionData::OMNIDIR_POWER; i++)
    {
        cout << "res " << result[i] << " <-> " << data.projection.mN[i] << endl;
    }

    Affine3DQ rguess;
    rguess.shift.loadFrom(&result[MODCostFunctionData::OMNIDIR_POWER]);
    rguess.shift.loadFrom(&result[MODCostFunctionData::OMNIDIR_POWER + 3]);

    cout << result;
}

bool checkIfStatic(CorrespondenceList *flowList, FisheyeEgomotionParameters &params)
{
    int fitCount = 0;
    for (size_t i = 0; i < flowList->size(); i++)
    {
        Correspondence &c = flowList->at(i);
        if ((c.start - c.end).l2Metric() < params.rsthreshold())
        {
            fitCount++;
        }
    }

    if (fitCount > flowList->size() * params.rsthresprec() / 100.0 )
    {
        return true;
    }

    return false;
}

int main(int argc, char **argv)
{

#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif
    QTG12Loader::registerMyself();
    QTRGB24Loader::registerMyself();
    QTRGB24Saver::registerMyself();

    std::string defaultProvider = "";

#ifdef WITH_CVS_FEATURES
    init_cvs_stereo_provider();
    defaultProvider = "CVS";
#endif
#ifdef WITH_OPENCV
    corecvs::Processor6DFactoryHolder::getInstance()->registerProcessor(new OpenCVMovingObjectFlowImplFactory);
    defaultProvider = "MeshFlow";
#endif

    if (argc <= 1) {
        printf(("Usage:\n"));
        printf((" fisheye_egomotion [<options>]\n"));
        UsagePrinter::printUsage(&FisheyeEgomotionParameters::reflection);
        printf(("\n"));
        printf(("Example\n"));
        printf((" fisheye_egomotion --frames=30 --skipf=0 --intrinsics=new-intr.txt --rthreshold=0.0032 --dthreshold=0.0034 --scene --unwarp --subpixel --provider=CVS /media/workarea/work/ADAS/mod2/video_svm_ab_front.h264"));
        return 0;
    }

    /*Process command line */
    CommandLineSetter s(argc, argv);
    FisheyeEgomotionParameters params;
    params.accept(s);

    cout << "We will use following parameters" << endl;
    cout << params << endl;

    if (params.caps())
    {
        BufferFactory::getInstance()->printCaps();
        Processor6DFactoryHolder::printCaps();
        return 0;
    }

    /* Prepare input */
    vector<string> stream = s.nonPrefix();
    cout << "We would open file:" << stream.back() << endl;
    ImageCaptureInterface *cam = new AviCapture(stream.back());

    /* Open input device */
    MOCProcessor processor;
    cam->imageInterfaceReceiver = &processor;
    cam->initCapture();
    cam->startCapture();


    int framecount = 0;
    unique_ptr<RGB24Buffer> prevFrame;
    ImageCaptureInterface::FramePair images;


    AVEncoder newOutVideo;
    AVEncoder newEsseVideo;
    AVEncoder unwarpOutVideo;

    Unwrapper unwrapper;
    bool modelLoaded = false;

    /* This need to be stabilised */
    Mesh3D trajectory;
    trajectory.switchColor();
    Affine3DQ position = Affine3DQ::Identity();

    /* Collect data */
    MODCostFunctionData dataForOpt;

    unique_ptr<Processor6D> flowGen(Processor6DFactoryHolder::getProcessor(params.provider()));
    if (flowGen != NULL)
    {
        SYNC_PRINT(("Creating %s provider. Success\n", params.provider().c_str()));
        if (params.traceParams())
        {
            std::map<std::string, DynamicObject> paramList = flowGen->getParameters();
            for(auto &it : paramList)
            {
                cout << "==== " << it.first << endl;
                cout << it.second << endl;
            }
        }

    }

    int requestedResults = Processor6D::RESULT_FLOW | Processor6D::RESULT_FLOAT_FLOW_LIST;
    if (params.subpixel()) {
        requestedResults |= Processor6D::RESULT_FLOAT_FLOW;
    }
    flowGen->requestResultsi(requestedResults);

    while (framecount < params.frames())
    {
        Statistics stats;
        stats.setValue("Frame Number", framecount);
        framecount++;

        for (int i=0; i <= params.skipf(); i++)
        {
            images.freeBuffers();
            images = cam->getFrameRGB24();
        }
        unique_ptr<RGB24Buffer> curFrame(images.rgbBufferDefault());
        images.setRgbBufferDefault(NULL); // Unlink from images

        if (prevFrame == NULL)
        {
            cout << "No previous frame" << endl;
            prevFrame = std::move(curFrame);
            continue;
        }

        if (!newOutVideo.open) {
            newOutVideo.startEncoding("new.avi", curFrame->h, curFrame->w);
        }

        if (!newEsseVideo.open) {
            newEsseVideo.startEncoding("esse.avi", curFrame->h, curFrame->w);
        }

        stats.enterContext("Mesh Flow->");

        flowGen->setStats(&stats);

/*        flowGen->reset();
        flowGen->beginFrame();
        flowGen->setFrameRGB24(Processor6D::FRAME_RIGHT_ID, prevFrame.get());
        flowGen->endFrame();*/
        flowGen->beginFrame();
        flowGen->setFrameRGB24(Processor6D::FRAME_RIGHT_ID, curFrame.get());
        flowGen->endFrame();

        CorrespondenceList *flowList = flowGen->getFlowList();

        if (flowList == NULL)
        {
            SYNC_PRINT(("Flow not generated\n "
                        "Exiting\n"));
            continue;
        }

        SYNC_PRINT(("Flow density %d\n", (int)flowList->size()));
#if 0
        for (size_t flowId = 0; flowId < std::min((int)flowList->size(), 10); flowId++)
        {
            cout << flowList->at(flowId) << endl;
        }
#endif
        stats.leaveContext();

        unique_ptr<RGB24Buffer> outputRaw (new RGB24Buffer(curFrame.get() ));
        unique_ptr<RGB24Buffer> outputEsse(new RGB24Buffer(curFrame.get() ));
        unique_ptr<RGB24Buffer> outputNew (new RGB24Buffer(curFrame.get() ));

        if (!modelLoaded) {
            unwrapper.model = getModel(curFrame->h, curFrame->w, params.intrinsics());
            modelLoaded = true;
        }
        /* Unwrap */

        if (!unwarpOutVideo.open) {
            unwarpOutVideo.startEncoding("unwrap.avi", unwrapper.unwrapSize.y(), unwrapper.unwrapSize.x());
        }

        unique_ptr<RGB24Buffer> unwarpOrig;
        unique_ptr<RGB24Buffer> unwarpRaw;
        unique_ptr<RGB24Buffer> unwarpNew;

        if (params.unwarp())
        {
            Mesh3D mesh;

            CalibrationDrawHelpers helpers;
            helpers.drawCamera(mesh, unwrapper.model, 1.0);

            stats.startInterval();
            unwarpOrig.reset(unwrapper.unwrap(curFrame.get()));

            mesh.addPlaneFrame(unwrapper.frame);

            mesh.dumpPLY("unwrap.ply");
            unwarpRaw.reset(new RGB24Buffer(unwarpOrig.get()));
            unwarpNew.reset(new RGB24Buffer(unwarpOrig.get()));
        }
        stats.resetInterval("Created Unwrapped Image");

        /* Debug draw */
        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            uint8_t status = flowList->at(i).value;

            RGBColor color = RGBColor::Red();
            switch (status) {
                case 0: color = RGBColor::Green(); break;
                case 1: color = RGBColor::Red(); break;
                case 2: color = RGBColor::Blue(); break;
                default:
                    break;
            }

            outputRaw->drawLine(data0, data1, color);
            outputRaw->drawCrosshare1(data1, color);

            if (params.unwarp()) {
                unwrapper.drawUnwrap(unwarpRaw.get(), data0, data1, color);
            }

        }

        stats.resetInterval("Draw flow over unwrapped");

        /**/

        CorrespondenceList list;

        int skipped = 0;
        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            Vector3dd ray0 = unwrapper.model.intrinsics->reverse(data0);
            Vector3dd ray1 = unwrapper.model.intrinsics->reverse(data1);

            if (ray0.z() < 0.15 && ray1.z() < 0.15) {
                skipped++;
                continue;
            }
            ray0 = ray0.normalisedProjective();
            ray1 = ray1.normalisedProjective();

            list.push_back(Correspondence(ray0.xy(), ray1.xy()));
        }

        cout << "Starting list size:" <<  list.size() << " we skipped" << skipped << endl;

        stats.resetInterval("Created estimator input");

        /* Estimator */
        Affine3DQ move = Affine3DQ::Identity();
        Vector2dd vanish = Vector2dd::Zero();
        EssentialMatrix matrix;
        EssentialDecomposition decomp;

        bool isStatic  = checkIfStatic(flowList, params);
        stats.resetInterval("Checking for static");
        stats.setValue("static", isStatic);

        if (!isStatic)
        {
            RansacEstimator estimator(8, 100, params.rthreshold());
            estimator.ransacParams.setUseMedian(true);
            estimator.trace = true;

            vector<Correspondence *> flowInput;
            flowInput.reserve(list.size());
            for (size_t d = 0 ; d < list.size(); d++)
            {
                flowInput.push_back(&list[d]);
            }

            matrix = estimator.getEssentialRansac(&flowInput);
            stats.setValue("Ransac inliers %", estimator.fitPercent);
            cout << matrix << endl;


            {
                cout << "List size:" << list.size() << endl;
                for (size_t count = 0; count < list.size(); count++)
                {
                    Correspondence &c = list[count];

                    RGBColor color = RGBColor::Red();
                    if (c.flags & Correspondence::FLAG_FAILER) {
                        color = RGBColor::Red();
                    }
                    if (c.flags & Correspondence::FLAG_PASSER) {
                        color = RGBColor::Green();
                    }
                    if (c.flags & Correspondence::FLAG_IS_BASED_ON) {
                        color.b() = 255;
                    }
                    Vector3dd r0(c.start.x(), c.start.y(), 1.0);
                    Vector3dd r1(c.end  .x(), c.end  .y(), 1.0);

                    Vector2dd p0 = unwrapper.model.intrinsics->project(r0.normalised());
                    Vector2dd p1 = unwrapper.model.intrinsics->project(r1.normalised());

                    outputEsse->drawLine      (p0, p1, color);
                    outputEsse->drawCrosshare1(p1, color);
                }

                for (size_t count = 0; count < list.size(); count++)
                {
                    Correspondence &c = list[count];

                    bool base = false;
                    if (c.flags & Correspondence::FLAG_IS_BASED_ON) {
                        base = true;
                    }

                    Vector3dd r1(c.end  .x(), c.end  .y(), 1.0);
                    Vector2dd p1 = unwrapper.model.intrinsics->project(r1.normalised());

                    if (base) {
                         outputEsse->drawArc(p1.x(), p1.y(), 10, RGBColor::Magenta());
                         outputEsse->drawArc(p1.x(), p1.y(), 12, RGBColor::Amber());
                         outputEsse->drawArc(p1.x(), p1.y(), 8 , RGBColor::Blue());

                    }
                }
            }

            EssentialDecomposition decomposition[4];
            matrix.decompose(decomposition);

            decomp = matrix.decompose(&flowInput,decomposition);
            cout << decomp << endl;

            /* Draw result */
            stats.resetInterval("Estimator and decompostion");
            move = decomp.toSecondCameraAffine();

            /**
             *    Draw debug for essential estimation
             **/
            Vector3dd direction = decomp.direction;
            if (direction.z() < 0) direction = - direction;
            vanish = unwrapper.model.intrinsics->project(direction);
            cout << "Vanish point: "  << vanish << endl;

        }

        position = move * position;
        trajectory.mulTransform(position);
        trajectory.addOrts(0.5);
        trajectory.popTransform();

        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            Vector3dd ray0 = unwrapper.model.intrinsics->reverse(data0);
            Vector3dd ray1 = unwrapper.model.intrinsics->reverse(data1);

            ray0 = ray0.normalisedProjective();
            ray1 = ray1.normalisedProjective();

            RGBColor color = RGBColor::Gray();

            if (!isStatic)
            {
                double outlay = matrix.epipolarDistance(ray0.xy(), ray1.xy());
                if (outlay > params.dthreshold()) {
                    color = RGBColor::rainbow1((outlay - params.dthreshold()) * 100);
                }
            } else {
                color = RGBColor::Amber();
                double outlay = (data0 - data1).l2Metric();
                if (outlay > params.dsthreshold()) {
                    color = RGBColor::rainbow1((outlay - params.dsthreshold()) * 100);
                }
            }

            outputNew->drawLine(data0, data1, color);
            outputNew->drawCrosshare1(data1, color);

            if (params.unwarp()) {
                unwrapper.drawUnwrap(unwarpNew.get(), data0, data1, color);
            }
        }

        for (int r = 4; r < 10; r++) {
            outputNew->drawArc(Circle2d(vanish, r), RGBColor::Yellow());
        }

        if (params.scene() && framecount == 2)
        {
            SYNC_PRINT(("Outputting scene\n"));
            FixtureScene scene;

            CameraFixture *frameFixture = scene.createCameraFixture();

            FixtureCamera *cam1 = scene.createCamera();
            FixtureCamera *cam2 = scene.createCamera();

            cam1->copyModelFrom(unwrapper.model);
            cam1->setLocation(Affine3DQ::Identity());

            cam2->copyModelFrom(unwrapper.model);
            cam2->setLocation(move);

            scene.addCameraToFixture(cam1, frameFixture);
            scene.addCameraToFixture(cam2, frameFixture);

            ImageRelatedData *image1 = scene.addImageToCamera(cam1, "prev.bmp");
            BufferFactory::getInstance()->saveRGB24Bitmap(prevFrame.get(),  image1->mImagePath);

            ImageRelatedData *image2 = scene.addImageToCamera(cam2, "curr.bmp");
            BufferFactory::getInstance()->saveRGB24Bitmap(curFrame.get(),  image2->mImagePath);

            SYNC_PRINT(("Triangulating and saving points:\n"));
            for (size_t corrId = 0; corrId < flowList->size(); corrId++) {
                SceneFeaturePoint *point = scene.createFeaturePoint();

                Vector2dd start = flowList->operator [](corrId).start;
                Vector2dd end   = flowList->operator [](corrId).end;

                SceneObservation observation1(cam1, point, Vector2dd::Zero(), frameFixture);
                point->observations[cam1] = observation1;
                point->observations[cam1].setUndist(start);

                SceneObservation observation2(cam2, point, Vector2dd::Zero(), frameFixture);
                point->observations[cam2] = observation2;
                point->observations[cam2].setUndist(end);

                /* Trigger cache calculation */
                point->observations[cam1].getDist();
                point->observations[cam2].getDist();
                point->observations[cam1].getRay();
                point->observations[cam2].getRay();

                Vector2dd center = unwrapper.model.intrinsics->principal();
                if ((start - center).l2Metric() > unwrapper.model.intrinsics->size().minimum() / 2)
                {
                    continue;
                }

                bool ok = false;
                Vector3dd pos = point->triangulateByRays(&ok);
                if (ok) {

                    Vector3dd d1 = point->observations[cam1].getRay();
                    Vector3dd d2 = point->observations[cam2].getRay();

                    if (d1.angleTo(d2) < degToRad(0.1)) {
                        point->color = RGBColor::Red();
                    } else {
                        point->color = RGBColor::FromFloat((prevFrame->elementBl(start).toFloat() + curFrame->elementBl(end).toFloat()) / 2);
                    }
                    point->reprojectedPosition = pos;
                    point->hasKnownReprojectedPosition = true;
                } else {
                    point->observations.clear();
                    //point->reprojectedPosition = Vector3dd(std::numeric_limits<double>::quiet_NaN());
                    point->hasKnownReprojectedPosition = false;
                }

                if (corrId % 100 == 0) {
                    SYNC_PRINT(("\r%2.2lf%%", 100.0 * corrId / flowList->size()));
                }
            }
            SYNC_PRINT(("...done\n"));

            {
                JSONPrinter printer("testscene.json");
                printer.visit(scene, "scene");
            }
        }



        stats.resetInterval("Debug output");


        BaseTimeStatisticsCollector collector;
        collector.addStatistics(stats);
        collector.printAdvanced();

        collector.drawOSD(outputNew.get());

        std::string extention = ".jpg";
        std::stringstream oname;
        /*
        oname.str("");
        oname << "raw" << framecount << extention;
        BufferFactory::getInstance()->saveRGB24Bitmap(outputRaw.get(), oname.str());
        */

        oname.str("");
        oname << "new" << framecount << extention;
        BufferFactory::getInstance()->saveRGB24Bitmap(outputNew.get(), oname.str());
        newOutVideo.addFrame(outputNew.get());

        oname.str("");
        oname << "esse" << framecount << extention;
        BufferFactory::getInstance()->saveRGB24Bitmap(outputEsse.get(), oname.str());
        newEsseVideo.addFrame(outputNew.get());

        if (params.unwarp())
        {
            oname.str("");
            oname << "unwarp-new" << framecount << extention;
            BufferFactory::getInstance()->saveRGB24Bitmap(unwarpNew.get(),  oname.str());
            unwarpOutVideo.addFrame(unwarpNew.get());
        }

        /* Joint cost function */
        if (params.joint() && !isStatic)
        {
            Affine3DQ guess = decomp.toSecondCameraAffine();

            dataForOpt.transform.push_back(guess);
            dataForOpt.obseravtions.reserve(dataForOpt.obseravtions.size() + flowList->size());

            for(size_t i = 0; i < flowList->size(); i++)
            {
                dataForOpt.obseravtions.push_back(flowList->at(i));
                dataForOpt.obseravtions.back().value = dataForOpt.transform.size() - 1;
            }
        }

        prevFrame = std::move(curFrame);

    }

     if (newOutVideo.open)
     {
         newOutVideo.endEncoding();
     }
     if (unwarpOutVideo.open)
     {
         unwarpOutVideo.endEncoding();
     }

     trajectory.dumpPLY("trajectory.ply");

     if (params.joint())
     {
         dataForOpt.projection = *unwrapper.model.getOmnidirectional();
         jointOptimisationRANSAC(dataForOpt);

         cout << "New intrinsics" << endl;
         ILFormat::saveIntrisics(dataForOpt.projection, cout);
         ILFormat::saveIntrisics(dataForOpt.projection, "new-intr.txt");
         cout << "======" << endl;
     }


    //system("ls -all");


    return 0;
}
