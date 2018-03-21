/*
    Bayer to PPM converter
 */


#include <QApplication>
#include <avEncoder.h>
#include <iostream>
#include <time.h>

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

#ifdef WITH_OPENCV
#include "openCVTools.h"
#include "moc-flow/openCVMovingObjectFlow.h"
#endif
#include "core/framesources/imageCaptureInterface.h"


#include "aviCapture.h"
#include "modCostFunction.h"

#include <core/reflection/jsonPrinter.h>


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
    for (int i = 0; i < MODCostFunctionData::OMNIDIR_POWER; i++)
    {
        cout << "res " << result.projectionN[i] << " <-> " << data.projection.mN[i] << endl;
        data.projection.mN[i] = result.projectionN[i];
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
        SYNC_PRINT(("Give me input\n"));       
        SYNC_PRINT((" fisheye_egomotion --caps\n"));
        SYNC_PRINT((" fisheye_egomotion [--provider=<flow name>] [--frames=<n>] [--joint] [--scene] [--skipf=<n>] [--unwrap] [--intrinsics=<filename>] <aviname>\n"));
        SYNC_PRINT(("   --provider which flow provider to use. For options see --caps \n"));
        SYNC_PRINT(("   --frames   how many frames to process \n"));
        SYNC_PRINT(("   --scene    create scene to deeply analise data \n"));
        SYNC_PRINT(("   --joint    try to optimise projection with flow\n"));
        SYNC_PRINT(("Give me input\n"));

        return 0;
    }

    /*Process command line */
    CommandLineSetter s(argc, argv);
    int    maxFrame   = s.getInt("frames", 10);
    int    skipFrames = s.getInt("skipf", 1);

    if (s.getBool("caps"))
    {
        BufferFactory::getInstance()->printCaps();
        Processor6DFactoryHolder::printCaps();
        return 0;
    }

    std::string provider = s.getString("provider", defaultProvider);
    bool produceUnwarp   = s.getBool("unwarp");
    bool jointCost       = s.getBool("joint");
    bool createScene     = s.getBool("scene");
    std::string intrName = s.getString("intrinsics", "");

    double ransacThreshold     = s.getDouble("rthreshold", 0.009);
    double detectThreshold     = s.getDouble("dthreshold", 0.009);


    /* Prepare input */
    vector<string> stream = s.nonPrefix();
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

    while (framecount < maxFrame)
    {
        Statistics stats;
        stats.setValue("Frame Number", framecount);
        framecount++;

        for (int i=0; i < skipFrames; i++)
        {
            images.freeBuffers();
            images = cam->getFrameRGB24();
        }
        unique_ptr<RGB24Buffer> curFrame(images.rgbBufferDefault());
        images.setRgbBufferDefault(NULL); // Unlink from images

        if (prevFrame == NULL)
        {            
            prevFrame = std::move(curFrame);
            continue;
        }

        if (!newOutVideo.open) {
            newOutVideo.startEncoding("new.avi", curFrame->h, curFrame->w);
        }

        if (!newEsseVideo.open) {
            newEsseVideo.startEncoding("esse.avi", curFrame->h, curFrame->w);
        }


        Processor6D *flowGen = Processor6DFactoryHolder::getProcessor(provider);
        if (flowGen != NULL)
        {
            SYNC_PRINT(("Creating %s provider. Success\n", provider.c_str()));
        }

        stats.enterContext("Mesh Flow->");
        flowGen->requestResultsi(Processor6D::RESULT_FLOW | Processor6D::RESULT_FLOAT_FLOW_LIST);
        flowGen->setStats(&stats);
        flowGen->beginFrame();
        flowGen->setFrameRGB24(Processor6D::FRAME_RIGHT_ID, prevFrame.get());
        flowGen->endFrame();
        flowGen->beginFrame();
        flowGen->setFrameRGB24(Processor6D::FRAME_RIGHT_ID, curFrame.get());
        flowGen->endFrame();

        CorrespondenceList *flowList = flowGen->getFlowList();
        SYNC_PRINT(("Flow density %d\n", flowList->size()));

        if (flowList == NULL)
        {
            SYNC_PRINT(("Flow not generated\n "
                        "Exiting\n"));
            return 4;
        }

        stats.leaveContext();

        unique_ptr<RGB24Buffer> outputRaw (new RGB24Buffer(curFrame.get() ));
        unique_ptr<RGB24Buffer> outputEsse(new RGB24Buffer(curFrame.get() ));
        unique_ptr<RGB24Buffer> outputNew (new RGB24Buffer(curFrame.get() ));

        if (!modelLoaded) {
            unwrapper.model = getModel(curFrame->h, curFrame->w, intrName);
            modelLoaded = true;
        }
        /* Unwrap */

        if (!unwarpOutVideo.open) {
            unwarpOutVideo.startEncoding("unwrap.avi", unwrapper.unwrapSize.y(), unwrapper.unwrapSize.x());
        }

        unique_ptr<RGB24Buffer> unwarpOrig;
        unique_ptr<RGB24Buffer> unwarpRaw;
        unique_ptr<RGB24Buffer> unwarpNew;

        if (produceUnwarp)
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

            if (produceUnwarp) {
                unwrapper.drawUnwrap(unwarpRaw.get(), data0, data1, color);
            }

        }

        stats.resetInterval("Draw flow over unwrapped");

        /**/

        CorrespondenceList list;

        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            Vector3dd ray0 = unwrapper.model.intrinsics->reverse(data0);
            Vector3dd ray1 = unwrapper.model.intrinsics->reverse(data1);

            if (ray0.z() < 0.15) {
                continue;
            }
            if (ray1.z() < 0.15) {
                continue;
            }

            ray0 = ray0.normalisedProjective();
            ray1 = ray1.normalisedProjective();

            list.push_back(Correspondence(ray0.xy(), ray1.xy()));
        }

        cout << "Starting list size:" <<  list.size() << endl;

        stats.resetInterval("Created estimator input");

        /* Estimator */
        RansacEstimator estimator(8, 100, ransacThreshold);
        estimator.ransacParams.setUseMedian(true);
        estimator.trace = true;

        vector<Correspondence *> flowInput;
        flowInput.reserve(list.size());
        for (size_t d = 0 ; d < list.size(); d++)
        {
            flowInput.push_back(&list[d]);
        }

        EssentialMatrix matrix = estimator.getEssentialRansac(&flowInput);
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

        EssentialDecomposition decomp = matrix.decompose(&flowInput,decomposition);
        cout << decomp << endl;

        /* Draw result */
        stats.resetInterval("Estimator and decompostion");

        Affine3DQ move = decomp.toSecondCameraAffine();
        position = move * position;
        trajectory.mulTransform(position);
        trajectory.addOrts(0.5);
        trajectory.popTransform();

        /**
         *    Draw debug for essential estimation
         **/

        Vector3dd direction = decomp.direction;
        if (direction.z() < 0) direction = - direction;
        Vector2dd vanish = unwrapper.model.intrinsics->project(direction);
        cout << "Vanish point: "  << vanish << endl;

        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            Vector3dd ray0 = unwrapper.model.intrinsics->reverse(data0);
            Vector3dd ray1 = unwrapper.model.intrinsics->reverse(data1);

            ray0 = ray0.normalisedProjective();
            ray1 = ray1.normalisedProjective();

            double outlay = matrix.epipolarDistance(ray0.xy(), ray1.xy());

            RGBColor color = RGBColor::Gray();

            if (outlay > detectThreshold) {
                color = RGBColor::rainbow1((outlay - detectThreshold) * 100);
            }

            outputNew->drawLine(data0, data1, color);
            outputNew->drawCrosshare1(data1, color);

            if (produceUnwarp) {
                unwrapper.drawUnwrap(unwarpNew.get(), data0, data1, color);
            }
        }

        for (int r = 4; r < 10; r++) {
            outputNew->drawArc(Circle2d(vanish, r), RGBColor::Yellow());
        }

        if (createScene && framecount == 2)
        {
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

            ImageRelatedData *image1 = scene.createImageData();
            image1->mImagePath = "prev.bmp";
            cam1->addImageToCamera(image1);
            BufferFactory::getInstance()->saveRGB24Bitmap(prevFrame.get(),  image1->mImagePath);

            ImageRelatedData *image2 = scene.createImageData();
            image2->mImagePath = "curr.bmp";
            cam2->addImageToCamera(image2);
            BufferFactory::getInstance()->saveRGB24Bitmap(curFrame.get(),  image2->mImagePath);


            for (size_t corrId = 0; corrId < flowInput.size(); corrId++) {
                SceneFeaturePoint *point = scene.createFeaturePoint();

                SceneObservation observation1(cam1, point, Vector2dd::Zero());
                point->observations[cam1] = observation1;
                point->observations[cam1].setUndist(flowInput[corrId]->start);

                SceneObservation observation2(cam2, point, Vector2dd::Zero());
                point->observations[cam2] = observation2;
                point->observations[cam2].setUndist(flowInput[corrId]->end);
            }

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

        if (produceUnwarp)
        {
            oname.str("");
            oname << "unwarp-new" << framecount << extention;
            BufferFactory::getInstance()->saveRGB24Bitmap(unwarpNew.get(),  oname.str());
            unwarpOutVideo.addFrame(unwarpNew.get());
        }

        /* Joint cost function */
        if (jointCost)
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

        delete_safe(flowGen);
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

     if (jointCost)
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
