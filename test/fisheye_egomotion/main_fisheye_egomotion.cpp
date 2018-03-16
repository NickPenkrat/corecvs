/*
    Bayer to PPM converter
 */

#include <OpenCVTools.h>
#include <QApplication>
#include <avEncoder.h>
#include <iostream>
#include <time.h>

#include <core/rectification/ransacEstimator.h>

#include <core/geometry/mesh3d.h>

#include <core/cameracalibration/calibrationDrawHelpers.h>

#include <core/buffers/rgb24/abstractPainter.h>

#include "core/buffers/bufferFactory.h"
#include "core/cameracalibration/cameraModel.h"
#include "core/buffers/converters/errorMetrics.h"
#include "core/reflection/commandLineSetter.h"
#include "core/utils/utils.h"
#include "qtFileLoader.h"
#include "core/cameracalibration/ilFormat.h"


#include "core/framesources/imageCaptureInterface.h"
#include "aviCapture.h"


#include "moc-flow/openCVMovingObjectFlow.h"

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

#ifdef WITH_CVS_FEATURES
    init_cvs_stereo_provider();
#endif
    corecvs::Processor6DFactoryHolder::getInstance()->registerProcessor(new OpenCVMovingObjectFlowImplFactory);

    if (argc <= 1) {
        SYNC_PRINT(("Give me input\n"));       
        SYNC_PRINT((" fisheye_egomotion --caps\n"));
        SYNC_PRINT((" fisheye_egomotion [--provider=<flow name>] [frames=<n>] [--unwrap] [] <aviname>\n"));
        return 0;
    }

    /*Process command line */
    CommandLineSetter s(argc, argv);
    int maxFrame = s.getInt("frames", 10);

    if (s.getBool("caps"))
    {
        BufferFactory::getInstance()->printCaps();
        Processor6DFactoryHolder::printCaps();
        return 0;
    }

    std::string provider = s.getString("provider", "MeshFlow");
    bool produceUnwarp = s.getBool("unwarp");

    /*Prepare input */
    vector<string> stream = s.nonPrefix();
    ImageCaptureInterface *cam = new AviCapture(stream.back());

    MOCProcessor processor;

    cam->imageInterfaceReceiver = &processor;
    cam->initCapture();
    cam->startCapture();


    int framecount = 0;
    RGB24Buffer *prevFrame = NULL;
    ImageCaptureInterface::FramePair images;


    AVEncoder newOut;
    AVEncoder rawOut;

    while (framecount < maxFrame)
    {
        Statistics stats;
        stats.setValue("Frame Number", framecount);
        framecount++;

        images.freeBuffers();
        images = cam->getFrameRGB24();

        RGB24Buffer *curFrame = images.rgbBufferDefault();
        images.setRgbBufferDefault(NULL); // Unlink from images

        if (prevFrame == NULL)
        {
            prevFrame = curFrame;
            continue;
        }

        //cv::Mat mInput1 = OpenCVTools::getCVMatFromRGB24Buffer(prevFrame);
        //cv::Mat mInput2 = OpenCVTools::getCVMatFromRGB24Buffer(curFrame);

        if (!newOut.open) {
            newOut.startEncoding("new.avi", curFrame->h, curFrame->w);
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
        flowGen->setFrameRGB24(Processor6D::FRAME_RIGHT_ID, prevFrame);
        flowGen->endFrame();
        flowGen->beginFrame();
        flowGen->setFrameRGB24(Processor6D::FRAME_RIGHT_ID, curFrame);
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


        unique_ptr<RGB24Buffer> outputPrev(new RGB24Buffer(prevFrame));
        unique_ptr<RGB24Buffer> outputRaw(new RGB24Buffer(curFrame));
        unique_ptr<RGB24Buffer> outputNew(new RGB24Buffer(curFrame));


        /*std::string input =
        "omnidirectional\n"
        "1578 1.35292 1.12018 5 0.520776 -0.561115 -0.560149 1.01397 -0.870155";
        */

        std::string input =
        "omnidirectional\n"
        "604 1.58492 1.07424 5 1.14169 -0.203229 -0.362134 0.351011 -0.147191";
        std::istringstream ss(input);


        Vector2dd inSize(curFrame->w, curFrame->h);
        OmnidirectionalProjection projection;

        projection.setSizeX(curFrame->w);
        projection.setSizeY(curFrame->h);
        projection.setDistortedSizeX(curFrame->w);
        projection.setDistortedSizeY(curFrame->h);

        projection.setPrincipalX(957.28999999999996);
        projection.setPrincipalY(648.84000000000003);
        projection.setFocal(689.58076000000005);
        projection.setN({
             -0.23202500000000001,
             -0.472026,
              0.52235399999999998,
              -0.25007699999999999});


        CameraModel model;
        //model.intrinsics.reset(projection.clone());
        model.intrinsics.reset(ILFormat::loadIntrisics(ss));
        model.setLocation(Affine3DQ::Identity());
        //cout << model << endl;

        /* Unwrap */
        Vector2dd unwrapSize(800, 800);

        PlaneFrame frame;
        frame.e1 = Vector3dd::OrtX() * 3;
        frame.e2 = Vector3dd::OrtY() * 3;
        frame.p1 = Vector3dd(-1.5, -1.5, 1.0);

        unique_ptr<RGB24Buffer> unwarpOrig;
        unique_ptr<RGB24Buffer> unwarpRaw;
        unique_ptr<RGB24Buffer> unwarpNew;


        if (produceUnwarp)
        {
            Mesh3D mesh;

            CalibrationDrawHelpers helpers;
            helpers.drawCamera(mesh, model, 1.0);

            stats.startInterval();
            unwarpOrig.reset(new RGB24Buffer(unwrapSize.y(), unwrapSize.x()));

            unwarpOrig->checkerBoard(20, RGBColor::White(), RGBColor::Black());

            mesh.addPlaneFrame(frame);
            for (int i = 0; i < unwarpOrig->h; i++)
            {
                for (int j = 0; j < unwarpOrig->w; j++)
                {
                    Vector2dd c((double)j / unwarpOrig->w, (double)i / unwarpOrig->h);
                    Vector3dd r = frame.getPoint(c);
                    //cout << "R" << r << endl;
                    Vector2dd p = model.intrinsics->project(r) /** inSize*/;

                    if (curFrame->isValidCoordBl(p)) {
                        unwarpOrig->element(i,j) = curFrame->elementBl(p);
                    } else {
                    }
                }
            }
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
                Vector3dd rd0 = model.intrinsics->reverse(data0);
                Vector3dd rd1 = model.intrinsics->reverse(data1);

                Vector2dd pf0 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd0, Vector3dd::Zero())) * unwrapSize;
                Vector2dd pf1 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd1, Vector3dd::Zero())) * unwrapSize;

                unwarpRaw->drawLine(pf0, pf1, color);
                unwarpRaw->drawCrosshare1(pf1, color);
            }

        }

        stats.resetInterval("Draw flow over unwrapped");

        /**/

        CorrespondenceList list;

        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            Vector3dd ray0 = model.intrinsics->reverse(data0);
            Vector3dd ray1 = model.intrinsics->reverse(data1);

            if (ray0.z() < 0.15) {
                continue;
            }
            if (ray1.z() < 0.15) {
                continue;
            }

            ray0.normalizeProjective();
            ray1.normalizeProjective();

            list.push_back(Correspondence(ray0.xy(), ray1.xy()));
            //cout << list.back() << endl;
        }

        cout << "Starting list size:" <<  list.size() << endl;

        stats.resetInterval("Created estimator input");

        RansacEstimator estimator(8, 500, 0.009);

        vector<Correspondence *> listPtr;
        listPtr.reserve(list.size());
        for (size_t d = 0 ; d < list.size(); d++)
        {
            listPtr.push_back(&list[d]);
        }

        EssentialMatrix matrix = estimator.getEssentialRansac(&listPtr);
        stats.setValue("Ransac inliers %", estimator.fitPercent);
        cout << matrix << endl;

        EssentialDecomposition decomposition[4];
        matrix.decompose(decomposition);

        /*cout << decomposition[0] << endl;
        cout << decomposition[1] << endl;
        cout << decomposition[2] << endl;
        cout << decomposition[3] << endl;*/

        EssentialDecomposition decomp = matrix.decompose(&listPtr,decomposition);
        cout << decomp << endl;

        stats.resetInterval("Estimator and decompostion");


        Vector2dd vanish = model.intrinsics->project(decomp.direction);
        cout << "Vanish point: "  << vanish << endl;

        for (int r = 4; r < 20; r++) {
            outputNew->drawArc(Circle2d(vanish, r), RGBColor::Yellow());
        }

        for (size_t i = 0; i < flowList->size(); i++)
        {
            Vector2dd data0 = flowList->at(i).start;
            Vector2dd data1 = flowList->at(i).end;

            Vector3dd ray0 = model.intrinsics->reverse(data0);
            Vector3dd ray1 = model.intrinsics->reverse(data1);

            ray0.normalizeProjective();
            ray1.normalizeProjective();

            double outlay = matrix.epipolarDistance(ray0.xy(), ray1.xy());

            RGBColor color = RGBColor::rainbow1(outlay * 100);

            outputNew->drawLine(data0, data1, color);
            outputNew->drawCrosshare1(data1, color);

            if (produceUnwarp) {
                Vector3dd rd0 = model.intrinsics->reverse(data0);
                Vector3dd rd1 = model.intrinsics->reverse(data1);

                Vector2dd pf0 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd0, Vector3dd::Zero())) * unwrapSize;
                Vector2dd pf1 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd1, Vector3dd::Zero())) * unwrapSize;

                unwarpNew->drawLine(pf0, pf1, color);
                unwarpNew->drawCrosshare1(pf1, color);
            }
        }

         stats.resetInterval("Debug output");

        std::string extention = ".jpg";
        std::stringstream oname;
#if 0
        oname.str("");
        oname << "prev" << framecount << extention;
        BufferFactory::getInstance()->saveRGB24Bitmap(outputPrev.get(), oname.str());
#endif
        /*
        oname.str("");
        oname << "raw" << framecount << extention;
        BufferFactory::getInstance()->saveRGB24Bitmap(outputRaw.get(), oname.str());
        */

       /* oname.str("");
        oname << "new" << framecount << extention;
        BufferFactory::getInstance()->saveRGB24Bitmap(outputNew.get(), oname.str());*/
        newOut.addFrame(outputNew.get());

        if (produceUnwarp)
        {
            oname.str("");
            oname << "unwarp-raw" << framecount << extention;
            BufferFactory::getInstance()->saveRGB24Bitmap(unwarpRaw.get(),  oname.str());

            oname.str("");
            oname << "unwarp-new" << framecount << extention;
            BufferFactory::getInstance()->saveRGB24Bitmap(unwarpNew.get(),  oname.str());
        }

        BaseTimeStatisticsCollector collector;
        collector.addStatistics(stats);
        collector.printAdvanced();

        delete_safe(flowGen);
        prevFrame = curFrame;
    }

     if (newOut.open)
     {
         newOut.endEncoding();
     }


    //system("ls -all");


    return 0;
}
