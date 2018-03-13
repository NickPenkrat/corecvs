/*
    Bayer to PPM converter
 */

#include <OpenCVTools.h>
#include <QApplication>
#include <iostream>
#include <time.h>

#include <core/rectification/ransacEstimator.h>

#include <core/geometry/mesh3d.h>

#include <core/cameracalibration/calibrationDrawHelpers.h>

#include "core/buffers/bufferFactory.h"
#include "core/cameracalibration/cameraModel.h"
#include "core/buffers/converters/errorMetrics.h"
#include "core/reflection/commandLineSetter.h"
#include "core/utils/utils.h"
#include "qtFileLoader.h"
#include "core/cameracalibration/ilFormat.h"


#include "core/framesources/imageCaptureInterface.h"


#include "moc-flow/openCVMovingObjectFlow.h"

using namespace std;
using namespace corecvs;


class UnwarpedReprojector
{


};


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

    ImageCaptureInterface *cam = NULL;


    RGB24Buffer *input1 = BufferFactory::getInstance()->loadRGB24Bitmap("in0.jpg");
    RGB24Buffer *input2 = BufferFactory::getInstance()->loadRGB24Bitmap("in1.jpg");
    RGB24Buffer *input3 = BufferFactory::getInstance()->loadRGB24Bitmap("in2.jpg");

    if (input1 == NULL || input2 == NULL || input3 == NULL)
    {
        SYNC_PRINT(("unable to open input\n"));
        return 1;
    }
    cv::Mat mInput1 = OpenCVTools::getCVMatFromRGB24Buffer(input1);
    cv::Mat mInput2 = OpenCVTools::getCVMatFromRGB24Buffer(input2);
    cv::Mat mInput3 = OpenCVTools::getCVMatFromRGB24Buffer(input3);

    Statistics stats;
    MeshFlow processor;
    processor.stats = &stats;
    processor.init(mInput1.rows, mInput1.cols);
    processor.computeMeshFlow(mInput1, mInput2);


    RGB24Buffer *output1 = OpenCVTools::getRGB24BufferFromCVMat(mInput1);
    RGB24Buffer *output2 = OpenCVTools::getRGB24BufferFromCVMat(mInput1);

    BaseTimeStatisticsCollector collector;
    collector.addStatistics(stats);
    collector.printAdvanced();

    /*std::string input =
    "omnidirectional\n"
    "1578 1.35292 1.12018 5 0.520776 -0.561115 -0.560149 1.01397 -0.870155";
    */

    std::string input =
    "omnidirectional\n"
    "604 1.58492 1.07424 5 1.14169 -0.203229 -0.362134 0.351011 -0.147191";
    std::istringstream ss(input);


    Vector2dd inSize(input1->w, input1->h);
    OmnidirectionalProjection projection;

    projection.setSizeX(input1->w);
    projection.setSizeY(input1->h);
    projection.setDistortedSizeX(input1->w);
    projection.setDistortedSizeY(input1->h);

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
    cout << model << endl;

    /* Unwrap */
    Mesh3D mesh;

    CalibrationDrawHelpers helpers;
    helpers.drawCamera(mesh, model, 1.0);

    RGB24Buffer *unwarpOrig = new RGB24Buffer(1500, 1500);
    Vector2dd unwrapSize(unwarpOrig->w, unwarpOrig->h);
    unwarpOrig->checkerBoard(20, RGBColor::White(), RGBColor::Black());
    PlaneFrame frame;
    frame.e1 = Vector3dd::OrtX() * 3;
    frame.e2 = Vector3dd::OrtY() * 3;
    frame.p1 = Vector3dd(-1.5, -1.5, 1.0);

    mesh.addPlaneFrame(frame);


    for (int i = 0; i < unwarpOrig->h; i++)
    {
        for (int j = 0; j < unwarpOrig->w; j++)
        {
            Vector2dd c((double)j / unwarpOrig->w, (double)i / unwarpOrig->h);
            Vector3dd r = frame.getPoint(c);
            //cout << "R" << r << endl;
            Vector2dd p = model.intrinsics->project(r) /** inSize*/;

            if (input1->isValidCoordBl(p)) {
                unwarpOrig->element(i,j) = input1->elementBl(p);
              //  cout << "Hit " <<  p << endl;
            } else {
              //  cout << "Miss " <<  p << endl;
            }
        }
    }
    mesh.dumpPLY("unwrap.ply");
    RGB24Buffer *unwarpRaw = new RGB24Buffer(unwarpOrig);
    RGB24Buffer *unwarpNew = new RGB24Buffer(unwarpOrig);



    /* Debug draw */
    for (size_t i = 0; i < processor.feat_cur_.size(); i++)
    {
        Point2f &cvData0  = processor.feat_prev_[i];
        Point2f &cvData1  = processor.feat_cur_ [i];

        Vector2dd data0(cvData0.x, cvData0.y);
        Vector2dd data1(cvData1.x, cvData1.y);

        uint8_t status = processor.feat_status_[i];

        RGBColor color = RGBColor::Red();
        switch (status) {
        case 0: color = RGBColor::Green(); break;
        case 1: color = RGBColor::Red(); break;
        case 2: color = RGBColor::Blue(); break;
        default:
            break;
        }

        output1->drawLine(data0, data1, color);
        output1->drawCrosshare1(data1, color);


        Vector3dd rd0 = model.intrinsics->reverse(data0);
        Vector3dd rd1 = model.intrinsics->reverse(data1);

        Vector2dd pf0 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd0, Vector3dd::Zero())) * unwrapSize;
        Vector2dd pf1 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd1, Vector3dd::Zero())) * unwrapSize;

        unwarpRaw->drawLine(pf0, pf1, color);
        unwarpRaw->drawCrosshare1(pf1, color);

    }

    /**/

    CorrespondenceList list;

    for (size_t i = 0; i < processor.feat_cur_.size(); i++)
    {
        Point2f &cvData0  = processor.feat_prev_[i];
        Point2f &cvData1  = processor.feat_cur_ [i];

        Vector2dd data0(cvData0.x, cvData0.y);
        Vector2dd data1(cvData1.x, cvData1.y);

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


    RansacEstimator estimator(8, 500, 0.009);

    vector<Correspondence *> listPtr;
    listPtr.reserve(list.size());
    for (size_t d = 0 ; d < list.size(); d++)
    {
        listPtr.push_back(&list[d]);
    }

    EssentialMatrix matrix = estimator.getEssentialRansac(&listPtr);
    cout << matrix << endl;

    EssentialDecomposition decomposition[4];
    matrix.decompose(decomposition);

    cout << decomposition[0] << endl;
    cout << decomposition[1] << endl;
    cout << decomposition[2] << endl;
    cout << decomposition[3] << endl;

    EssentialDecomposition decomp = matrix.decompose(&listPtr,decomposition);
    cout << decomp << endl;

    Vector2dd vanish = model.intrinsics->project(decomp.direction);
    cout << "Vanish point: "  << vanish << endl;

    for (int r = 4; r < 20; r++) {
        output2->drawArc(Circle2d(vanish, r), RGBColor::Yellow());
    }

    for (size_t i = 0; i < processor.feat_cur_.size(); i++)
    {
        Point2f &cvData0  = processor.feat_prev_[i];
        Point2f &cvData1  = processor.feat_cur_ [i];

        Vector2dd data0(cvData0.x, cvData0.y);
        Vector2dd data1(cvData1.x, cvData1.y);

        Vector3dd ray0 = model.intrinsics->reverse(data0);
        Vector3dd ray1 = model.intrinsics->reverse(data1);

        ray0.normalizeProjective();
        ray1.normalizeProjective();

        double outlay = matrix.epipolarDistance(ray0.xy(), ray1.xy());

        RGBColor color = RGBColor::rainbow1(outlay * 100);

        output2->drawLine(data0, data1, color);
        output2->drawCrosshare1(data1, color);

        Vector3dd rd0 = model.intrinsics->reverse(data0);
        Vector3dd rd1 = model.intrinsics->reverse(data1);

        Vector2dd pf0 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd0, Vector3dd::Zero())) * unwrapSize;
        Vector2dd pf1 = frame.intersectWithRayPoint(Ray3d::FromDirectionAndOrigin(rd1, Vector3dd::Zero())) * unwrapSize;

        unwarpNew->drawLine(pf0, pf1, color);
        unwarpNew->drawCrosshare1(pf1, color);
    }




    BufferFactory::getInstance()->saveRGB24Bitmap(output1, "out0.bmp");
    BufferFactory::getInstance()->saveRGB24Bitmap(output2, "out1.bmp");
    BufferFactory::getInstance()->saveRGB24Bitmap(unwarpRaw,  "unwrap-raw.bmp");

    delete_safe(unwarpOrig);
    delete_safe(unwarpRaw);
    delete_safe(unwarpNew);

    /*delete_safe(unwarpOrig);
    delete_safe(unwarpOrig);*/



    return 0;
}
