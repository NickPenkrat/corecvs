/**
 * \file main_test_deform.cpp
 * \brief This is the main file for the test deform
 *
 * \date Mar 01, 2012
 * \author alexander
 *
 * \ingroup autotest
 */

#include <iostream>
#include "global.h"

#include "fixedPointDisplace.h"
#include "matrix33.h"
#include "projectiveTransform.h"
#include "g12Buffer.h"
#include "bufferFactory.h"
#include "bmpLoader.h"
#include "rgb24Buffer.h"
#include "radialCorrection.h"

#include "preciseTimer.h"

using namespace std;
using corecvs::Matrix33;
using corecvs::ProjectiveTransform;
using corecvs::G12Buffer;
using corecvs::BufferFactory;
using corecvs::FixedPointDisplace;
using corecvs::BMPLoader;
using corecvs::RGB24Buffer;

void testFastDeform ()
{

    Matrix33 inverseLeftMatrix(
        1, 0, -13,
        0, 1, -9.5,
        0, 0, 1
    );

    ProjectiveTransform inverseLeft(inverseLeftMatrix);


    G12Buffer *image = BufferFactory::getInstance()->loadG12Bitmap("data/pair/image0001_c0.pgm");
    ASSERT_TRUE(image, "Could not open test image\n");
    ASSERT_TRUE(image->verify(),"Input image is corrupted");
    G12Buffer *buffer1Transformed = image->doReverseTransform<ProjectiveTransform>(&inverseLeft, image->h, image->w);
    ASSERT_TRUE(buffer1Transformed->verify(),"Result image is corrupted");

    FixedPointDisplace *displace = new FixedPointDisplace (inverseLeft, image->h, image->w);
    G12Buffer *buffer2Transformed = image->doReverseDeformationBlPrecomp(displace, image->h, image->w);

    BMPLoader().save("proc.bmp",  buffer1Transformed);
    BMPLoader().save("proc1.bmp", buffer2Transformed);

    G12Buffer *difference = G12Buffer::difference(buffer1Transformed, buffer2Transformed);
    difference->gainOffset(1/20.0, 0);

    BMPLoader().save("diff50.bmp", difference);

    delete difference;
    delete buffer2Transformed;
    delete buffer1Transformed;
    delete displace;
    delete image;

}

template<typename operation>
    void touchOperationElementwize (RGB24Buffer *buffer, operation &map)
    {
        for (int i = 0; i < buffer->h; i++)
        {
            RGBColor *thisElemRunner = &(buffer->element(i,0));
            for (int j = 0; j < buffer->w; j++)
            {
                map(i, j, (RGBColor *)thisElemRunner);
                thisElemRunner++;
            }
        }
    }


void testFastDeform24 ()
{
    Matrix33 inverseLeftMatrix(
        1, 0, -13,
        0, 1, -9.5,
        0, 0, 1
    );

    ProjectiveTransform inverseLeft(inverseLeftMatrix);


    RGB24Buffer *image = new RGB24Buffer(300, 300);

    auto operation = [](int i, int j, RGBColor *pixel)
    {
        i = i / 20;
        j = j / 20;
        if ( (i % 2) &&  (j % 2))   *pixel = RGBColor::Green();
        if (!(i % 2) &&  (j % 2))   *pixel = RGBColor::Yellow();
        if ( (i % 2) && !(j % 2))   *pixel = RGBColor::Red();
        if (!(i % 2) && !(j % 2))   *pixel = RGBColor::Blue();
    };
    touchOperationElementwize(image, operation);
    RGB24Buffer *buffer1Transformed = image->doReverseDeformation  <RGB24Buffer, ProjectiveTransform>(inverseLeft);

    /* This would produce empty buffer due to overflow. This is by design */
    RGB24Buffer *buffer2Transformed = image->doReverseDeformationBl<RGB24Buffer, ProjectiveTransform>(&inverseLeft);
    RGB24Buffer *buffer3Transformed = image->doReverseDeformationBlTyped<ProjectiveTransform>(&inverseLeft);

    FixedPointDisplace *displace = new FixedPointDisplace (inverseLeft, image->h, image->w);
    RGB24Buffer *buffer4Transformed = image->doReverseDeformationBlPrecomp(displace, image->h, image->w);


    BMPLoader().save("input.bmp", image);
    BMPLoader().save("transform-int.bmp"    , buffer1Transformed);
    BMPLoader().save("transform-bl.bmp"     , buffer2Transformed);
    BMPLoader().save("transform-blt.bmp"    , buffer3Transformed);
    BMPLoader().save("transform-precomp.bmp", buffer4Transformed);


    delete_safe(buffer1Transformed);
    delete_safe(buffer2Transformed);
    delete_safe(image);
}

void testRadialInversion( void )
{
    RGB24Buffer *image = new RGB24Buffer(2500, 4000);

    auto operation = [](int i, int j, RGBColor *pixel)
    {
        i = i / 100;
        j = j / 200;
        if ( (i % 2) &&  (j % 2))   *pixel = RGBColor::Green();
        if (!(i % 2) &&  (j % 2))   *pixel = RGBColor::Yellow();
        if ( (i % 2) && !(j % 2))   *pixel = RGBColor::Red();
        if (!(i % 2) && !(j % 2))   *pixel = RGBColor::Blue();
    };
    touchOperationElementwize(image, operation);

    LensDistortionModelParameters deformator;
    deformator.setPrincipalX(image->w / 2);
    deformator.setPrincipalY(image->h / 2);

    deformator.setTangentialX(0.000001);
    deformator.setTangentialY(0.000001);

    deformator.setAspect(1.0);
    deformator.setScale(1.0);

    deformator.mKoeff.push_back( 0.0001);
    deformator.mKoeff.push_back(-0.00000002);
    deformator.mKoeff.push_back( 0.00000000000003);

    RadialCorrection T(deformator);
    PreciseTimer timer;

    cout << "Starting deformaiton... " << flush;
    timer = PreciseTimer::currentTime();
    RGB24Buffer *defomed = image->doReverseDeformationBlTyped<RadialCorrection>(&T);
    cout << "done in: " << timer.usecsToNow() << "us" << endl;

    /* */
    cout << "Starting invertion... " << flush;
    RadialCorrection invert = T.invertCorrection(image->h, image->w, 30);
    cout << "done" << endl;

    cout << "Starting backprojection... " << flush;
    timer = PreciseTimer::currentTime();
    RGB24Buffer *backproject = defomed->doReverseDeformationBlTyped<RadialCorrection>(&invert);
    cout << "done in: " << timer.usecsToNow() << "us" << endl;
    cout << "done" << endl;

    BMPLoader().save("input.bmp"      , image);
    BMPLoader().save("forward.bmp"    , defomed);
    BMPLoader().save("backproject.bmp", backproject);



}

int main (int /*argC*/, char ** /*argV*/)
{
    //testFastDeform ();
    //testFastDeform24 ();

    testRadialInversion();

    cout << "PASSED" << endl;
    return 0;
}
