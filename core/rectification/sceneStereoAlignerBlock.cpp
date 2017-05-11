#include "fixtureScene.h"
#include "affine.h"
#include "sceneStereoAlignerBlock.h"
#include "stereoAligner.h"

namespace corecvs {

SceneStereoAlignerBlock::SceneStereoAlignerBlock()
{

}

int SceneStereoAlignerBlock::operator ()()
{
    if (inScene() == NULL)
    {
        SYNC_PRINT(("Fail. No input"));
        return 1;
    }

    FixtureCamera *camera1 = inScene()->getCameraByNumber(inFixture1(), inCamera1());
    FixtureCamera *camera2 = inScene()->getCameraByNumber(inFixture2(), inCamera2());

    if (camera1 == NULL || camera2 == NULL)
    {
        SYNC_PRINT(("Fail. No camera input"));
        return 2;
    }

    Affine3DQ pos1 = camera1->getWorldLocation();
    Affine3DQ pos2 = camera2->getWorldLocation();



    return 0;

}

} //namespace corecvs

