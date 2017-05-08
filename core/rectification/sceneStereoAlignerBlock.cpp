#include "fixtureScene.h"
#include "affine.h"
#include "sceneStereoAlignerBlock.h"

namespace corecvs {

SceneStereoAlignerBlock::SceneStereoAlignerBlock()
{

}

void SceneStereoAlignerBlock::operator ()()
{
    if (inScene() == NULL)
    {
        SYNC_PRINT(("Fail. No input"));
        return;
    }

    FixtureCamera *camera1 = inScene()->getCameraByNumber(inFixture1(), inCamera1());
    FixtureCamera *camera2 = inScene()->getCameraByNumber(inFixture2(), inCamera2());

    if (camera1 == NULL || camera2 == NULL)
    {
        SYNC_PRINT(("Fail. No camera input"));
        return;
    }

    Affine3DQ pos1 = camera1->getWorldLocation();
    Affine3DQ pos2 = camera2->getWorldLocation();



}

} //namespace corecvs

