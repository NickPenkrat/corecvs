#ifndef SCENE_STEREO_ALIGNER_BLOCK_H
#define SCENE_STEREO_ALIGNER_BLOCK_H

#include "newStyleBlock.h"
#include "sceneStereoAlignerBlockBase.h"

#include <fixtureCamera.h>

namespace corecvs {

class SceneStereoAlignerBlock : public SceneStereoAlignerBlockBase, public NewStyleBlock
{
public:
    SceneStereoAlignerBlock();


    virtual int operator ()();

    ~SceneStereoAlignerBlock() {
    }

    FixtureCamera *camera1 = NULL;
    FixtureCamera *camera2 = NULL;
};

} // namespace corecvs

#endif // SCENE_STEREO_ALIGNER_BLOCK_H
