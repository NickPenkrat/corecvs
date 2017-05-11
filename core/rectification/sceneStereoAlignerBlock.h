#ifndef SCENE_STEREO_ALIGNER_BLOCK_H
#define SCENE_STEREO_ALIGNER_BLOCK_H

#include "newStyleBlock.h"
#include "sceneStereoAlignerBlockBase.h"

namespace corecvs {

class SceneStereoAlignerBlock : public SceneStereoAlignerBlockBase, public NewStyleBlock
{
public:
    SceneStereoAlignerBlock();


    virtual int operator ()();

    ~SceneStereoAlignerBlock() {
    }
};

} // namespace corecvs

#endif // SCENE_STEREO_ALIGNER_BLOCK_H
