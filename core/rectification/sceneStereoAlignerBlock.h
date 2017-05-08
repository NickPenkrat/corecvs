#ifndef SCENE_STEREO_ALIGNER_BLOCK_H
#define SCENE_STEREO_ALIGNER_BLOCK_H

#include "sceneStereoAlignerBlockBase.h"

namespace corecvs {

class SceneStereoAlignerBlock : public SceneStereoAlignerBlockBase
{
public:
    SceneStereoAlignerBlock();


    virtual void operator ()();

    ~SceneStereoAlignerBlock() {
    }
};

} // namespace corecvs

#endif // SCENE_STEREO_ALIGNER_BLOCK_H
