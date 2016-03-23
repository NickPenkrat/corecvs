#include "sceneAligner.h"

bool corecvs::SceneAligner::tryAlign(ReconstructionFixtureScene* scene)
{
	if (scene->is3DAligned)
		return true;
	return false;
}
