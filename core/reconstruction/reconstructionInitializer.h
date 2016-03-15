#ifndef RECONSTRUCTIONINITIALIZER_H
#define RECONSTRUCTIONINITIALIZER_H

#include "reconstructionFixtureScene.h"

namespace corecvs
{
/*
 * This class is responsible for reconstruction initialization.
 *
 * When there is no 3D pointcloud and we cannot use P3P+RANSAC
 * in order to robustly determine pose and position of multicamera,
 * we are trying to detect relative orientation/position using
 * more correspondences.
 */
struct ReconstructionInitializer
{
	ReconstructionFixtureScene *rfs;
};
};

#endif
