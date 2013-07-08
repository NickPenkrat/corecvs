/**
 * \file cascadeClassifier.h
 * \brief This file should hold the header for cascade classifier
 *
 * \ingroup cppcorefiles
 * \date Jun 24, 2010
 * \author alexander
 */

#ifndef CASCADECLASSIFIER_H_
#define CASCADECLASSIFIER_H_

#include "global.h"

#include "g12Buffer.h"
#include "mipmapPyramid.h"
namespace core3vi {

class CascadeClassifier
{
public:
    //AbstractMidmapPyramid<G12Buffer> *

    CascadeClassifier();
    virtual ~CascadeClassifier();
};


} //namespace core3vi
#endif /* CASCADECLASSIFIER_H_ */

