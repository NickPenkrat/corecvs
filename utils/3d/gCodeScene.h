#ifndef GCODESCENE_H
#define GCODESCENE_H

/**
 * \file gCodeScene.h
 *
 **/

#include <ostream>

#include "global.h"

#include "scene3D.h"
#include "sceneShaded.h"
#include "cloudViewDialog.h"
#include "gcodeLoader.h"
#include "generated/draw3dParameters.h"
#include "draw3dParametersControlWidget.h"
#include "draw3dCameraParametersControlWidget.h"
#include "generated/drawGCodeParameters.h"

#include "reflectionWidget.h"

class GCodeScene : public SceneShaded {

public:
    DrawGCodeParameters mParameters;

    GCodeProgram *owned;

    GCodeScene();

#if 0
    virtual void prepareMesh(CloudViewDialog * /*dialog*/) {}
#endif

    virtual void drawMyself(CloudViewDialog * /*dialog*/) override;

    virtual bool dump(const QString &targetFile) override;

    virtual void setParameters(void * params) override;

    virtual ParametersControlWidgetBase *getContolWidget() override
    {
        ReflectionWidget *result = new ReflectionWidget(&DrawGCodeParameters::reflection);
        result->setParameters(&mParameters);
        return result;
    }

    void replaceGode(GCodeProgram *newGCode) {
        delete_safe(owned);
        owned = newGCode;
        updateMesh();
    }

    void updateMesh();

    virtual ~GCodeScene();
};

#endif // GCODESCENE_H
/* EOF */
