#ifndef SCENESHADED_H
#define SCENESHADED_H

#include <QtOpenGL/QtOpenGL>
#include <QtOpenGL/QGLFunctions>

#include "draw3dCameraParametersControlWidget.h"
#include "scene3D.h"
#include "mesh3DDecorated.h"
#include "shadedSceneControlWidget.h"

class QOpenGLShaderProgram;



class SceneShaded : public Scene3D, public QGLFunctions
{
public:
    ShadedSceneControlParameters mParameters;


    QOpenGLShaderProgram *mProgram = NULL;
    GLuint mPosAttr;
    GLuint mColAttr;

    GLuint mFaceColAttr;
    GLuint mTexAttr;
    GLuint mNormalAttr;

    GLuint mTextureSampler;
    GLuint mBumpSampler;


    GLuint mModelViewMatrix;
    GLuint mProjectionMatrix;

    /*Textures*/
    GLuint mTexture;


    /* Some test data */
    Mesh3DDecorated *mMesh = NULL;

    SceneShaded()
    {

    }

    virtual ParametersControlWidgetBase *getContolWidget() override
    {
        ShadedSceneControlWidget *result = new ShadedSceneControlWidget();
        result->setParameters(mParameters);
        return result;
    }

    virtual void setParameters(void * params)  override;

    virtual void prepareMesh(CloudViewDialog * /*dialog*/) override;
    virtual void drawMyself(CloudViewDialog *dialog)  override;

    virtual ~SceneShaded() {}
};



#endif // SCENESHADED_H
