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


    QString pointShaderCache;
    QString edgeShaderCache;
    QString faceShaderCache;

    enum ShaderTarget{
        FACE,
        POINT,
        EDGE,
        LAST
    };
    QOpenGLShaderProgram *mProgram[ShaderTarget::LAST];

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
    GLuint mTexture = -1;
    GLuint mBumpmap = -1;



    /* Some test data */
    Mesh3DDecorated *mMesh = NULL;

    SceneShaded()
    {
        for (int target = 0; target < ShaderTarget::LAST; target++)
        {
            mProgram[target] == NULL;
        }
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
    void addTexture(GLuint texId, RGB24Buffer *input);
};



#endif // SCENESHADED_H
