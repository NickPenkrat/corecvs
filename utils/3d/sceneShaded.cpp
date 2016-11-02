#include <qglfunctions.h>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QtGui/QMatrix4x4>

//#include <GL/gl.h>

#include <qopengl.h>
#include "sceneShaded.h"


static const char *vertexShaderSource =
    "attribute highp vec4 posAttr;\n"
    "attribute lowp  vec4 colAttr;\n"
    "varying lowp vec4 col;\n"
    "uniform highp mat4 modelview;\n"
    "uniform highp mat4 projection;\n"
    "void main() {\n"
    "   col = colAttr / 1.0;\n"
    "   col.a = 1.0;\n"
    "   gl_Position = projection * modelview * posAttr;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying lowp vec4 col;\n"
    "void main() {\n"
    "   gl_FragColor = col;\n"
    "   gl_FragDepth = gl_FragCoord.z;\n"
    "}\n";


void SceneShaded::prepareMesh(CloudViewDialog * /*dialog*/)
{
    initializeGLFunctions();

    mProgram = new QOpenGLShaderProgram();
    mProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    mProgram->link();

    mPosAttr = mProgram->attributeLocation("posAttr");
    mColAttr = mProgram->attributeLocation("colAttr");
    mModelViewMatrix  = mProgram->uniformLocation("modelview");
    mProjectionMatrix = mProgram->uniformLocation("projection");


}

void SceneShaded::drawMyself(CloudViewDialog * /*dialog*/)
{

    mProgram->bind();

    float arr[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, arr);
    QMatrix4x4 modelview(
            arr[0], arr[4], arr[ 8], arr[12],
            arr[1], arr[5], arr[ 9], arr[13],
            arr[2], arr[6], arr[10], arr[14],
            arr[3], arr[7], arr[11], arr[15]
    );

    glGetFloatv(GL_PROJECTION_MATRIX, arr);
    QMatrix4x4 projection(
            arr[0], arr[4], arr[ 8], arr[12],
            arr[1], arr[5], arr[ 9], arr[13],
            arr[2], arr[6], arr[10], arr[14],
            arr[3], arr[7], arr[11], arr[15]
    );

    /*matrix.perspective(60.0f, 4.0f/3.0f, 0.1f, 100.0f);
    matrix.translate(0, 0, -2);*/
   // matrix.rotate(100.0f * m_frame / screen()->refreshRate(), 0, 1, 0);

    mProgram->setUniformValue(mModelViewMatrix, modelview);
    mProgram->setUniformValue(mProjectionMatrix, projection);



    GLfloat vertices[] = {
       0.0f, 0.707f,
       -0.5f, -0.5f,
       0.5f, -0.5f
    };

    GLfloat colors[] = {
       1.0f, 0.0f, 0.0f,
       0.0f, 1.0f, 0.0f,
       0.0f, 0.0f, 1.0f
    };

    bool depthTest =  glIsEnabled(GL_DEPTH_TEST);
    glEnable(GL_DEPTH_TEST);


    glVertexAttribPointer(mPosAttr, 2, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(mColAttr, 3, GL_FLOAT, GL_FALSE, 0, colors);

    glEnableVertexAttribArray(mPosAttr);
    glEnableVertexAttribArray(mColAttr);

    glDrawArrays(GL_TRIANGLES, 0, 3);

    glDisableVertexAttribArray(mPosAttr);
    glDisableVertexAttribArray(mColAttr);

    /* Draw embedded Mesh */
    if (mMesh != NULL)
    {
        /* Draw faces */

        glVertexAttribPointer(mPosAttr, 3, GL_DOUBLE, GL_FALSE, 0, &(mMesh->vertexes.front().x()));
        glEnableVertexAttribArray(mPosAttr);

        if (mMesh->hasColor && mMesh->vertexesColor.size() == mMesh->vertexes.size())
        {
            RGBColor *color = &mMesh->vertexesColor.front();
            glVertexAttribPointer(mColAttr, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(RGBColor), color);
            glEnableVertexAttribArray(mColAttr);
        }
        if (mMesh->hasNormals)
        {

        }

        glDrawElements(GL_TRIANGLES, GLsizei(mMesh->faces.size() * 3), GL_UNSIGNED_INT, &(mMesh->faces[0][0]));

    }

    glDisableVertexAttribArray(mPosAttr);
    glDisableVertexAttribArray(mColAttr);

    if (!depthTest) {
        glDisable(GL_DEPTH_TEST);
    }

    mProgram->release();


}

void SceneShaded::setParameters(void *params)
{
    mParameters = *static_cast<ShadedSceneControlParameters *>(params);
}
