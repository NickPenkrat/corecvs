#include <stdio.h>
#include <fstream>
#include <iostream>

#include "global.h"
#include "log.h"
#include "multicameraScene.h"
#include "mesh3d.h"
#include "line.h"
#include "propertyListVisitor.h"
#include "defaultSetter.h"

using std::ofstream;

struct TriangulatorParameters {
    bool renderCams;
    bool renderFeaturePoints;
    bool renderFeatureLines;
    bool renderInputLines;
    bool renderInputPoints;
    double camCenterX;
    double camCenterY;



    TriangulatorParameters()
    {
        DefaultSetter ds;
        accept(ds);
    }


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(renderCams         , true , "renderCams");
        visitor.visit(renderFeaturePoints, true , "renderFeaturePoints");
        visitor.visit(renderFeatureLines , true , "renderFeatureLines");
        visitor.visit(renderInputLines   , true , "renderInputLines");
        visitor.visit(renderInputPoints  , true , "renderInputPoints");
        visitor.visit(camCenterX         , 2698.5 , "camCenterX");
        visitor.visit(camCenterY         , 1799.0 , "camCenterY");

    }

};


/**
 *  Sorry for C-style black magic;
 */
const char *getExtention(const char *name)
{
    int pos = strlen(name);
    pos--;
    while (pos > 0 && name[pos] != '.') pos--;

    if (pos < 0) return NULL;

    return name + pos;
}

struct NamedPoint
{
    string name;
    Vector3dd point;
};


void drawLines(MulticameraScene &scene, vector<FeaturePoint> &points ,Mesh3D &mesh)
{
    for (unsigned i = 0; i < points.size(); i++)
    {
        /*Ok. Some math */
        int pointNum = i;
        FeaturePoint &point = points[pointNum];
        L_INFO_P("Point %d has %d measurements", pointNum, point.measurements.size());

        for (unsigned m = 0; m < point.measurements.size(); m++)
        {
            SfmMeasurement &measure = point.measurements[m];

            int imageid = measure.image;
            BundlerCamera cam = scene.cameraList[imageid];
            Vector2dd coord = measure.coord;
            Ray3d ray = cam.rayFromPixel(coord);

            for (int i = 0; i < 1; i++)
            {
                mesh.addLine(ray.getPoint(i), ray.getPoint(i + 6));
            }
        }

        mesh.addPoint(point.position);
    }
}

int main (int argc, char **argv)
{
    MulticameraScene scene;

    printf("Bundler/SFM postreconstrutor...\n");

    if (argc != 2)
    {        
        printf("No input\n"
               "Usage:\n"
               "\n"
               " test_topcon <bundler cam file>.txt\n"
               " or\n"
               " test_topcon <VisialSfM>.nvm\n"
              );

        return -1;
    }


    /* Loading the parameteres */
    PropertyList props;
    TriangulatorParameters params;
    if (!props.load("topcon.conf"))
    {
        PropertyListWriterVisitor visitor(&props);
        params.accept<PropertyListWriterVisitor>(visitor);
        props.save("topcon.conf");
    } else {
        PropertyListReaderVisitor visitor(&props);
        params.accept<PropertyListReaderVisitor>(visitor);
    }




    const char* filename = argv[1];
    SYNC_PRINT(("Starting the read of %s\n", filename));
    const char* extention = getExtention(filename);
    if (extention == NULL || !strcmp(extention, ".txt"))
    {
        scene.loadBundlerFile(filename);
    } else if (!strcmp(extention, ".nvm"))
    {
        scene.loadNVMFile(filename);
    } else {
        L_INFO_P("I don't know how to load this");
        return -1;
    }

    double scale = 0.1;
    Vector3dd someSize(scale);
    Mesh3D cameraMesh;
    //cameraMesh.addAOB(AxisAlignedBox3d::ByCenter(Vector3dd(0.0,0.0,0.0), someSize));
    cameraMesh.addCamera(scene.cameraList[0].cameraIntrinsics, scale);

    Mesh3D mesh;
    for (unsigned i = 0; i < scene.cameraList.size(); i ++)
    {
        BundlerCamera &cam = scene.cameraList[i];
        Matrix44 trans = Matrix44(cam.rotation, cam.translation);
        Mesh3D campos = cameraMesh.transformed(trans);
        mesh.add(campos);
    }

    if (params.renderFeatureLines)
    {
        drawLines(scene, scene.points, mesh);
    }

    if (params.renderFeaturePoints)
    {
        for (unsigned pointNum = 0; pointNum < scene.points.size(); pointNum++)
        {
            FeaturePoint &point = scene.points[pointNum];
            mesh.addPoint(point.position);
        }
    }

    /* Ok lets load a bit of data*/
    scene.loadInput("manual.match");

     L_INFO << "Center" << Vector2dd(params.camCenterX, params.camCenterY);
    /* So far a small hack*/
    for (unsigned i = 0; i < scene.input.size(); i++)
    {
        for (unsigned m = 0; m < scene.input[i].measurements.size(); m++)
        {
            scene.input[i].measurements[m].coord -= Vector2dd(params.camCenterX, params.camCenterY);

        }
    }

    /* Draw pretty lines */
    if (params.renderInputLines)
    {
        drawLines(scene, scene.input, mesh);
    }

    /* Trinagulating*/


    for (unsigned i = 0; i < scene.input.size(); i++)
    {
        FeaturePoint &input = scene.input[i];
        int count = 0;
        Vector3dd sumPos(0.0);
        for (unsigned m1 = 0; m1 < input.measurements.size(); m1++)
        {
            for (unsigned m2 = m1 + 1; m2 < input.measurements.size(); m2++)
            {
                SfmMeasurement &measure1 = input.measurements[m1];
                SfmMeasurement &measure2 = input.measurements[m2];

                int imageid1 = measure1.image;
                int imageid2 = measure2.image;

                BundlerCamera &cam1 = scene.cameraList[imageid1];
                BundlerCamera &cam2 = scene.cameraList[imageid2];

                Vector2dd coord1 = measure1.coord;
                Vector2dd coord2 = measure2.coord;

                L_INFO << "Points" << coord1 << " - " << coord2;

                Ray3d ray1 = cam1.rayFromPixel(coord1);
                Ray3d ray2 = cam2.rayFromPixel(coord2);

                Vector3dd point = ray1.intersect(ray2);
                input.guesses.push_back(point);
                mesh.addPoint(point);
                count++;
                sumPos += point;
            }
        }
        if (count != 0) {
            input.position = sumPos / count;
        }
    }

    /* Distances log */
    vector<NamedPoint> pointsToReport;

    for (unsigned i = 0; i < scene.cameraList.size(); i++)
    {
        NamedPoint np;
        std::ostringstream stringStream;
        stringStream << "Camera" << i;
        np.name = stringStream.str();
        np.point = scene.cameraList[i].translation;
        pointsToReport.push_back(np);
    }

    for (unsigned i = 0; i < scene.input.size(); i++)
    {
        FeaturePoint &input = scene.input[i];
        NamedPoint np;
        np.name = input.name;
        np.point = input.position;
        pointsToReport.push_back(np);
    }

    for (unsigned p1 = 0; p1 < pointsToReport.size(); p1++)
    {
        for (unsigned p2 = p1 + 1; p2 < pointsToReport.size(); p2++)
        {
            printf("%s to %s - %lf\n",
                   pointsToReport[p1].name.c_str(),
                   pointsToReport[p2].name.c_str(),
                   !(pointsToReport[p1].point - pointsToReport[p2].point) * 7080.4
            );
        }
    }



    ofstream outply;
    outply.open ("out.ply", std::ios::out);
    if (outply.fail())
    {
        SYNC_PRINT(("main: Can't open file\n"));
        return 1;
    }

    mesh.dumpPLY(outply);
    outply.close();







    return 0;
}
