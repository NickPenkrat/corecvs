/**
 * \file plyLoader.cpp
 *
 * \date Nov 13, 2012
 **/

#include <sstream>

#include "plyLoader.h"
#include "utils.h"

namespace corecvs {


using std::istringstream;

PLYLoader::PLYLoader()
{
}

/**
 *
 *  format ascii 1.0
    comment author: Greg Turk
    comment object: another cube
    element vertex 8
    property float x
    property float y
    property float z
    property uchar red                   { start of vertex color }
    property uchar green
    property uchar blue
    element face 7
    property list uchar int vertex_index  { number of vertices for each face }
    element edge 5                        { five edges in object }
    property int vertex1                  { index to first vertex of edge }
    property int vertex2                  { index to second vertex }
    property uchar red                    { start of edge color }
    property uchar green
    property uchar blue
 *
 *
 **/

int PLYLoader::loadPLY(istream &input, Mesh3D &mesh)
{
    //char line[300];
    string line;

    HelperUtils::getlineSafe (input, line);
    if (line != "ply") {
        SYNC_PRINT(("Not a PLY file\n"));
        return 1;
    }

    const char *com_format     = "format";
    const char *com_comment    = "comment";
    const char *com_element    = "element";
    const char *com_property   = "property";
    const char *com_end_header = "end_header";
    const char *form_ascii     = "ascii";
    const char *form_binaryle  = "binary_little_endian";



    const char *el_vertex = "vertex";
    const char *el_face   = "face";
    const char *el_edge   = "edge";

    int objNumber[OBJ_LAST] = {-1, -1, -1};
    PlyFormat plyFormat = OTHER;
    ObjType propState = OBJ_VERTEX;


    vector<Prop> objProps[OBJ_LAST];

    while (!input.eof())
    {
        HelperUtils::getlineSafe (input, line);

        istringstream work(line);
        string command;
        work >> command;

        SYNC_PRINT(("Input line: %s\n", line.c_str()));

        if (command == com_format)
        {
            string format;
            work >> format;
            if (format ==  form_ascii)
            {
                plyFormat = ASCII;
                SYNC_PRINT(("PLY ascii format\n"));
                continue;
            }

            if (format ==  form_binaryle)
            {
                plyFormat = BINARY_LITTLE_ENDIAN;
                SYNC_PRINT(("PLY binary format\n"));
                continue;
            }

            SYNC_PRINT(("PLY format not supported. Format here <%s>\n", format.c_str()));
            return 1;
        }

        if (command == com_comment) {
            continue;
        }

        if (command == com_property)
        {
            Prop p;
            work >> p;
            if (work.bad()) {
               SYNC_PRINT(("Prop failed to load\n"));
               continue;
            }
            objProps[propState].push_back(p);
            continue;
        }

        if (command == com_element)
        {

            string objtype;
            work >> objtype;

            if (objtype == el_vertex)
            {
                propState = OBJ_VERTEX;
            }

            if (objtype == el_face)
            {
                propState = OBJ_FACE;
            }

            if (objtype == el_edge)
            {
                propState = OBJ_EDGE;
            }

            work >> objNumber[propState];
            if (work.bad()) {
               SYNC_PRINT(("Number is not found"));
            }
            SYNC_PRINT(("Number %d %d\n", propState, objNumber[propState]));
            continue;
        }

        if (command == com_end_header)
        {
            break;
        }
    }

    if (objNumber[OBJ_VERTEX] == -1)
    {
        SYNC_PRINT(("Necessary data was missing form the header\n"));
        return 1;
    }

    for (int k = 0; k < OBJ_LAST; k++)
    {
        for (unsigned i = 0; i < objProps[k].size(); i++)
        {
            SYNC_PRINT((" %d %d \n", objProps[k][i].type, objProps[k][i].name));
        }
        SYNC_PRINT(("\n"));
    }

    /* Checking if we support this format? */
    bool supportVertex =
            (objProps[OBJ_VERTEX].size() == 3 &&
             objProps[OBJ_VERTEX][0].name == PROP_NAME_X &&
             objProps[OBJ_VERTEX][1].name == PROP_NAME_Y &&
             objProps[OBJ_VERTEX][2].name == PROP_NAME_Z) ||
            (objProps[OBJ_VERTEX].size() == 6 &&
             objProps[OBJ_VERTEX][0].name == PROP_NAME_X &&
             objProps[OBJ_VERTEX][1].name == PROP_NAME_Y &&
             objProps[OBJ_VERTEX][2].name == PROP_NAME_Z &&
             objProps[OBJ_VERTEX][3].name == PROP_NAME_RED &&
             objProps[OBJ_VERTEX][4].name == PROP_NAME_GREEN &&
             objProps[OBJ_VERTEX][5].name == PROP_NAME_BLUE);
    bool vertexColor = (objProps[OBJ_VERTEX].size() == 6);

    bool supportFace =
            (objProps[OBJ_FACE].size() == 0 ) ||
            (objProps[OBJ_FACE].size() == 1 &&
             objProps[OBJ_FACE][0].name == PROP_NAME_VERTEX_INDEX ) ||
            (objProps[OBJ_FACE].size() == 4 &&
             objProps[OBJ_FACE][0].name == PROP_NAME_VERTEX_INDEX &&
             objProps[OBJ_FACE][1].name == PROP_NAME_RED &&
             objProps[OBJ_FACE][2].name == PROP_NAME_GREEN &&
             objProps[OBJ_FACE][3].name == PROP_NAME_BLUE);
    bool faceColor = (objProps[OBJ_FACE].size() == 4);

    bool supportEdge =
            (objProps[OBJ_EDGE].size() == 0 ) ||
            (objProps[OBJ_EDGE].size() == 2 &&
             objProps[OBJ_EDGE][0].name == PROP_NAME_VERTEX1 &&
             objProps[OBJ_EDGE][1].name == PROP_NAME_VERTEX2 ) ||
            (objProps[OBJ_EDGE].size() == 5 &&
             objProps[OBJ_EDGE][0].name == PROP_NAME_VERTEX1 &&
             objProps[OBJ_EDGE][1].name == PROP_NAME_VERTEX2 &&
             objProps[OBJ_EDGE][2].name == PROP_NAME_RED &&
             objProps[OBJ_EDGE][3].name == PROP_NAME_GREEN &&
             objProps[OBJ_EDGE][4].name == PROP_NAME_BLUE);
    bool edgeColor = (objProps[OBJ_EDGE].size() == 5);

    if (!supportVertex || !supportFace || !supportEdge) {
         SYNC_PRINT(("Unsupported format\n"));
        return 1;
    }

    bool hasColor = vertexColor && edgeColor && faceColor;
    mesh.switchColor(hasColor);

    if (plyFormat == ASCII)
    {
        for (int i = 0; i < objNumber[OBJ_VERTEX]; i++)
        {
            HelperUtils::getlineSafe (input, line);
            istringstream work(line);

            Vector3dd vertex;
            work >> vertex.x() >> vertex.y() >> vertex.z();

            if (hasColor) {
                work >> mesh.currentColor;
                SYNC_PRINT(("Color %d %d %d\n", mesh.currentColor.r(), mesh.currentColor.g(), mesh.currentColor.b()));

            }

            if (work.bad()) {
                SYNC_PRINT(("Corrupted vertex %d <%s>\n", i, line.c_str()));
                return 1;
            }

            mesh.addVertex(vertex);
        }

        for (int i = 0; i <  objNumber[OBJ_FACE]; i++)
        {
            HelperUtils::getlineSafe (input, line);
            istringstream work(line);
            int points;
            work >> points;
            if (work.bad()) {
                SYNC_PRINT(("Corrupted face number %d\n", i));
                return 1;
            }
            if (points != 3) {
                SYNC_PRINT(("We only support faces with 3 sides not %d\n", points));
                continue;
            }         
            Vector3d32 face;
            work >> face.x() >> face.y() >> face.z();

            if (work.bad()) {
                SYNC_PRINT(("Corrupted face number %d\n", i));
                return 1;
            }

            if (hasColor) {
                work >> mesh.currentColor;
                SYNC_PRINT(("Color %d %d %d\n", mesh.currentColor.r(), mesh.currentColor.g(), mesh.currentColor.b()));
            }

            if (!face.isInCube(
                    Vector3d32(                        0,                         0,                          0),
                    Vector3d32(objNumber[OBJ_VERTEX] - 1, objNumber[OBJ_VERTEX] - 1, objNumber[OBJ_VERTEX] - 1)))
            {
                SYNC_PRINT(("Vertex index is wrong on face %i\n", i));
                return 1;
            }
            SYNC_PRINT(("%d %d %d\n", face.x(), face.y(), face.z()));

            mesh.addFace(face);
        }

        for (int i = 0; i <  objNumber[OBJ_EDGE]; i++)
        {
            HelperUtils::getlineSafe (input, line);
            istringstream work(line);
            Vector2d32 edge;

            work >> edge.x() >> edge.y();

            if (work.bad()) {
                SYNC_PRINT(("Corrupted edge number %d\n", i));
                return 1;
            }

            if (hasColor) {
                work >> mesh.currentColor;
                SYNC_PRINT(("Color %d %d %d\n", mesh.currentColor.r(), mesh.currentColor.g(), mesh.currentColor.b()));

            }

            if (!edge.isInHypercube(
                    Vector2d32(                        0,                         0),
                    Vector2d32(objNumber[OBJ_VERTEX] - 1, objNumber[OBJ_VERTEX] - 1)))
            {
                SYNC_PRINT(("Vertex index is wrong on edge %i\n", i));
                return 1;
            }
            SYNC_PRINT(("Edge %d %d\n", edge.x(), edge.y()));

            mesh.addEdge(edge);
        }
    } else {
        for (int i = 0; i < objNumber[OBJ_VERTEX]; i++)
        {
            float f;
            Vector3dd vertex;
            input.read((char *)&f, sizeof(f)); vertex.x() = f;
            input.read((char *)&f, sizeof(f)); vertex.y() = f;
            input.read((char *)&f, sizeof(f)); vertex.z() = f;
            mesh.vertexes.push_back(vertex);
        }
    }

    return 0;
}


PLYLoader::~PLYLoader()
{
}

// PROP_TYPE_FLOAT,
// PROP_TYPE_UCHAR,
// PROP_TYPE_INT,
// PROP_TYPE_LIST,

istream &operator >>(istream &in, PLYLoader::Prop &toLoad)
{
    string type;
    in >> type;
    if (in.bad()) {
        return in;
    }
    if (type == "float") toLoad.type = PLYLoader::PROP_TYPE_FLOAT;
    if (type == "uchar") toLoad.type = PLYLoader::PROP_TYPE_UCHAR;
    if (type == "int")   toLoad.type = PLYLoader::PROP_TYPE_INT;
    if (type == "list") {
        toLoad.type = PLYLoader::PROP_TYPE_LIST;
        string dummy;
        in >> dummy; // elementType
        in >> dummy; // indexType
    }

    if (toLoad.type == PLYLoader::PROP_TYPE_CORRUPT) {
        return in;
    }

    /* loading name */
    /*PROP_NAME_X,
    PROP_NAME_Y,
    PROP_NAME_Z,

    PROP_NAME_RED,
    PROP_NAME_GREEN,
    PROP_NAME_BLUE,

    PROP_NAME_VECTOR1,
    PROP_NAME_VECTOR2,*/

    string name;
    in >> name;
    if (in.bad()) {
        return in;
    }

    if (name == "x") toLoad.name = PLYLoader::PROP_NAME_X;
    if (name == "y") toLoad.name = PLYLoader::PROP_NAME_Y;
    if (name == "z") toLoad.name = PLYLoader::PROP_NAME_Z;

    if (name == "red")   toLoad.name = PLYLoader::PROP_NAME_RED;
    if (name == "green") toLoad.name = PLYLoader::PROP_NAME_GREEN;
    if (name == "blue")  toLoad.name = PLYLoader::PROP_NAME_BLUE;

    if (name == "vertex1") toLoad.name = PLYLoader::PROP_NAME_VERTEX1;
    if (name == "vertex2") toLoad.name = PLYLoader::PROP_NAME_VERTEX2;

    if (name == "vertex_index") toLoad.name = PLYLoader::PROP_NAME_VERTEX_INDEX;

    return in;

}

} // namespace corecvs
