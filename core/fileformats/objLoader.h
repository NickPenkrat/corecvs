#ifndef OBJLOADER_H
#define OBJLOADER_H
/**
 * \file plyLoader.h
 *
 * \date Nov 13, 2012
 **/

#include <iostream>
#include <vector>

#include "global.h"

#include "mesh3d.h"
#include "mesh3DDecorated.h"


#include "rgb24Buffer.h"
namespace corecvs {

using std::vector;

class OBJMaterial {
public:
    enum  {
        KOEF_AMBIENT,
        KOEF_DIFFUSE,
        KOEF_SPECULAR,
        KOEF_LAST
    };

    Vector3dd koefs[KOEF_LAST];

    enum  {
        TEX_AMBIENT,
        TEX_DIFFUSE,
        TEX_SPECULAR,
        TEX_HIGHLIGHT,
        TEX_ALPHA,
        TEX_BUMP,
        TEX_LAST
    };

    RGB24Buffer* tex[TEX_LAST] = {NULL};

    OBJMaterial() {

        for (size_t i = 0; i < CORE_COUNT_OF(koefs); i++)
        {
            koefs[i] = RGBColor::White().toDouble();
        }

        for (size_t j = 0; j < CORE_COUNT_OF(tex); j++)
        {
            tex[j] = NULL;
        }
    }

    friend ostream & operator <<(ostream &out, const OBJMaterial &material)
    {
        for (size_t i = 0; i < CORE_COUNT_OF(material.koefs); i++)
        {
            out << material.koefs[i] << endl;
        }

        for (size_t j = 0; j < CORE_COUNT_OF(material.tex); j++)
        {
            if (material.tex[j] != NULL) {
                out << material.tex[j]->getSize() << endl;
            } else {
                out << "- No -" << endl;
            }
        }
        return out;
    }

};




class OBJLoader {
public:
    bool trace = true;


    OBJLoader();
    virtual ~OBJLoader();
    int loadOBJ(istream &input, Mesh3DDecorated &mesh);
    int loadMaterial(istream &input, OBJMaterial &material, const std::string &path = "");

    int loadOBJSimple(istream &input, Mesh3D &mesh);
    int saveOBJSimple(ostream &out, Mesh3D &mesh);
};

} // namespace corecvs
/* EOF */


#endif // OBJLOADER_H
