#include <fstream>

#include "core/utils/utils.h"

#include "core/fileformats/meshLoader.h"
#include "core/fileformats/plyLoader.h"
#include "core/fileformats/stlLoader.h"
#include "core/fileformats/objLoader.h"
#include "core/fileformats/gcodeLoader.h"

namespace corecvs {
using namespace std;

/*
 // Depricated
bool MeshLoader::endsWith(const string &fileName, const char *extention)
{
    size_t extLen = strlen(extention);
    if (fileName.compare(fileName.length() - extLen, extLen, extention) == 0)
        return true;
    return false;
}
*/

MeshLoader::MeshLoader() :
    trace(false)
{

}

static const char *PLY_EXT = ".ply";
static const char *STL_EXT = ".stl";
static const char *OBJ_EXT = ".obj";
static const char *GCODE_EXT = ".gcode";

bool MeshLoader::load(Mesh3D *mesh, const string &fileName)
{
    ifstream file;
    file.open(fileName, ios::in);
    if (file.fail())
    {
        SYNC_PRINT(("MeshLoader::load(): Can't open mesh file <%s>\n", fileName.c_str()));
        return false;
    }

    if (HelperUtils::endsWith(fileName, PLY_EXT))
    {
        SYNC_PRINT(("MeshLoader::load(): Loading PLY <%s>\n", fileName.c_str()));
        PLYLoader loader;
        loader.trace = trace;
        if (loader.loadPLY(file, *mesh) != 0)
        {
           SYNC_PRINT(("MeshLoader::load(): Unable to load mesh\n"));
           file.close();
           return false;
        }
    }

    if (HelperUtils::endsWith(fileName, STL_EXT))
    {
        SYNC_PRINT(("MeshLoader::load(): Loading STL <%s>\n", fileName.c_str()));
        STLLoader loader;
        if (loader.loadBinarySTL(file, *mesh) != 0)
        {
           SYNC_PRINT(("MeshLoader::load(): Unable to load mesh"));
           file.close();
           return false;
        }
    }

    if (HelperUtils::endsWith(fileName, OBJ_EXT))
    {
        SYNC_PRINT(("MeshLoader::load(): Loading OBJ <%s>\n", fileName.c_str()));
        OBJLoader loader;
        if (loader.loadOBJSimple(file, *mesh) != 0)
        {
           SYNC_PRINT(("MeshLoader::load(): Unable to load mesh"));
           file.close();
           return false;
        }
    }

    if (HelperUtils::endsWith(fileName, GCODE_EXT))
    {
        SYNC_PRINT(("MeshLoader::load(): Loading GCODE <%s>\n", fileName.c_str()));
        GcodeLoader loader;
        if (loader.loadGcode(file, *mesh) != 0)
        {
           SYNC_PRINT(("MeshLoader::load(): Unable to load mesh"));
           file.close();
           return false;
        }
    }

    mesh->dumpInfo(cout);
    return true;
}

bool MeshLoader::save(Mesh3D *mesh, const string &fileName)
{
    ofstream file;
    file.open(fileName, ios::out);
    if (file.fail())
    {
        SYNC_PRINT(("MeshLoader::save(): Can't open mesh file <%s> for writing\n", fileName.c_str()));
        return false;
    }

    if (HelperUtils::endsWith(fileName, PLY_EXT))
    {
        SYNC_PRINT(("MeshLoader::save(): Saving PLY <%s>\n", fileName.c_str()));
        PLYLoader loader;
        int res = loader.savePLY(file, *mesh);
        if (res != 0)
        {
           SYNC_PRINT(("MeshLoader::save(): Unable to save mesh code=%d\n", res ));
           file.close();
           return false;
        }
    }

    if (HelperUtils::endsWith(fileName, OBJ_EXT))
    {
        SYNC_PRINT(("MeshLoader::save(): Saving OBJ <%s>\n", fileName.c_str()));
        OBJLoader loader;
        int res = loader.saveOBJSimple(file, *mesh);
        if (res != 0)
        {
           SYNC_PRINT(("MeshLoader::save(): Unable to save mesh code=%d\n", res ));
           file.close();
           return false;
        }
    }

    if (HelperUtils::endsWith(fileName, STL_EXT))
    {
        SYNC_PRINT(("MeshLoader::save(): Saving binary STL <%s>\n", fileName.c_str()));
        STLLoader loader;
        if (loader.saveBinarySTL(file, *mesh) != 0)
        {
           SYNC_PRINT(("MeshLoader::save(): Unable to load mesh"));
           file.close();
           return false;
        }
    }


    return true;
}

std::string MeshLoader::extentionList()
{
    return string("*") + string(PLY_EXT) + " *" + string(STL_EXT) + " *" + string(OBJ_EXT) + " *" + string(GCODE_EXT);
}

} //namespace corecvs
