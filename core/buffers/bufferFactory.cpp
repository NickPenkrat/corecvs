/**
 * \file bufferFactory.cpp
 * \brief Holds the implementation of the factory for the loadable buffer
 *
 * \ingroup cppcorefiles
 * \date Jun 22, 2010
 * \author alexander
 */
#include "bufferFactory.h"
#include "ppmLoader.h"
#include "rawLoader.h"
#include "bmpLoader.h"

namespace corecvs {

CountedPtr<BufferFactory> BufferFactory::sThis;

//static
BufferFactory* BufferFactory::getInstance()
{
    CORE_ASSERT_TRUE(sThis.get() != NULL, "Out of memory!");

    if (sThis.get()->mLoadersG12.size() == 0)
    {
        sThis.get()->registerLoader(new PPMLoaderG12());
        sThis.get()->registerLoader(new RAWLoader());
        sThis.get()->registerLoader(new BMPLoaderG12());
        sThis.get()->registerLoaderG16(new PPMLoaderG16());     // specific reader to load 16bits data PPMs

        sThis.get()->registerLoader(new PPMLoaderRGB24());
        sThis.get()->registerLoader(new BMPLoaderRGB24());

        sThis.get()->registerSaver(new BMPSaverRGB24());
    }
    return sThis.get();
}

void BufferFactory::printCaps()
{
    BufferFactory *factory = BufferFactory::getInstance();

    cout << "BufferFactory knows:" << endl;

    cout << "  G12 loader" << endl;
    for (auto it : factory->mLoadersG12) cout << "\t" << it->name() << endl;

    cout << "  G16 loader" << endl;
    for (auto it : factory->mLoadersG16) cout << "\t" << it->name() << endl;

    cout << "  RGB24 loader" << endl;
    for (auto it : factory->mLoadersRGB24) cout << "\t" << it->name() << endl;

    cout << "  RuntimeType loader" << endl;
    for (auto it : factory->mLoadersRuntime) cout << "\t" << it->name() << endl;

    cout << "----" << endl;

    cout << "  RGB24 saver" << endl;
    for (auto it : factory->mSaversRGB24) cout << "\t" << it->name() << endl;

}

template<typename BufferType>
BufferType *loadBuffer(string name, vector<BufferLoader<BufferType> *> &loaders)
{
    for (auto it : loaders)
    {
        //SYNC_PRINT(("BufferFactory::load(%s): loader <%s>\n", name.c_str(), it->name().c_str()));

        if (!(it->acceptsFile(name)))
            continue;

        BufferType *result = NULL;
        try {
            result = it->load(name);
        }
        catch (std::exception &)
        {
            SYNC_PRINT(("BufferFactory::load(): loader <%s> violates contract by throwing unexpected exception", it->name().c_str()));
        }

        if (result != NULL) {
            return result;
        }

        SYNC_PRINT(("BufferFactory::load(%s):  loader <%s> agreed to load, but failed\n", name.c_str(), it->name().c_str()));
    }
    return NULL;
}


template<typename BufferType>
bool saveBuffer(BufferType *buffer, string name, string preferedProvider, vector<BufferSaver<BufferType> *> &savers)
{
    SYNC_PRINT(("BufferFactory::save(%s)", name.c_str()));

    BufferSaver<BufferType> *saver = NULL;

    for (auto it : savers)
    {
        if (!(it->acceptsFile(name)))
            continue;

        if (saver == NULL) saver = it;

        if (it->name() == preferedProvider)
        {
            saver = it;
        }
    }

    if (saver) {
        return saver->save(*buffer, name);
    }
    return false;
}


G12Buffer *BufferFactory::loadG12Bitmap(string name)
{
    return loadBuffer(name, mLoadersG12);
}

G12Buffer *BufferFactory::loadG16Bitmap(string name)
{
    return loadBuffer(name, mLoadersG16);
}

bool BufferFactory::saveRGB24Bitmap(RGB24Buffer *buffer, std::string name, std::string saverHint)
{
    return saveBuffer(buffer, name, saverHint, mSaversRGB24);
}

RGB24Buffer *BufferFactory::loadRGB24Bitmap(string name)
{
    return loadBuffer(name, mLoadersRGB24);
}

RuntimeTypeBuffer *BufferFactory::loadRuntimeTypeBitmap(std::string name)
{
    return loadBuffer(name, mLoadersRuntime);
}

std::vector<std::string> BufferFactory::resolutionsRGB24()
{
    return std::vector<std::string>();
}

std::vector<std::string> BufferFactory::resolutionsG12()
{
    return std::vector<std::string>();
}

std::vector<std::string> BufferFactory::resolutionsRuntimeType()
{
    return std::vector<std::string>();
}

BufferFactory::~BufferFactory()
{
    // delete all registered loaders

    for (auto it : mLoadersG12) { delete_safe(it); }
    mLoadersG12.clear();

    for (auto it : mLoadersG16) { delete_safe(it); }
    mLoadersG16.clear();

    for (auto it : mLoadersRGB24) { delete_safe(it); }
    mLoadersRGB24.clear();

    for (auto it : mLoadersRuntime) { delete_safe(it); }
    mLoadersRuntime.clear();

    for (auto it : mSaversRGB24) { delete_safe(it); }
    mSaversRGB24.clear();

    //printf("BufferFactory has been destroyed.\n");
}

} //namespace corecvs
