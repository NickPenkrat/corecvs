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

//#if __cplusplus > 199711L
#if defined(WIN32) && (_MSC_VER < 1900) // we need a threadsafety singleton initialization described in �6.7.4 of the C++11 standard, msvc2013 doesn't support it fully... Don't care about gcc-versions
#include <mutex>
#include <atomic>
static std::mutex mtx;
std::atomic<corecvs::BufferFactory *> instance(nullptr);
#endif

namespace corecvs {

//static
BufferFactory* BufferFactory::getInstance()
{
#if defined(WIN32) && (_MSC_VER < 1900) // we need a threadsafety singleton initialization
    auto *p = instance.load(std::memory_order_acquire);
    if (!p)
    {
        std::lock_guard<std::mutex> lck(mtx);
        p = instance.load(std::memory_order_relaxed);
        if (!p)
        {
            p = new BufferFactory;
            instance.store(p, std::memory_order_release);
        }
    }
    return p;
#else
    static BufferFactory instance;
    return &instance;
#endif
}

BufferFactory::BufferFactory()
{
    registerLoader(new PPMLoaderG12());
    registerLoader(new RAWLoader());
    registerLoader(new BMPLoaderG12());
    registerLoaderG16(new PPMLoaderG16());     // specific reader to load 16bits data PPMs

    registerLoader(new PPMLoaderRGB24());
    registerLoader(new BMPLoaderRGB24());

    registerSaver(new BMPSaverRGB24());
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
    bool found = false;
    for (auto it : loaders)
    {
        //SYNC_PRINT(("BufferFactory::load(%s): loader <%s>\n", name.c_str(), it->name().c_str()));

        if (!(it->acceptsFile(name)))
            continue;
        found = true;

        BufferType *result = NULL;
        try {
            result = it->load(name);
        }
        catch (std::exception &)
        {
            SYNC_PRINT(("BufferFactory::load(): loader <%s> violates contract by throwing unexpected exception\n", it->name().c_str()));
        }

        if (result != NULL) {
            return result;
        }

        SYNC_PRINT(("BufferFactory::load(%s):  loader <%s> agreed to load, but failed\n", name.c_str(), it->name().c_str()));
    }

    if (!found)
        SYNC_PRINT(("BufferFactory::load(%s):  no loaders for it!\n", name.c_str()));

    return NULL;
}

template<typename BufferType>
bool saveBuffer(BufferType *buffer, string name, string preferedProvider, vector<BufferSaver<BufferType> *> &savers)
{
    SYNC_PRINT(("BufferFactory::save(%s, preffered:%s)\n", name.c_str(), preferedProvider.c_str()));

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
