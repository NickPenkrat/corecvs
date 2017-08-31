#pragma once
/**
 * \file bufferFactory.h
 * \brief This file contains the expandable interface for loading bitmaps from files and/or streams
 *
 * \ingroup cppcorefiles
 * \date Jun 22, 2010
 */
#include <string>
#include <vector>

#include "global.h"

#include "g12Buffer.h"
#include "rgb24Buffer.h"
#include "bufferLoader.h"
#include "runtimeTypeBuffer.h"
#include "countedPtr.h"

namespace corecvs {

using std::string;

/** Singleton for buffer loading
 */
class BufferFactory
{
public:
    /** Public function to get the only one instance of this object.
     *  It will be automatically destroyed on shut down.
     */
    static BufferFactory* getInstance();

    static void printCaps();

    /**
     * Public function to add own buffer loader, i.e. QtFileLoader
     **/
    bool registerLoader(BufferLoader<G12Buffer> * loader)
    {
        mLoadersG12.push_back(loader);
        return true;
    }

    bool registerLoaderG16(BufferLoader<G12Buffer> * loader)
    {
        mLoadersG16.push_back(loader);
        return true;
    }

    bool registerLoader(BufferLoader<RGB24Buffer> * loader)
    {
        mLoadersRGB24.push_back(loader);
        return true;
    }

    bool registerLoader(BufferLoader<RuntimeTypeBuffer> * loader)
    {
        mLoadersRuntime.push_back(loader);
        return true;
    }


    bool registerSaver(BufferSaver<RGB24Buffer> * saver)
    {
        mSaversRGB24.push_back(saver);
        return true;
    }

    /** Main function to get a 12-bits buffer from the file with the given path name
     */
    virtual G12Buffer* loadG12Bitmap(string name);

    /** Main function to get a 32-bits buffer from the file with the given path name
     */
    virtual RGB24Buffer* loadRGB24Bitmap(string name);

    /** Main function to get a fixed 8|32 bits buffer from the file with the given path name
     */
    virtual RuntimeTypeBuffer* loadRuntimeTypeBitmap(string name);

    /** Main function to get a 16-bits buffer from the file with the given path name
     */
    virtual G12Buffer* loadG16Bitmap(string name);




    /**
     */
    virtual bool saveRGB24Bitmap(RGB24Buffer* buffer, string name, string saverHint);


    /**
     **/
    virtual std::vector<std::string> resolutionsRGB24();
    virtual std::vector<std::string> resolutionsG12();
    virtual std::vector<std::string> resolutionsRuntimeType();


private:
    friend class CountedPtr<BufferFactory>;                 // this class will create/destroy the factory indeed
    BufferFactory() {}
    virtual ~BufferFactory();

    static CountedPtr<BufferFactory> sThis;

    /**
     * List of loaders - the supported formats
     */
    vector<BufferLoader<G12Buffer>   *> mLoadersG12;
    vector<BufferLoader<G12Buffer>   *> mLoadersG16;
    vector<BufferLoader<RGB24Buffer> *> mLoadersRGB24;
    vector<BufferLoader<RuntimeTypeBuffer> *> mLoadersRuntime;

    //vector<BufferSaver<RuntimeTypeBuffer> *> mSaversRuntime;
    vector<BufferSaver<RGB24Buffer> *> mSaversRGB24;
};

} //namespace corecvs
