#pragma once
/**
 * \file QTFileLoader.h
 * \brief This file is responsible for adding support for core of all file formats that are
 * supported by QT
 *
 * \date Jun 24, 2010
 * \author alexander
 */
#include <string>

#include <QImage>

#include "global.h"

#include "bufferLoader.h"
#include "g12Buffer.h"
#include "bufferFactory.h"
#include "rgb24Buffer.h"

class QTemporaryFile;

class QTFileLoader
{
public:
    QTFileLoader() {}
    virtual ~QTFileLoader() {}

    static corecvs::RGB24Buffer *RGB24BufferFromQImage(QImage *image);
    static QImage* RGB24BufferToQImage(corecvs::RGB24Buffer &buffer);
    static void save(const std::string& name, corecvs::RGB24Buffer *input, int quality = 95);
    static QTemporaryFile* saveTemporary(const QImage& image);
};

class QTG12Loader : public corecvs::BufferLoader<corecvs::G12Buffer>
{
public:
    static int registerMyself()
    {
        corecvs::BufferFactory::getInstance()->registerLoader(new QTG12Loader());
        return 0;
    }

    virtual bool acceptsFile(std::string name);
    virtual corecvs::G12Buffer * load(std::string name);

    virtual std::string name() override { return "QTG12Loader(multitype)"; }

    virtual ~QTG12Loader();
};

class QTRGB24Loader : public corecvs::BufferLoader<corecvs::RGB24Buffer>
{
public:
    static int registerMyself()
    {
        corecvs::BufferFactory::getInstance()->registerLoader(new QTRGB24Loader());
        return 0;
    }

    virtual bool acceptsFile(std::string name);
    virtual corecvs::RGB24Buffer * load(std::string name);
    virtual std::string name() override { return "QTRGB24Loader(multitype)"; }

    virtual ~QTRGB24Loader();
};

class QTRuntimeLoader : public corecvs::BufferLoader<corecvs::RuntimeTypeBuffer>
{
public:
    static int registerMyself()
    {
        corecvs::BufferFactory::getInstance()->registerLoader(new QTRuntimeLoader());
        return 0;
    }

    virtual bool acceptsFile(std::string name);
    virtual corecvs::RuntimeTypeBuffer * load(std::string name);
    virtual std::string name() override { return "QTRuntimeLoader(multitype)"; }

    virtual ~QTRuntimeLoader();
};
