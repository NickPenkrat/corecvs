# try use global config
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
    include($$ROOT_DIR/config.pri)
} else {
    #message(Using local config)
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

QT       += core

TEMPLATE = app
TARGET   = raytracerender
CONFIG  += console

#include(../../core/core.pri)
include(../../utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!


with_libjpeg {
    LIBJPEG_WRAPPER_DIR = $$ROOT_DIR/src/open/wrappers/libjpeg
    include($$LIBJPEG_WRAPPER_DIR/libjpeg.pri)
#    include($$LIBJPEG_WRAPPER_DIR/libjpegLibs.pri)

    contains(DEFINES, WITH_LIBJPEG) {
        INCLUDEPATH += $$LIBJPEG_WRAPPER_DIR
    }
}

with_libpng {
    message(We have libpng)
    LIBPNG_WRAPPER_DIR = $$ROOT_DIR/src/open/wrappers/libpng
    include($$LIBPNG_WRAPPER_DIR/libpng.pri)
#    include($$LIBPNG_WRAPPER_DIR/libpngLibs.pri)

    contains(DEFINES, WITH_LIBPNG) {
        INCLUDEPATH += $$LIBPNG_WRAPPER_DIR
    }
}

SOURCES += \
    main_raytracerender.cpp \
    scene_scanner.cpp       \
    scene_speedup.cpp       \
    scene_pole.cpp \
    scene_test1.cpp \
    scene_large.cpp

