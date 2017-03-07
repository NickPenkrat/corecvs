# try use global config
exists(../../../../config.pri) {
    #message(Using global config)
    ROOT_DIR=../../../..
    include($$ROOT_DIR/config.pri)
} else {
    message(Using local config)
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}


DESTDIR = $$ROOT_DIR/bin
OBJECTS_DIR = $$ROOT_DIR/.obj/$$TARGET_ORIG$$BUILD_CFG_NAME
MOC_DIR  = $$OBJECTS_DIR
UI_DIR   = $$OBJECTS_DIR
RCC_DIR  = $$OBJECTS_DIR

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

include($$PWD/../../core/core.pri)

SOURCES += main.cpp

with_libjpeg {
    LIBJPEG_WRAPPER_DIR = $$ROOT_DIR/src/open/wrappers/libjpeg
    include($$LIBJPEG_WRAPPER_DIR/libjpeg.pri)

    contains(DEFINES, WITH_LIBJPEG) {
        INCLUDEPATH += $$LIBJPEG_WRAPPER_DIR
    }
}

with_libpng {
    message(We have libpng)
    LIBPNG_WRAPPER_DIR = $$ROOT_DIR/src/open/wrappers/libpng
    include($$LIBPNG_WRAPPER_DIR/libpng.pri)

    contains(DEFINES, WITH_LIBPNG) {
        INCLUDEPATH += $$LIBPNG_WRAPPER_DIR
    }
}




OTHER_FILES +=

HEADERS +=
