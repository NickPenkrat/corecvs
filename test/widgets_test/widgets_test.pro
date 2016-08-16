# try use global config
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
    include($$ROOT_DIR/config.pri)
} else {
    message(Using local config)
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

QT       += core

TEMPLATE = app
TARGET   = widgets_test
CONFIG  += console

#include(../../core/core.pri)
include($$ROOT_DIR/src/open/utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!

TARGET_ORIG = $$TARGET
TARGET      = $$join(TARGET,,,$$BUILD_CFG_SFX)  # add 'd' at the end for debug versions

OBJECTS_DIR = $$ROOT_DIR/.obj/$$TARGET_ORIG$$BUILD_CFG_NAME
MOC_DIR  = $$OBJECTS_DIR
UI_DIR   = $$OBJECTS_DIR
RCC_DIR  = $$OBJECTS_DIR


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

DESTDIR  = $$ROOT_DIR/bin

SOURCES += \
    main_widgets_test.cpp \

GEN_FOLDER="../../tools/generator/Generated"
exists($$GEN_FOLDER/testClass.cpp) {
    message(Adding generator test)

    SOURCES += $$GEN_FOLDER/testClass.cpp
    HEADERS += $$GEN_FOLDER/testClass.h
    INCLUDEPATH += $$GEN_FOLDER

    DEFINES += INCLUDE_EXAMPLE
}
