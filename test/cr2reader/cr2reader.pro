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

TEMPLATE = app
TARGET   = cr2reader

include(../../core/core.pri)

OBJECTS_DIR = $$ROOT_DIR/.obj/cr2reader$$BUILD_CFG_NAME
MOC_DIR  = $$OBJECTS_DIR
#UI_DIR  = $$OBJECTS_DIR
#RCC_DIR = $$OBJECTS_DIR
DESTDIR = $$ROOT_DIR/bin

isEmpty(LIBRAW_DIR) {
    #LIBRAW_DIR=C:/dev/LibRaw-0.17.0            # TODO: use external env.var for this!
}

exists($$LIBRAW_DIR/msvc-build/Debug/raw.lib) {
    INCLUDEPATH += $$LIBRAW_DIR/libraw
    LIBS        += $$LIBRAW_DIR/msvc-build/Debug/raw.lib
} else {
    !build_pass: message(file $$LIBRAW_DIR/msvc-build/Debug/raw.lib not found! The LibRaw is disabled)
}


SOURCES += \ 
    cr2reader.cpp \
    main_cr2reader.cpp

HEADERS += \ 
    cr2reader.h
