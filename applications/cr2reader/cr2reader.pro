TEMPLATE = app
TARGET   = cr2reader
QT      += core xml
QT      -= gui
CONFIG  += console
CONFIG  -= app_bundle
CONFIG  += debug

ROOT_DIR=../../../..
include($$ROOT_DIR/config.pri)

OBJECTS_DIR = $$ROOT_DIR/.obj/cr2reader$$BUILD_CFG_NAME
MOC_DIR  = $$OBJECTS_DIR
#UI_DIR  = $$OBJECTS_DIR
#RCC_DIR = $$OBJECTS_DIR
DESTDIR = $$ROOT_DIR/bin

include(../../core/core.pri)

INCLUDEPATH += C:/dev/LibRaw-0.17.0/libraw
LIBS    += C:/dev/LibRaw-0.17.0/msvc-build/Debug/raw.lib

SOURCES += \ 
    cr2reader.cpp \
    main_cr2reader.cpp

HEADERS += \ 
    cr2reader.h

