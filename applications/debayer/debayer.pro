TEMPLATE = app
TARGET   = debayer
QT      += core xml
QT      -= gui
CONFIG  += console
CONFIG  -= app_bundle
CONFIG  += debug

ROOT_DIR=../../../..
include($$ROOT_DIR/config.pri)

OBJECTS_DIR = $$ROOT_DIR/.obj/debayer$$BUILD_CFG_NAME
MOC_DIR  = $$OBJECTS_DIR
#UI_DIR  = $$OBJECTS_DIR
#RCC_DIR = $$OBJECTS_DIR

include(../../core/core.pri)

INCLUDEPATH += converters

SOURCES += \ 
    main_debayer.cpp

HEADERS += \ 
#    cr2reader.h
