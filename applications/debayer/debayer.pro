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
TARGET   = debayer
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
