ROOT_DIR=../../../..
include($$ROOT_DIR/config.pri)

TEMPLATE = app
TARGET   = vcprojFix
CONFIG  += console
QT      -= gui

DESTDIR     = $$ROOT_DIR/bin
OBJECTS_DIR = $$ROOT_DIR/.obj/$$TARGET$$BUILD_CFG_NAME
MOC_DIR     = $$OBJECTS_DIR

SOURCES += main_vcprojFix.cpp
