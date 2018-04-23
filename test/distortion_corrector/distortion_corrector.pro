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

TEMPLATE = app
TARGET   = distortion_corrector
CONFIG  += console
QT 	+= core
QT 	-= xml
QT 	-= widgets
QT 	-= opengl
#QT 	-= gui   # is not allowed as this app uses qtFileLoader, which uses QImage that belongs to QtGui

include(../../utils/utils.pri)                      # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!

SOURCES += distortion_corrector.cpp
