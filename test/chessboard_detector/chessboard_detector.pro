 CONFIG += disable_cusparse
#CONFIG += disable_blas		# when open will be linked corecvs.lib that requires SparseMatrix math
#CONFIG += disable_tbb		# when open will be linked corecvs.lib that requires tbb.lib

# try use global config 
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    include($$ROOT_DIR/config.pri)
} else { 
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

TEMPLATE = app
TARGET   = chessboard_detector
CONFIG  += console
QT 	-= core gui xml

#CONFIG -= with_opencv
#!win32: CONFIG += with_libjpeg with_libpng

include(../../core/core.pri)

#include(../../utils/utils.pri)                      # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!
DESTDIR     = $$ROOT_DIR/bin
OBJECTS_DIR = $$ROOT_DIR/.obj/$$TARGET$$BUILD_CFG_NAME
MOC_DIR     = $$OBJECTS_DIR

with_opencv {                                       # all this stuff was extracted from opencv.pri to speedup including
    OPENCV_WRAPPER_DIR = $$PWD/../../wrappers/opencv
    include($$OPENCV_WRAPPER_DIR/opencvLibs.pri)

    contains(DEFINES, WITH_OPENCV) {
        INCLUDEPATH += $$OPENCV_WRAPPER_DIR

        HEADERS += $$OPENCV_WRAPPER_DIR/openCvFileReader.h   $$OPENCV_WRAPPER_DIR/openCVTools.h
        SOURCES += $$OPENCV_WRAPPER_DIR/openCvFileReader.cpp $$OPENCV_WRAPPER_DIR/openCVTools.cpp
    }
}

with_libjpeg {
    LIBJPEG_WRAPPER_DIR = $$PWD/../../wrappers/libjpeg
    include($$LIBJPEG_WRAPPER_DIR/libjpeg.pri)
}

with_libpng {
    LIBPNG_WRAPPER_DIR = $$PWD/../../wrappers/libpng
    include($$LIBPNG_WRAPPER_DIR/libpng.pri)
}


SOURCES += main_chessboard_detector.cpp
