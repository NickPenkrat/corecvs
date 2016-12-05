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
TARGET   = imageview

#include($$ROOT_DIR/src/open/utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!
# Here relative path is preferred to be independant of $$ROOT_DIR
include(../../utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!


INCLUDEPATH += .

HEADERS += \
    imageViewMainWindow.h

SOURCES += \
    main_imageview.cpp \
    imageViewMainWindow.cpp

FORMS += \
    imageViewMainWindow.ui

#RESOURCES += ../../resources/main.qrc
