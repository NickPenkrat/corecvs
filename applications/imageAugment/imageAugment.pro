ROOT_DIR=../../../..
ROOT_DIR=$$PWD/$$ROOT_DIR
include($$ROOT_DIR/config.pri)

TEMPLATE = app
TARGET   = imageAugment
CONFIG  += console
QT      += core
#QT      += gui widgets

include($$ROOT_DIR/git-version.pri)
include($$ROOT_DIR/src/open/utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!


SOURCES += main.cpp \
    blur/blurApplicator.cpp \
    resize/resizeApplicator.cpp

HEADERS += \
    applicator/applicator.h \
    blur/blurApplicator.h \
    resize/resizeApplicator.h

FORMS += \

INCLUDEPATH += applicator
INCLUDEPATH += blur
INCLUDEPATH += resize

DEPENDPATH += applicator

