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
TARGET   = merger

HOSTBASE_DIR=../base
include ($$HOSTBASE_DIR/baseApplication.pri)            # it uses HOSTBASE_DIR, detects HOSTBASE_BINDIR, OBJECTS_DIR, ...

INCLUDEPATH += .
INCLUDEPATH += parametersMapper
INCLUDEPATH += generatedParameters

HEADERS += \
    mergerDialog.h \
    mergerThread.h \
    mergerControlWidget.h \                           # control widgets
    generatedParameters/merger.h \                    # parameters for calculation threads, host dialogs etc.
    parametersMapper/parametersMapperMerger.h \       # parameters for params mapper

SOURCES += \
    main_merger.cpp \
    mergerDialog.cpp \
    mergerThread.cpp \
    mergerControlWidget.cpp \                         # control widgets
    generatedParameters/merger.cpp \                  # parameters for calculation threads, host dialogs etc.
    parametersMapper/parametersMapperMerger.cpp \     # parameters for params mapper

FORMS += \
    ui/mergerControlWidget.ui \


OTHER_FILES +=                                          \
    $$ROOT_DIR/src/open/tools/generator/xml/merger.xml  \
    $$ROOT_DIR/src/open/tools/generator/regen-merger.sh


OTHER_FILES += legacy/*

#RESOURCES += ../../resources/main.qrc
