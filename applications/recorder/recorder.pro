include(../../../config.pri)

TARGET   = virecorder
TEMPLATE = app

HOSTBASE_DIR=../host-base
include ($$HOSTBASE_DIR/hostBase.pri)                   # it uses HOSTBASE_DIR, detects HOSTBASE_BINDIR, OBJECTS_DIR, ...

INCLUDEPATH += .

HEADERS += \
    recorderDialog.h \
    recorderThread.h \
    recorderControlWidget.h \                           # control widgets
    generatedParameters/recorder.h \                    # parameters for calculation threads, host dialogs etc.
    parametersMapper/parametersMapperRecorder.h \       # parameters for params mapper

SOURCES += \
    main_virecorder.cpp \
    recorderDialog.cpp \
    recorderThread.cpp \
    recorderControlWidget.cpp \                         # control widgets
    generatedParameters/recorder.cpp \                  # parameters for calculation threads, host dialogs etc.
    parametersMapper/parametersMapperRecorder.cpp \     # parameters for params mapper

FORMS += \
    ui/recorderControlWidget.ui \

