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
TARGET   = fisheye_egomotion
CONFIG  += console

#include(../../core/core.pri)
include(../../utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!


TARGET_ORIG = $$TARGET
TARGET      = $$join(TARGET,,,$$BUILD_CFG_SFX)  # add 'd' at the end for debug versions

OBJECTS_DIR = $$ROOT_DIR/.obj/$$TARGET_ORIG$$BUILD_CFG_NAME
MOC_DIR  = $$OBJECTS_DIR
UI_DIR   = $$OBJECTS_DIR
RCC_DIR  = $$OBJECTS_DIR

DESTDIR  = $$ROOT_DIR/bin

!win32 {
    # Linux version
    exists($$ROOT_DIR/bin/libcvs_res_interface.a): exists($$ROOT_DIR/bin/libcvs_core_restricted.a) {
        !build_pass: message(Switching on CVS featuring)
        LIBS            = -L$$ROOT_DIR/bin -lcvs_res_interface -lcvs_core_restricted $$LIBS
        PRE_TARGETDEPS += $$ROOT_DIR/bin/libcvs_res_interface.a $$ROOT_DIR/bin/libcvs_core_restricted.a
        DEFINES += WITH_CVS_FEATURES
    } else {
        !build_pass: message(no bin/libcvs_res_interface(|libcvs_core_restricted).a. No CVS features!)
    }

}

SOURCES += \ 
    main_fisheye_egomotion.cpp \
    modCostFunction.cpp        \
    modPointsCostFunction.cpp   \
    xml/generated/*.cpp        \

HEADERS += \
    modCostFunction.h          \
    modPointsCostFunction.h     \
    xml/generated/*.h          \


INCLUDEPATH += xml/generated/

OTHER_FILES += $$PWD/xml/*.xml
OTHER_FILES += $$PWD/../../tools/generator/regen-fisheye.sh
