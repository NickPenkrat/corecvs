# This file must be included for any external usage of cvs_utils library!
#
# Input1 parameter  - $$UTILSDIR
# Input2 parameter  - $$TARGET
# Output parameters - $$UTILS_BINDIR
#

# Utils lib uses core, core-res. These includes must be before adding utils lib for mingw linker!
# But they create unneccessary dependence core->utils, although they are not linked together, but should!
#
COREDIR = $$PWD/../core
include($$COREDIR/core.pri)                         # it uses COREDIR, TARGET and detects     COREBINDIR!

!exists($$COREDIR/core.pri) {
    message(Utils from <$$UTILSDIR> rely on the core which is expected here <$$COREDIR/core.pri>. PWD: $$PWD)
}


UTILS_INCLUDEPATH = \
    $$UTILSDIR \
    $$UTILSDIR/3d \
    $$UTILSDIR/camcalc \
    $$UTILSDIR/corestructs \
    $$UTILSDIR/corestructs/coreWidgets \
    $$UTILSDIR/corestructs/libWidgets \
    $$UTILSDIR/corestructs/parametersMapper \
    $$UTILSDIR/distortioncorrector \                # include isn't used, but need for DEPENDPATH!
    $$UTILSDIR/fileformats \
    $$UTILSDIR/filters \
    $$UTILSDIR/filters/graph \
    $$UTILSDIR/filters/ui \                         # include isn't used, but need for DEPENDPATH!
#   $$UTILSDIR/flowcolorers \
    $$UTILSDIR/framesources \
    $$UTILSDIR/framesources/directShow \
    $$UTILSDIR/framesources/decoders \
    $$UTILSDIR/framesources/v4l2 \
    $$UTILSDIR/processor \
    $$UTILSDIR/rectifier \
#   $$UTILSDIR/serializer \     # obsolete?
    $$UTILSDIR/statistics \     # obsolete?
    $$UTILSDIR/uis \
    $$UTILSDIR/uis/cloudview \
    $$UTILSDIR/visitors \
    $$UTILSDIR/widgets \

INCLUDEPATH += $$UTILS_INCLUDEPATH

UTILS_BINDIR = $$UTILSDIR/../../../bin

contains(TARGET, cvs_utils) {
    !win32-msvc*: DEPENDPATH += $$UTILS_INCLUDEPATH # it should set automatically by dependencies from includes!
                                                    # Nevertheless mingw compiler requires this when utils's ui-sources use utils's ui-sources.
} else {
    !win32-msvc*: DEPENDPATH += $$UTILS_INCLUDEPATH # msvc sets this automatically by deps from includes for other projects! :(

    # The "UTILS_INTDIR" path should be known for other projects that use "cvs_utils"!
    # For the cvs_utils itself it's set automatically.
    win32 {
        UTILS_INTDIR = $$UTILSDIR/../../../.obj/cvs_utils/$$BUILD_CFG_NAME
    } else {
        UTILS_INTDIR = $$UTILSDIR/../../../.obj/cvs_utils
    }
    # this is needed as some new sources of cvs_utils use ui-sources from this project!
    INCLUDEPATH += $$UTILS_INTDIR                       # add autogenerated ui* headers of cvs_utils into include path for the current project

    UTILS_TARGET_NAME = cvs_utils
    UTILS_TARGET_NAME = $$join(UTILS_TARGET_NAME,,,$$BUILD_CFG_SFX)

    LIBS = \
        -L$$UTILS_BINDIR\
        -l$$UTILS_TARGET_NAME \
        $$LIBS

    win32-msvc* {
        UTILS_TARGET_NAME = $$join(UTILS_TARGET_NAME,,,.lib)
        PRE_TARGETDEPS   += $$UTILS_BINDIR/$$UTILS_TARGET_NAME
    } else {
        UTILS_TARGET_NAME = $$join(UTILS_TARGET_NAME,,lib,.a)
        PRE_TARGETDEPS   += $$UTILS_BINDIR/$$UTILS_TARGET_NAME
    }
}

# Note: debug and release libs will be overwritten on !win32 config!
#
DESTDIR = $$UTILS_BINDIR


CONFIG += with_opengl                           # always include here OpenGL dependent modules as utils's and related projects need this!
with_opengl {
    QT += opengl                                # this must be defined for utils's and all related sources

   #unix:LIBS += -lGL -lQtOpenGL  # Qt must add this

    win32 {
        LIBS += -lglu32 -lopengl32              # these libs must be exactly here: before openCV but after our libs!!! It's a magic of mingw, for msvc it's easier.:)
    } else {
        LIBS += -lXtst -lX11 -lXext -lGLU       # these libs must be exactly here: they're required by OpenGL and some other stuff...
    }

    INCLUDEPATH += $$UTILSDIR/opengl

    with_openglext {
        DEFINES += WITH_OPENGLEXT
    }
}

with_ueye {
    UEYE_PATH = $$(UEYE_PATH)

    win32 {
        isEmpty(UEYE_PATH): UEYE_PATH = "C:/Program Files/IDS/uEye"

        # Try to use provided install path
        exists($$UEYE_PATH/Develop/lib/uEye_api_64.lib) {
            DEFINES += WITH_UEYE

            INCLUDEPATH += $$UEYE_PATH/Develop/include

            LIBS += \
                -L$$UEYE_PATH/Develop/lib \
                -luEye_api_64 \

            !build_pass:message(Using uEye <$$UEYE_PATH>)
        } else {
            !build_pass:message(Unable to find uEye at <$$UEYE_PATH>)
        }
    } else:exists("/usr/lib/libueye_api.so") {
        LIBS += -lueye_api \

        DEFINES += WITH_UEYE
    } else {
        !build_pass:message(Unable to find uEye at "/usr/lib/libueye_api.so")
    }
}

with_opencv {                                   # all this stuff was extracted from opencv.pri to speedup including
    OPENCV_WRAPPER_DIR = $$UTILSDIR/../wrappers/opencv
    include($$OPENCV_WRAPPER_DIR/opencvLibs.pri)

    contains(DEFINES, WITH_OPENCV) {
        INCLUDEPATH += $$OPENCV_WRAPPER_DIR
        INCLUDEPATH += $$OPENCV_WRAPPER_DIR/faceDetect
    }
}

with_directshow {
    DIRECT_SHOW_WRAPPER_DIR = $$UTILSDIR/../wrappers/directShow
    include($$DIRECT_SHOW_WRAPPER_DIR/directShowLibs.pri)
}


##############################################
# Useful common part for all cvs project apps
##############################################

PROJ_INTDIR = $$UTILSDIR/../../../.obj/$$TARGET

win32 {
    win32-msvc* {
        OBJECTS_DIR = $$PROJ_INTDIR/$$BUILD_CFG_NAME
    } else {
        OBJECTS_DIR = $$PROJ_INTDIR/$$BUILD_CFG_NAME
    }
} else {
    OBJECTS_DIR = $$PROJ_INTDIR
}
MOC_DIR = $$OBJECTS_DIR                             # not PROJ_INTDIR as the compiler regenerates them if config has been changed
UI_DIR  = $$OBJECTS_DIR
RCC_DIR = $$OBJECTS_DIR

# Note: debug and release libs will be overwritten on !win32 only
#
TARGET_ORIG = $$TARGET
TARGET      = $$join(TARGET,,,$$BUILD_CFG_SFX)      # add 'd' at the end for debug win32 version

DESTDIR = $$UTILS_BINDIR

# to delete also target lib by 'clean' make command (distclean does this)
win32 {
    QMAKE_CLEAN += "$(DESTDIR_TARGET)"              # for linux qmake doesn't generate DESTDIR_TARGET :(
} else {
    QMAKE_CLEAN += "$(DESTDIR)$(TARGET)"            # for win such cmd exists with inserted space :(
}
