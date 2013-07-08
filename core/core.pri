# This file should be included by any project outside that wants to use core's files.
#
# input1 parameter: $$COREDIR    - path to core project files
# input2 parameter: $$TARGET     - the current project output name
# output parameter: $$COREBINDIR - path to the output|used core library
#

CORE_INCLUDEPATH = \
    $$COREDIR/src/alignment \
#   $$COREDIR/src/alignment/camerasCalibration \    # used via path
    $$COREDIR/src/assignment \
    $$COREDIR/src/automotive \
#   $$COREDIR/src/automotive/simulation \           # not used, obsolete
    $$COREDIR/src/boosting \
    $$COREDIR/src/buffers \
#   $$COREDIR/src/buffers/converters \              # not used
    $$COREDIR/src/buffers/fixeddisp \
    $$COREDIR/src/buffers/flow \
    $$COREDIR/src/buffers/histogram \
    $$COREDIR/src/buffers/kernels \
#   $$COREDIR/src/buffers/kernels/fastconverter \   # not used ?
    $$COREDIR/src/buffers/kernels/fastkernel \
    $$COREDIR/src/buffers/memory \
    $$COREDIR/src/buffers/morphological \
    $$COREDIR/src/buffers/rgb24 \
#   $$COREDIR/src/buffers/voxels \                  # not used
    $$COREDIR/src/cammodel \
#   $$COREDIR/src/clegacy \                         # not used ?
#   $$COREDIR/src/clegacy/math \                    # not used
    $$COREDIR/src/fileformats \
    $$COREDIR/src/filters \
    $$COREDIR/src/filters/blocks \
    $$COREDIR/src/function \
    $$COREDIR/src/geometry \
    $$COREDIR/src/kalman \
    $$COREDIR/src/kltflow \
    $$COREDIR/src/math \
#   $$COREDIR/src/math/avx \                        # not used
#   $$COREDIR/src/math/fixed \                      # not used
    $$COREDIR/src/math/generic \
    $$COREDIR/src/math/matrix \
#   $$COREDIR/src/math/neon \
    $$COREDIR/src/math/sse \
    $$COREDIR/src/math/vector \
    $$COREDIR/src/meanshift \
    $$COREDIR/src/rectification \
    $$COREDIR/src/reflection \
    $$COREDIR/src/segmentation \
#    $$COREDIR/src/serializer \                      # not used
    $$COREDIR/src/stats \
    $$COREDIR/src/tbbwrapper \
    $$COREDIR/src/tinyxml \
    $$COREDIR/src/utils \
    $$COREDIR/src/utils/visitors \
    $$COREDIR/src/clustering3d \
    $$COREDIR/src/xml \
    $$COREDIR/src/xml/generated \                   # to allow including of generated headers without directory name prefix



INCLUDEPATH += $$CORE_INCLUDEPATH

COREBINDIR = $$COREDIR/../../bin

contains(TARGET, core-3vi): !contains(TARGET, core-3vi-restricted) {
    win32-msvc* {
        DEPENDPATH += $$COREDIR/src/xml             # helps to able including sources by generated.pri from their dirs
    }
    else {
        DEPENDPATH += \
#           $$COREDIR \
            $$CORE_INCLUDEPATH                      # msvc sets this automatically by deps from includes for this project
    }
} else {
    !win32-msvc*: DEPENDPATH += $$CORE_INCLUDEPATH  # msvc sets this automatically by deps from includes for other projects! :(

    CORE_TARGET_NAME = core-3vi
    CORE_TARGET_NAME = $$join(CORE_TARGET_NAME,,,$$BUILD_CFG_SFX)

    LIBS = \
        -L$$COREBINDIR \
        -l$$CORE_TARGET_NAME \
        $$LIBS

    win32-msvc* {
        CORE_TARGET_NAME = $$join(CORE_TARGET_NAME,,,.lib)
        PRE_TARGETDEPS  += $$COREBINDIR/$$CORE_TARGET_NAME
    } else {
        CORE_TARGET_NAME = $$join(CORE_TARGET_NAME,,lib,.a)
        PRE_TARGETDEPS  += $$COREBINDIR/$$CORE_TARGET_NAME
    }
}
