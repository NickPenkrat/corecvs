include(../../config.pri)

CONFIG  += staticlib
TARGET   = core-3vi
TEMPLATE = lib

COREDIR = .
include(core.pri)               # it uses COREDIR, TARGET and detects COREBINDIR!

CORE_INTDIR = ../../.obj/core
win32 {
    OBJECTS_DIR = $$CORE_INTDIR/$$BUILD_CFG_NAME
} else {
    OBJECTS_DIR = $$CORE_INTDIR
}
MOC_DIR = $$CORE_INTDIR
UI_DIR  = $$CORE_INTDIR

# Note: debug and release libs will be overwritten on !win32 only
#
TARGET  = $$join(TARGET,,,$$BUILD_CFG_SFX)

DESTDIR = $$COREBINDIR

# to delete also target lib by 'clean' make command (distclean does this)
win32 {
    QMAKE_CLEAN += "$(DESTDIR_TARGET)"          # for linux qmake doesn't generate DESTDIR_TARGET :(
} else {
    QMAKE_CLEAN += "$(DESTDIR)$(TARGET)"        # for win such cmd exists with inserted space :(
}

#
# include sources and headers for each subdir
#
include(src/alignment/alignment.pri)
include(src/assignment/assignment.pri)
include(src/automotive/automotive.pri)
include(src/boosting/boosting.pri)
include(src/buffers/buffers.pri)
include(src/cammodel/cammodel.pri)
include(src/fileformats/fileformats.pri)
include(src/filters/filters.pri)
include(src/function/function.pri)
include(src/geometry/geometry.pri)
include(src/kalman/kalman.pri)
include(src/kltflow/kltflow.pri)
include(src/math/math.pri)
include(src/meanshift/meanshift.pri)
include(src/rectification/rectification.pri)
include(src/reflection/reflection.pri)
include(src/segmentation/segmentation.pri)
include(src/stats/stats.pri)
include(src/tbbwrapper/tbbwrapper.pri)
include(src/utils/utils.pri)
include(src/xml/generated/generated.pri)
include(src/clustering3d/clustering3d.pri)
