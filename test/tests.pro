# We are to include config.pri here to detect whether we use opencv or not!
# try use global config
exists(../../../config.pri) {
    #message(Using global config)
    include(../../../config.pri)
} else {
    #message(Using local config)
    include(../cvs-config.pri)
}

TEMPLATE = subdirs

SUBDIRS +=            \
#   grab              \
    grab_N_captures   \
    serialize1        \
    fileloader        \
    deproject         \
    softrender        \
    raytracerender    \
    stereo_generator  \
    widgets_test      \
    widget_harness    \
#    command_harness   \
    example_scene     \
    chessboard_detector \


# SUBDIRS +=     \
#    adoptcolor  \
#    avigrab     \  // Depricated
#    decodebayer \
#    genvectorui \
#    aLowCodec   \
#    new_board_detector \
#    cr2reader           \
#    debayer          \

with_avcodec {
   SUBDIRS +=  avencode
}

with_ceres {
    SUBDIRS += ceres_playground
}

!win32 {
    SUBDIRS += jitplayground
    SUBDIRS += gcodeplayground
}

with_qscript {
   SUBDIRS += qtScriptConsole
}


with_opencv {
    SUBDIRS +=       \
#        distortion_corrector \
#        matcher_basic \
#        matcher_full
}

grab                                = grab/grab.pro
grab_N_captures                     = grab_N_captures/grab_N_captures.pro
serialize1                          = serialize1/serialize1.pro
jitplayground                       = jitplayground/jitplayground.pro
gcodeplayground                     = gcodeplayground/gcodeplayground.pro
example_scene                       = example_scene/example_scene.pro

ceres_playground                    = ceres_playground/ceres_playground.pro

adoptcolor                          = adoptcolor/adoptcolor.pro
avigrab                             = avigrab/avigrab.pro
decodebayer                         = decodebayer/decodebayer.pro
genvectorui                         = genvectorui/genvectorui.pro
aLowCodec                           = aLowCodec/aLowCodec.pro

cr2reader.file                      = cr2reader/cr2reader.pro
cr2reader.depends                  -= utils

debayer.file                        = debayer/debayer.pro
debayer.depends                    -= utils

deproject.file                      = deproject/deproject.pro
avencode.file                       = avencode/avencode.pro

softrender.file                     = softrender/softrender.pro
softrender.depends                 -= utils

raytracerender.file                 = raytracerender/raytracerender.pro
raytracerender.depends             -= utils

widgets_test.file                   = widgets_test/widgets_test.pro
widget_harness.file                 = widget_harness/widget_harness.pro
command_harness.file                = command_harness/command_harness.pro

new_board_detector	            = new_board_detector/new_board_detector.pro

distortion_corrector	            = distortion_corrector/distortion_corrector.pro

matcher_basic                       = matcher_basic/matcher_basic.pro
matcher_full                        = matcher_full/matcher_full.pro

chessboard_detector                 = chessboard_detector/chessboard_detector.pro

qtScriptConsole                     = qtScriptConsole/qtScriptConsole.pro

stereo_generator                    = stereo_generator/stereo_generator.pro

