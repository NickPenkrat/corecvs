# We are to include config.pri here to detect whether we use opencv or not!
# try use global config
exists(../../../config.pri) {
    #message(Using global config)
    include(../../../config.pri)
} else {
    message(Using local config)
    include(../cvs-config.pri)
}

TEMPLATE = subdirs

SUBDIRS +=       \
#   grab         \
    grab_N_captures \
    serialize1   \
    jitplayground \
    fileloader \
#   adoptcolor   \
#    avigrab     \
#    decodebayer \
#    genvectorui  \
#    aLowCodec \
#    new_board_detector \
#    cr2reader           \
    debayer          \
#    qtScriptConsole  \
    softrender       \
    raytracerender   \
    stereo_generator \
    widgets_test \
    example_scene \


with_opencv {
    SUBDIRS +=       \
#        distortion_corrector \
#        matcher_basic \
#        matcher_full \
        chessboard_detector \

}

grab                                = grab/grab.pro
grab_N_captures                     = grab_N_captures/grab_N_captures.pro
serialize1                          = serialize1/serialize1.pro
jitplayground                       = jitplayground/jitplayground.pro
example_scene                       = example_scene/example_scene.pro
adoptcolor                          = adoptcolor/adoptcolor.pro
avigrab                             = avigrab/avigrab.pro
decodebayer                         = decodebayer/decodebayer.pro
genvectorui                         = genvectorui/genvectorui.pro
aLowCodec                           = aLowCodec/aLowCodec.pro

cr2reader.file                      = cr2reader/cr2reader.pro
cr2reader.depends                  -= utils

debayer.file                        = debayer/debayer.pro
debayer.depends                    -= utils

softrender.file                     = softrender/softrender.pro
softrender.depends                 -= utils

raytracerender.file                 = raytracerender/raytracerender.pro
raytracerender.depends             -= utils

widgets_test.file                   = widgets_test/widgets_test.pro

new_board_detector	            = new_board_detector/new_board_detector.pro

distortion_corrector	            = distortion_corrector/distortion_corrector.pro

matcher_basic                       = matcher_basic/matcher_basic.pro
matcher_full                        = matcher_full/matcher_full.pro

chessboard_detector                 = chessboard_detector/chessboard_detector.pro

qtScriptConsole                     = qtScriptConsole/qtScriptConsole.pro

stereo_generator                    = stereo_generator/stereo_generator.pro
