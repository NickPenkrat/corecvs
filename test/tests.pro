# try use global config
exists(../../../config.pri) {
    #message(Using global config)
    include(../../../config.pri)
} else {
    message(Using local config)
    include(../config.pri)
}

TEMPLATE = subdirs

SUBDIRS +=       \
    grab         \
    grab_N_captures \
    serialize1   \
#   adoptcolor   \
#    avigrab     \
    decodebayer  \
    genvectorui  \

with_opencv {
    SUBDIRS +=       \
    opencvLineDetector \
}

grab.file            = grab/grab.pro
grab_N_captures.file = grab_N_captures/grab_N_captures.pro
adoptcolor.file      = adoptcolor/adoptcolor.pro
avigrab.file         = avigrab/avigrab.pro
decodebayer.file     = decodebayer/decodebayer.pro
genvectorui.file     = genvectorui/genvectorui.pro
opencvLineDetector.file = opencvLineDetector/opencvLineDetector.pro
serialize1.file      = serialize1/serialize1.pro
adoptcolor.file      = adoptcolor/adoptcolor.pro
avigrab.file         = avigrab/avigrab.pro
topcon.file          = topcon/topcon.pro
decodebayer.file     = decodebayer/decodebayer.pro
