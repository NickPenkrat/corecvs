# try use global config
exists(../../../config.pri) {
    #message(Using global config)
    include(../../../config.pri)
} else {
    message(Using local config)
    include(../config.pri)
}

TEMPLATE = subdirs

SUBDIRS +=          \
    grab            \
    grab_N_captures \
    opencvLineDetector \
    xmlserialize    \
#    adoptcolor      \
#    avigrab         \
    topcon          \
#    decodebayer     \



grab.file            = grab/grab.pro
grab_N_captures.file = grab_N_captures/grab_N_captures.pro
opencvLineDetector.file = opencvLineDetector/opencvLineDetector.pro
xmlserialize.file    = xmlserialize/xmlserialize.pro
adoptcolor.file      = adoptcolor/adoptcolor.pro
avigrab.file         = avigrab/avigrab.pro
topcon.file          = topcon/topcon.pro
decodebayer.file     = decodebayer/decodebayer.pro

