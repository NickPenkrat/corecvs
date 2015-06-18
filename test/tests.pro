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
    serialize1   \
    adoptcolor   \
#    avigrab     \
    decodebayer \



grab.file          = grab/grab.pro
serialize1.file    = serialize1/serialize1.pro
adoptcolor.file    = adoptcolor/adoptcolor.pro
avigrab.file       = avigrab/avigrab.pro
decodebayer.file   = decodebayer/decodebayer.pro

