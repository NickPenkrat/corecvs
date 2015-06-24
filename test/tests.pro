# try use global config
exists(../../../config.pri) {
    #message(Using global config)
    include(../../../config.pri)
} else {
    message(Using local config)
    include(../config.pri)
}

TEMPLATE = subdirs

    grab_N_captures \
#   adoptcolor      \
#   avigrab         \
    serialize1      \


grab_N_captures.file = grab_N_captures/grab_N_captures.pro
serialize1.file    = serialize1/serialize1.pro
