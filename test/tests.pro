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
#    serialize1   \
#   adoptcolor   \
#    avigrab     \
#    decodebayer \
#    genvectorui  \
#    aLowCodec \
#    new_board_detector \
#    cr2reader           \
    debayer             \


with_opencv {
    SUBDIRS +=       \
#        distortion_corrector \
#        matcher_basic \
#        matcher_full \
#        camera_calibration \
#        chessboard_detector \
        calibration \
#        calibration_job_generate \
#        calibration_job_detect \
#        calibration_job_estimate_distortion \
#        calibration_job_apply_undistortion \
#        calibration_job_calibrate \
        \
        geometry_verificator \
#        photostation_placer \
        photostation_placer_features \

}

grab.file                           = grab/grab.pro
grab_N_captures.file                = grab_N_captures/grab_N_captures.pro
serialize1.file                     = serialize1/serialize1.pro
adoptcolor.file                     = adoptcolor/adoptcolor.pro
avigrab.file                        = avigrab/avigrab.pro
decodebayer.file                    = decodebayer/decodebayer.pro
genvectorui.file                    = genvectorui/genvectorui.pro
aLowCodec.file                      = aLowCodec/aLowCodec.pro

cr2reader.file                      = cr2reader/cr2reader.pro
cr2reader.depends                   -= utils
debayer.file                        = debayer/debayer.pro
debayer.depends                     -= utils

new_board_detector.file             = new_board_detector/new_board_detector.pro

distortion_corrector.file           = distortion_corrector/distortion_corrector.pro

matcher_basic.file                  = matcher_basic/feature2d_basic.pro
matcher_full.file                   = matcher_full/matcher_full.pro

camera_calibration.file             = camera_calibration/camera_calibration.pro
chessboard_detector.file            = chessboard_detector/chessboard_detector.pro

calibration.file                    = calibration/calibration.pro

calibration_job_generate            = calibration_job_generate/calibration_job_generate.pro
calibration_job_detect              = calibration_job_detect/calibration_job_detect.pro
calibration_job_estimate_distortion = calibration_job_estimate_distortion/calibration_job_estimate_distortion.pro
calibration_job_apply_undistortion  = calibration_job_apply_undistortion/calibration_job_apply_undistortion.pro
calibration_job_calibrate           = calibration_job_calibrate/calibration_job_calibrate.pro

geometry_verificator                = geometry_verificator/geometry_verificator.pro

#photostation_placer.file            = photostation_placer/photostation_placer.pro
photostation_placer_features.file   = photostation_placer_features/photostation_placer_features.pro
