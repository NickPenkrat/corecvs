with_opencv {
    OPENCV_PATH = $$(OPENCV_PATH)
    win32 {
        !isEmpty(OPENCV_PATH) {
            exists($$OPENCV_PATH/include/opencv2/core/version.hpp) {
                INCLUDEPATH += $$OPENCV_PATH/include
            }
            DEFINES += WITH_OPENCV          # we assume that we'll find OpenCV if it's installed; otherwise please adapt this file.

            exists($$OPENCV_PATH/build/x64/mingw/bin/libopencv_core242.dll): !win32-msvc* {
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/mingw/bin>)
                INCLUDEPATH += \
                    $$OPENCV_PATH/build/include
                LIBS += \
                    -L$$OPENCV_PATH/build/x64/mingw/lib/ \
                    -llibopencv_calib3d242 \
                    -llibopencv_video242 \
                    -llibopencv_core242 \
                    -llibopencv_highgui242 \
                    -llibopencv_features2d242 \
                    -llibopencv_flann242 \
                    -llibopencv_legacy242 \
                    -llibopencv_nonfree242 \
                    -llibopencv_imgproc242 \
                    -llibopencv_objdetect242
            } else:exists($$OPENCV_PATH/build/x64/mingw/bin/libopencv_core246.dll): !win32-msvc* {
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/mingw/bin>)
                INCLUDEPATH += \
                    $$OPENCV_PATH/build/include
                LIBS += \
                    -L$$OPENCV_PATH/build/x64/mingw/lib/ \
                    -llibopencv_calib3d246 \
                    -llibopencv_video246 \
                    -llibopencv_core246 \
                    -llibopencv_highgui246 \
                    -llibopencv_features2d246 \
                    -llibopencv_flann246 \
                    -llibopencv_legacy246 \
                    -llibopencv_nonfree246 \
                    -llibopencv_imgproc246 \
                    -llibopencv_objdetect246
            } else:exists($$OPENCV_PATH/build/x64/vc10/bin/opencv_core246.dll): win32-msvc* {
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc10/bin>)
                INCLUDEPATH += \
                    $$OPENCV_PATH/build/include
                LIBS += \
                    -L$$OPENCV_PATH/build/x64/vc10/lib/ \
                    -lopencv_calib3d246 \
                    -lopencv_video246 \
                    -lopencv_core246 \
                    -lopencv_highgui246 \
                    -lopencv_features2d246 \
                    -lopencv_flann246 \
                    -lopencv_legacy246 \
                    -lopencv_nonfree246 \
                    -lopencv_imgproc246 \
                    -lopencv_objdetect246
            } else:exists($$OPENCV_PATH/build/x64/vc10/bin/opencv_core247.dll): win32-msvc* {
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc10/bin>)
                INCLUDEPATH += \
                    $$OPENCV_PATH/build/include
                LIBS += \
                    -L$$OPENCV_PATH/build/x64/vc10/lib/ \
                    -lopencv_calib3d247 \
                    -lopencv_video247 \
                    -lopencv_core247 \
                    -lopencv_highgui247 \
                    -lopencv_features2d247 \
                    -lopencv_flann247 \
                    -lopencv_legacy247 \
                    -lopencv_nonfree247 \
                    -lopencv_imgproc247 \
                    -lopencv_objdetect247
            } else:exists($$OPENCV_PATH/bin/libopencv_core249.dll) {
                !build_pass:message(Using <$$OPENCV_PATH/bin>)
                INCLUDEPATH += $$OPENCV_PATH/include \
                    $$OPENCV_PATH/modules/core/include \
                    $$OPENCV_PATH/modules/calib3d/include \
                    $$OPENCV_PATH/modules/video/include \
                    $$OPENCV_PATH/modules/highgui/include \
                    $$OPENCV_PATH/modules/imgproc/include \
                    $$OPENCV_PATH/modules/features2d/include \
                    $$OPENCV_PATH/modules/flann/include \
                    $$OPENCV_PATH/modules/objdetect/include \
                    $$OPENCV_PATH/modules/legacy/include
                LIBS += \
                    -L$$OPENCV_PATH/bin/ \
                    -llibopencv_calib3d249 \
                    -llibopencv_video249 \
                    -llibopencv_core249 \
                    -llibopencv_highgui249 \
                    -llibopencv_features2d249 \
                    -llibopencv_legacy249 \
                    -llibopencv_objdetect249 \
                    -llibopencv_imgproc249
            } else:exists($$OPENCV_PATH/build/x64/vc12/bin/opencv_core2411.dll) {
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc12/bin>)
                INCLUDEPATH += $$OPENCV_PATH/build/include \
                    $$OPENCV_PATH/sources/modules/core/include \
                    $$OPENCV_PATH/sources/modules/calib3d/include \
                    $$OPENCV_PATH/sources/modules/video/include \
                    $$OPENCV_PATH/sources/modules/highgui/include \
                    $$OPENCV_PATH/sources/modules/imgproc/include \
                    $$OPENCV_PATH/sources/modules/features2d/include \
                    $$OPENCV_PATH/sources/modules/flann/include \
                    $$OPENCV_PATH/sources/modules/objdetect/include \
                    $$OPENCV_PATH/sources/modules/legacy/include
                LIBS += \
                    -L$$OPENCV_PATH/build/x64/vc12/lib/ \
                    -lopencv_calib3d2411 \
                    -lopencv_video2411 \
                    -lopencv_core2411 \
                    -lopencv_highgui2411 \
                    -lopencv_features2d2411 \
                    -lopencv_legacy2411 \
                    -lopencv_objdetect2411 \
                    -lopencv_imgproc2411
            } else {
                message(Using <$$OPENCV_PATH>)
                message(Unsupported OpenCV version - please adapt the opencvLibs.pri for other versions)
            }
        } else {
            !build_pass:message(OpenCV not found)
        }

    } else:macx {
        !build_pass:message(Compiling with OpenCV from $$OPENCV_PATH)

        LIBS += -L$$OPENCV_PATH/lib/
        LIBS += -lopencv_highgui -lopencv_video -lopencv_core

        INCLUDEPATH += $$OPENCV_PATH/include
        DEPENDPATH  += $$OPENCV_PATH/include
        DEFINES     += WITH_OPENCV
    } else {
        isEmpty(OPENCV_PATH) {
            !build_pass:message(Compiling with system OpenCV)
#           LIBS += -lcv -lhighgui -lcxcore
            LIBS += -lopencv_highgui -lopencv_video -lopencv_core -lopencv_flann -lopencv_imgproc -lopencv_calib3d -lopencv_features2d -lopencv_objdetect # for opencv 2.3+
            LIBS += -lopencv_nonfree
        } else {
            !build_pass:message(Compiling with OpenCV from $$OPENCV_PATH)
            LIBS += -L$$OPENCV_PATH/lib/

            LIBS += -lopencv_legacy
            LIBS += -lopencv_highgui
            LIBS += -lopencv_contrib
#           LIBS += -lopencv_ts
            LIBS += -lopencv_objdetect
            LIBS += -lopencv_calib3d
            LIBS += -lopencv_ml
            LIBS += -lopencv_flann
#           LIBS += -lopencv_videostab
            LIBS += -lopencv_nonfree
            LIBS += -lopencv_features2d
#           LIBS += -lopencv_photo
            LIBS += -lopencv_video
            LIBS += -lopencv_gpu
            LIBS += -lopencv_core
            LIBS += -lopencv_imgproc
#           LIBS += -lopencv_stitching

            INCLUDEPATH += $$OPENCV_PATH/include
            DEPENDPATH  += $$OPENCV_PATH/include
        }
        DEFINES += WITH_OPENCV
    }
}
