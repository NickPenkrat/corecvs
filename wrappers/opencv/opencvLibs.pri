with_opencv {
    OPENCV_PATH = $$(OPENCV_PATH)
    win32 {
        !isEmpty(OPENCV_PATH) {
            exists($$OPENCV_PATH/include/opencv2/core/version.hpp) {
                INCLUDEPATH += $$OPENCV_PATH/include
            }
            DEFINES += WITH_OPENCV          # we assume that we'll find OpenCV if it's installed; otherwise please adapt this file.

            OPENCV_INC_INSTALLED    = $$OPENCV_PATH/build/include
            OPENCV_INC_NOTINSTALLED = $$OPENCV_PATH/include \
                                      $$OPENCV_PATH/modules/calib3d/include \
                                      $$OPENCV_PATH/modules/video/include \
                                      $$OPENCV_PATH/modules/core/include \
                                      $$OPENCV_PATH/modules/highgui/include \
                                      $$OPENCV_PATH/modules/features2d/include \
                                      $$OPENCV_PATH/modules/flann/include \
                                      $$OPENCV_PATH/modules/legacy/include \
                                      $$OPENCV_PATH/modules/nonfree/include \
                                      $$OPENCV_PATH/modules/imgproc/include \
                                      $$OPENCV_PATH/modules/objdetect/include \
                                      $$OPENCV_PATH/modules/ml/include \

            OPENCV_249_LIBS         = -lopencv_calib3d249    -lopencv_video249  -lopencv_core249   -lopencv_highgui249 \
                                      -lopencv_features2d249 -lopencv_flann249  -lopencv_legacy249 -lopencv_nonfree249 \
                                      -lopencv_imgproc249    -lopencv_objdetect249

            exists($$OPENCV_PATH/build/x64/vc10/bin/opencv_core249.dll): win32-msvc* {  # installed OpenCV v.2.4.9 with msvc10 support without GPU
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc10/bin>)
                INCLUDEPATH += $$OPENCV_INC_INSTALLED
                LIBS        += -L$$OPENCV_PATH/build/x64/vc10/lib/ $$OPENCV_249_LIBS
                    
            } else:exists($$OPENCV_PATH/build/bin/Release/opencv_core249.dll): win32-msvc* {   # git's OpenCV tag=2.4.9 built by MSVC with GPU
                !build_pass:message(Using <$$OPENCV_PATH/build/bin/Release>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS        += -L$$OPENCV_PATH/build/lib/Release/  $$OPENCV_249_LIBS
                
            } else:exists($$OPENCV_PATH/build/x64/mingw/bin/libopencv_core246.dll): !win32-msvc* {   # installed OpenCV v.2.4.6 with MINGW support
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/mingw/bin>)
                INCLUDEPATH += $$OPENCV_INC_INSTALLED
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
            } else:exists($$OPENCV_PATH/build.mg/bin/libopencv_core249.dll): !win32-msvc* {   # git's OpenCV tag=2.4.9 built by MINGW without GPU
                !build_pass:message(Using <$$OPENCV_PATH/build.mg/bin>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS += \
                    -L$$OPENCV_PATH/build.mg/bin/ \
                    -llibopencv_calib3d249 \
                    -llibopencv_video249 \
                    -llibopencv_core249 \
                    -llibopencv_highgui249 \
                    -llibopencv_features2d249 \
                    -llibopencv_flann249 \
                    -llibopencv_legacy249 \
                    -llibopencv_nonfree249 \
                    -llibopencv_imgproc249 \
                    -llibopencv_objdetect249
            } else {
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
