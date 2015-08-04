with_opencv {
    OPENCV_PATH = $$(OPENCV_PATH)
    win32 {
        !isEmpty(OPENCV_PATH) {
            exists($$OPENCV_PATH/include/opencv2/core/version.hpp) {
                INCLUDEPATH += $$OPENCV_PATH/include                            # when we made "make install" there
            }
            DEFINES += WITH_OPENCV          # we assume that we'll find OpenCV if it's installed; otherwise please adapt this file.

            #OPENCV_INC_NOTINSTALLED = $$OPENCV_PATH/build/include    # strange path, usually empty, but distributive has all includes there?...

            # the following paths are taking includes directly from sources
            #
            exists($$OPENCV_PATH/sources/modules/core/include/opencv2/core/core.hpp) { 	# since v.2.4.11 sources at folder
                OPENCV_INC_NOTINSTALLED += \
                    $$OPENCV_PATH/sources/modules/core/include \
                    $$OPENCV_PATH/sources/modules/calib3d/include \
                    $$OPENCV_PATH/sources/modules/video/include \
                    $$OPENCV_PATH/sources/modules/highgui/include \
                    $$OPENCV_PATH/sources/modules/imgproc/include \
                    $$OPENCV_PATH/sources/modules/features2d/include \
                    $$OPENCV_PATH/sources/modules/flann/include \
                    $$OPENCV_PATH/sources/modules/objdetect/include \
                    $$OPENCV_PATH/sources/modules/legacy/include \
                    $$OPENCV_PATH/sources/modules/nonfree/include \
                    $$OPENCV_PATH/sources/modules/ml/include \

            } else:exists($$OPENCV_PATH/modules/core/include/opencv2/core/core.hpp) {        # 2.4.9 has sources at the root folder
                OPENCV_INC_NOTINSTALLED += \
                    $$OPENCV_PATH/modules/core/include \
                    $$OPENCV_PATH/modules/calib3d/include \
                    $$OPENCV_PATH/modules/video/include \
                    $$OPENCV_PATH/modules/highgui/include \
                    $$OPENCV_PATH/modules/imgproc/include \
                    $$OPENCV_PATH/modules/features2d/include \
                    $$OPENCV_PATH/modules/flann/include \
                    $$OPENCV_PATH/modules/objdetect/include \
                    $$OPENCV_PATH/modules/legacy/include \
                    $$OPENCV_PATH/modules/nonfree/include \
                    $$OPENCV_PATH/modules/ml/include \

            }

            OPENCV_249_LIBS_R       = -lopencv_calib3d249     -lopencv_video249   -lopencv_core249    -lopencv_highgui249 \
                                      -lopencv_features2d249  -lopencv_flann249   -lopencv_legacy249  -lopencv_nonfree249 \
                                      -lopencv_imgproc249     -lopencv_objdetect249

            OPENCV_249_LIBS_D       = -lopencv_calib3d249d    -lopencv_video249d  -lopencv_core249d   -lopencv_highgui249d \
                                      -lopencv_features2d249d -lopencv_flann249d  -lopencv_legacy249d -lopencv_nonfree249d \
                                      -lopencv_imgproc249d    -lopencv_objdetect249d
            CONFIG(debug, debug|release) {
                OPENCV_249_LIBS = $$OPENCV_249_LIBS_D
                OPENCV_249_LIBS_ADD_OWN_BUILT = -L$$OPENCV_PATH/build/lib/Debug/    $$OPENCV_249_LIBS
            }
            CONFIG(release, debug|release) {
                OPENCV_249_LIBS = $$OPENCV_249_LIBS_R
                OPENCV_249_LIBS_ADD_OWN_BUILT = -L$$OPENCV_PATH/build/lib/Release/  $$OPENCV_249_LIBS
            }

            exists($$OPENCV_PATH/build/bin/Release/opencv_core249.dll): win32-msvc* {   # git's OpenCV tag=2.4.9 built by any own MSVC
                !build_pass:message(Using <$$OPENCV_PATH/build/bin/Release|Debug>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS        += $$OPENCV_249_LIBS_ADD_OWN_BUILT

            } else:exists($$OPENCV_PATH/build/x64/vc10/bin/opencv_core249.dll): win32-msvc2010 {  # installed OpenCV v.2.4.9 with msvc10 support without GPU
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc10/bin>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS        += -L$$OPENCV_PATH/build/x64/vc10/lib/ $$OPENCV_249_LIBS

            } else:exists($$OPENCV_PATH/build/x64/vc12/bin/opencv_core249.dll): win32-msvc2012 {  # installed OpenCV v.2.4.9 with msvc12 support without GPU
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc12/bin>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS        += -L$$OPENCV_PATH/build/x64/vc12/lib/ $$OPENCV_249_LIBS

            } else:exists($$OPENCV_PATH/build.mg/bin/libopencv_core249.dll): !win32-msvc* {   # git's OpenCV tag=2.4.9 built by MINGW without GPU
                !build_pass:message(Using <$$OPENCV_PATH/build.mg/bin>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS += \
                    -L$$OPENCV_PATH/build.mg/bin/ \
                    -llibopencv_calib3d249    -llibopencv_video249 -llibopencv_core249   -llibopencv_highgui249 \
                    -llibopencv_features2d249 -llibopencv_flann249 -llibopencv_legacy249 -llibopencv_nonfree249 \
                    -llibopencv_imgproc249    -llibopencv_objdetect249

            } else:exists($$OPENCV_PATH/build/x64/vc12/bin/opencv_core2411.dll) {
                !build_pass:message(Using <$$OPENCV_PATH/build/x64/vc12/bin>)
                INCLUDEPATH += $$OPENCV_INC_NOTINSTALLED
                LIBS += \
                    -L$$OPENCV_PATH/build/x64/vc12/lib/ \
                    -lopencv_calib3d2411    -lopencv_video2411  -lopencv_core2411   -lopencv_highgui2411     \
                    -lopencv_features2d2411 -lopencv_flann2411  -lopencv_legacy2411 -llibopencv_nonfree2411  \
                    -lopencv_imgproc2411    -lopencv_objdetect2411
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
            LIBS += -lopencv_calib3d   -lopencv_core    -lopencv_features2d -lopencv_flann \
                    -lopencv_highgui   -lopencv_imgproc -lopencv_legacy     -lopencv_ml \
                    -lopencv_objdetect -lopencv_ocl     -lopencv_video

            LIBS += -lopencv_nonfree
            LIBS += -lopencv_contrib -lopencv_gpu -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_videostab -lopencv_gpu

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
