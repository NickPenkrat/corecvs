# try use global config 
exists(../../../config.pri) {
    ROOT_DIR=../../..
    include($$ROOT_DIR/config.pri)
} else {
    message(Using local config)
    ROOT_DIR=..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

TEMPLATE = app
TARGET   = test_core
CONFIG  += console
CONFIG  -= app_bundle

#message(OPEN_ROOT_DIRECTORY = $${OPEN_ROOT_DIRECTORY})
#include($${OPEN_ROOT_DIRECTORY}/core/core.pri)
#include($${OPEN_ROOT_DIRECTORY}/wrappers/gtest/gtest.pri)

include(../core/core.pri)
include(../wrappers/gtest/gtest.pri)

DESTDIR = $$ROOT_DIR/bin

TARGET_ORIG = $$TARGET                              # store original target name for proper detection of the obj.dir
TARGET      = $$join(TARGET,,,$$BUILD_CFG_SFX)      # add 'd' at the end for debug versions

OBJECTS_DIR = $$ROOT_DIR/.obj/$$TARGET_ORIG$$BUILD_CFG_NAME

MOC_DIR  = $$OBJECTS_DIR                            # we have to set it to omit creating dummy dirs: debug,release
#UI_DIR  = $$OBJECTS_DIR
#RCC_DIR = $$OBJECTS_DIR

SOURCES += \
    main.cpp \
    \
    affine/main_test_affine.cpp \
    arithmetics/main_test_arithmetics.cpp \
    assignment/main_test_assignment.cpp \
    automotive/main_test_automotive.cpp \
    buffer/main_test_buffer.cpp \
    cameramodel/main_test_cameramodel.cpp \
    cholesky/main_test_cholesky.cpp \
    cloud/main_test_cloud.cpp \
    color/main_test_color.cpp \
    convolve/main_test_convolve.cpp \
    cornerdetector/main_test_cornerdetector.cpp \
    deform/main_test_deform.cpp \
    derivative/main_test_derivative.cpp \
    draw/main_test_draw.cpp \
    eigen/main_test_eigen.cpp \
    #face_recognition/main_test_face_recognition.cpp \
    #face_recognition1/main_test_face_recognition1.cpp \
    #fileformats/main_test_fileformats.cpp \ #todo: RAW Image load failed
    #filter_blocks/main_test_filter_blocks.cpp \ #todo: can't build
    #gaussianSolution/main.cpp \
    geometry/main_test_geometry.cpp \
    gradient/main_test_gradient.cpp \
    histogram/main_test_histogram.cpp \
    homography/main_test_homography.cpp \
    integral/main_test_integral.cpp \
    kalman/main_test_kalman.cpp \
    klt_cycle/main_test_klt_cycle.cpp \
    levenberg/main_test_levenberg.cpp \
    #logger/main_test_logger.cpp \
    matrix/main_test_matrix.cpp \ # todo: windows: Assert at matrix\main_test_matrix.cpp:381 - Internal problem with double and stdout
    #midmap_pyramid/main_test_midmap_pyramid.cpp #todo: main + file read
    moments/main_test_moments.cpp \
    morphologic/main_test_morphologic.cpp \
    #ocv-homo/main_test_ocv-homo.cpp \ # todo: main + opencv deps
    #openCV/main_test_openCV.cpp \ # todo: main + opencv deps
    #polynomDistortion/main.cpp \ # todo: can't build
    #ransac/main_test_ransac.cpp \ #todo: the best test ever seen
    rectificator/main_test_rectificator.cpp \
    rectificator1/main_test_rectificator1.cpp \
    rgb24buffer/main_test_rgb24buffer.cpp \
    serializer/main_test_serializer.cpp \
    #sig_epsilon/main_test_sig_epsilon.cpp \ #todo: missing include
    #signature_compare/main_test_signature_compare.cpp \ #todo: file read
    similarity/main_test_similarity.cpp \
    #snooker/ #todo: move code
    sphericdist/main_test_sphericdist.cpp \
    ssewrappers/main_test_ssewrappers.cpp \ #todo: can't build
    #stateMachineTest/ #todo: qt4 deps
    #tbb_wrapper/main_test_tbb_wrapper.cpp #todo: tbb deps
    #train_vj/main_train_vj.cpp #todo: file read
    triangulator/main_test_triangulator.cpp \
    vector/main_test_vector.cpp \
    \    
    yuv/main_test_yuv.cpp \
    cameracalibration/main_test_camera_structs.cpp


# Never to be fixed
OTHER_FILES += \
    cppunit_test/cppunit_test.cpp \


# Things need to be fixed
OTHER_FILES += \
    fastkernel/main_test_fastkernel.cpp \
    fastkernel_profile/main_test_fastkernel_profile.cpp \
    fastkernel_double/main_test_fastkernel_double.cpp \
    distortion/main_test_distortion.cpp \

