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
UI_DIR   = $$OBJECTS_DIR
RCC_DIR  = $$OBJECTS_DIR

OTHER_FILES += \
    gentest.sh

# Never to be fixed
OTHER_FILES += \
    cppunit_test/cppunit_test.cpp \
    snooker/main_test_snooker.cpp \                     # TODO: not the test code!

# Things need to be fixed
OTHER_FILES += \
   #ocv-homo/main_test_ocv-homo.cpp \                   # TODO: main + opencv deps
   #openCV/main_test_openCV.cpp \                       # TODO: main + opencv deps


SOURCES += \
    main.cpp \
    \
    affine/main_test_affine.cpp \
    aLowCodec/main_test_aLowCodec.cpp \
    arithmetics/main_test_arithmetics.cpp \
    assignment/main_test_assignment.cpp \
    automotive/main_test_automotive.cpp \
    buffer/main_test_buffer.cpp \
    cameramodel/main_test_cameramodel.cpp \
    cholesky/main_test_cholesky.cpp \
    cloud/main_test_cloud.cpp \
    color/main_test_color.cpp \
    commandline/main_test_commandline.cpp \
    convolve/main_test_convolve.cpp \
    cornerdetector/main_test_cornerdetector.cpp \
    deform/main_test_deform.cpp \
    derivative/main_test_derivative.cpp \
    distortion/main_test_distortion.cpp \                 # TODO: need to be fixed soon
    draw/main_test_draw.cpp \
    eigen/main_test_eigen.cpp \
   #face_recognition/main_test_face_recognition.cpp \     # TODO: absent input data!
   #face_recognition1/main_test_face_recognition1.cpp \   # TODO: absent input data!
    fastkernel/main_test_fastkernel.cpp \
    fastkernel_double/main_test_fastkernel_double.cpp \   # TODO: move to perf-tests ?
    fastkernel_profile/main_test_fastkernel_profile.cpp \ # TODO: move to perf-tests ?
    fileformats/main_test_fileformats.cpp \
   #filter_blocks/main_test_filter_blocks.cpp \           # TODO: can't build
    gaussianSolution/main_gaussianSolution.cpp \          # TODO: check it...
    geometry/main_test_geometry.cpp \
    gradient/main_test_gradient.cpp \
    histogram/main_test_histogram.cpp \
    homography/main_test_homography.cpp \
    integral/main_test_integral.cpp \
    kalman/main_test_kalman.cpp \
    klt_cycle/main_test_klt_cycle.cpp \
    levenberg/main_test_levenberg.cpp \
    logger/main_test_logger.cpp \
    matrix/main_test_matrix.cpp \                       # TODO: Windows: assert at matrix\main_test_matrix.cpp:385 - Internal problem with double and stdout
    midmap_pyramid/main_test_midmap_pyramid.cpp \
    moments/main_test_moments.cpp \
    morphologic/main_test_morphologic.cpp \
    polynomDistortion/main_polynomDistortion.cpp \      # TODO: can't build - check...?
    ransac/main_test_ransac.cpp \
    readers/main_test_readers.cpp \
    rectificator/main_test_rectificator.cpp \
    rectificator1/main_test_rectificator1.cpp \
    rgb24buffer/main_test_rgb24buffer.cpp \
    serializer/main_test_serializer.cpp \
    similarity/main_test_similarity.cpp \
    sphericdist/main_test_sphericdist.cpp \
    ssewrappers/main_test_ssewrappers.cpp \
    tbb_wrapper/main_test_tbb_wrapper.cpp \
   #train_vj/main_train_vj.cpp                          # TODO: read missed file via param
    triangulator/main_test_triangulator.cpp \
    vector/main_test_vector.cpp \
    yuv/main_test_yuv.cpp \
    conic/main_test_conic.cpp \
    calstructs/main_test_calstructs.cpp \

