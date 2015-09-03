include(../../../config.pri)

include($${OPEN_ROOT_DIRECTORY}/core/core.pri)
include($${OPEN_ROOT_DIRECTORY}/wrappers/gtest/gtest.pri)

TEMPLATE = app
TARGET   = test_core
CONFIG  += console
CONFIG  -= app_bundle

SOURCES += main.cpp \
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
#cppunit_test/cppunit_test.cpp \
deform/main_test_deform.cpp \
derivative/main_test_derivative.cpp \
distortion/main_test_distortion.cpp \
draw/main_test_draw.cpp \
eigen/main_test_eigen.cpp \
face_recognition/main_test_face_recognition.cpp \
face_recognition1/main_test_face_recognition1.cpp \
fastkernel/main_test_fastkernel.cpp \
fastkernel_profile/main_test_fastkernel_profile.cpp \
fileformats/main_test_fileformats.cpp \
filter_blocks/main_test_filter_blocks.cpp \


