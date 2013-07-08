include(../../config.pri)

TEMPLATE = subdirs

with_opencv:!macx {
    #!contains(SUBDIRS, ocv-homo):      # such operator doesn't work!
    SUBDIRS *= ocv-homo                 # adds project only if it doesn't exist
    SUBDIRS *= openCV
}

SUBDIRS += \
    convolve \
    face_recognition1 \
    face_recognition \
    integral \
    klt_cycle \
    midmap_pyramid \
    rectificator \
    rectificator1 \
#   sig_epsilon \
#   signature_compare \
    train_vj \
    matrix \
    buffer \
    fileformats \
    arithmetics \
    ssewrappers \
    fastkernel \
    assignment \
#   cppunit_test \
    cornerdetector \
    cholesky \
    kalman \
    automotive \
    sphericdist \
#   snooker \
    color \
    geometry \
    moments \
    draw \
    affine \
    derivative \
    vector \
    tbb_wrapper \
    homography \
    ransac \
    levenberg \
    rgb24buffer \
    morphologic \
    gradient \
    serializer \
    histogram \
    deform \
    polynomDistortion \
    fastkernel_profile \
    logger \
#    filter_blocks \
    cameramodel \
#    stateMachineTest \
    triangulator \
	cloud \
	distortion \
