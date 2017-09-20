HEADERS += \
    rectification/correspondenceList.h \
    rectification/essentialMatrix.h \
    rectification/essentialEstimator.h \
    rectification/iterativeEstimator.h \
    rectification/ransacEstimator.h \
    rectification/stereoAligner.h \ 
    rectification/triangulator.h \
    rectification/ransac.h \
    rectification/multicameraEstimator.h \
    rectification/multicameraTriangulator.h \
    $$PWD/sceneStereoAlignerBlock.h


SOURCES += \
    rectification/essentialMatrix.cpp \
    rectification/essentialEstimator.cpp \
    rectification/iterativeEstimator.cpp \
    rectification/ransacEstimator.cpp \
    rectification/correspondenceList.cpp \
    rectification/stereoAligner.cpp \
    rectification/triangulator.cpp \
    rectification/multicameraEstimator.cpp \
    rectification/multicameraTriangulator.cpp \
    $$PWD/sceneStereoAlignerBlock.cpp

SOURCES += \
    xml/generated/essentialDerivative.cpp \
    xml/generated/essentialDerivative1.cpp \


