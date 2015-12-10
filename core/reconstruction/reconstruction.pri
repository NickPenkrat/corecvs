HEADERS += \
	reconstruction/reconstructionStructs.h \
	reconstruction/multiPhotostationScene.h \
	reconstruction/pnpSolver.h \
	reconstruction/relativeNonCentralP6PSolver.h \
	reconstruction/relativeNonCentralRansacSolver.h \
	reconstruction/photostationPlacer.h


SOURCES += \
    reconstruction/multiPhotostationScene.cpp \
    reconstruction/pnpSolver.cpp \
    reconstruction/pnpGbActionMatrix.cpp \
    reconstruction/p34pGbActionMatrix.cpp \
    reconstruction/relativeNonCentralP6PSolver.cpp \
    reconstruction/setupTemplate1NCP6P.cpp \
    reconstruction/multiplyTemplateNCP6P.cpp \
    reconstruction/extractLastBasisNCP6P.cpp \
    reconstruction/setupActionMatrixNCP6P.cpp \
    reconstruction/relativeNonCentralRansacSolver.cpp \
    reconstruction/photostationPlacer.cpp



CONFIG += c++11
