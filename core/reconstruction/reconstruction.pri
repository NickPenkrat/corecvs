HEADERS += \
	reconstruction/reconstructionStructs.h \
	reconstruction/multiPhotostationScene.h \
	reconstruction/pnpSolver.h \
	reconstruction/relativeNonCentralP6PSolver.h \
	reconstruction/relativeNonCentralRansacSolver.h \
	reconstruction/photostationPlacer.h \
	reconstruction/absoluteNonCentralRansacSolver.h \
	reconstruction/reconstructionFixtureScene.h \
	reconstruction/essentialFeatureFilter.h \
	reconstruction/sceneGenerator.h \
	reconstruction/reconstructionInitializer.h


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
    reconstruction/photostationPlacer.cpp \
    reconstruction/absoluteNonCentralRansacSolver.cpp \
    reconstruction/reconstructionFixtureScene.cpp \
    reconstruction/essentialFeatureFilter.cpp \
    reconstruction/sceneGenerator.cpp \
    reconstruction/reconstructionInitializer.cpp
