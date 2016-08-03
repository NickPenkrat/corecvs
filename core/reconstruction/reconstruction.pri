HEADERS += \
    reconstruction/reconstructionStructs.h \
    reconstruction/pnpSolver.h \
    reconstruction/relativeNonCentralP6PSolver.h \
    reconstruction/relativeNonCentralO3PSolver.h \
    reconstruction/relativeNonCentralRansacSolver.h \
    reconstruction/photostationPlacer.h \
    reconstruction/absoluteNonCentralRansacSolver.h \
    reconstruction/reconstructionFixtureScene.h \
    reconstruction/essentialFeatureFilter.h \
    reconstruction/sceneGenerator.h \
#    reconstruction/reconstructionInitializer.h \
    reconstruction/sceneAligner.h \
    reconstruction/reconstructionFunctor.h \
    reconstruction/iterativeReconstructionSettings.h


SOURCES += \
    reconstruction/pnpSolver.cpp \
    reconstruction/pnpGbActionMatrix.cpp \
    reconstruction/p34pGbActionMatrix.cpp \
    reconstruction/relativeNonCentralP6PSolver.cpp \
    reconstruction/multiplyTemplateNCP6P.cpp \
    reconstruction/extractLastBasisNCP6P.cpp \
    reconstruction/setupActionMatrixNCP6P.cpp \
    reconstruction/relativeNonCentralRansacSolver.cpp \
    reconstruction/photostationPlacer.cpp \
    reconstruction/absoluteNonCentralRansacSolver.cpp \
    reconstruction/reconstructionFixtureScene.cpp \
    reconstruction/essentialFeatureFilter.cpp \
    reconstruction/sceneGenerator.cpp \
#    reconstruction/reconstructionInitializer.cpp \
    reconstruction/sceneAligner.cpp \
    reconstruction/reconstructionFunctor.cpp \
    reconstruction/relativeNonCentralO3PSolver.cpp \


SOURCES_NOOPTIMIZE = \
        reconstruction/setupTemplateNCO3P.cpp \
        reconstruction/setupTemplate1NCP6P.cpp \

with_fastbuild:!win32 {

    nooptimize.name = nooptimize
    nooptimize.input = SOURCES_NOOPTIMIZE
    nooptimize.dependency_type = TYPE_C
    nooptimize.variable_out = OBJECTS
    nooptimize.output = ${QMAKE_VAR_OBJECTS_DIR}${QMAKE_FILE_IN_BASE}$${first(QMAKE_EXT_OBJ)}
    nooptimize.commands = $${QMAKE_CXX} $(CXXFLAGS) -O0 $(INCPATH) -c ${QMAKE_FILE_IN} -o ${QMAKE_FILE_OUT} # Note the -O0
    QMAKE_EXTRA_COMPILERS += nooptimize


} else {

    SOURCES += $$SOURCES_NOOPTIMIZE
}
