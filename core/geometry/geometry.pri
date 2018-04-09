HEADERS += \
    geometry/ellipticalApproximation.h \
    geometry/axisAlignedBox.h \
    geometry/rectangle.h \
    geometry/line.h \
    geometry/triangulation.h \
    geometry/polygons.h \
    geometry/mesh3d.h \
    geometry/vptree.h \
    geometry/convexPolyhedron.h \
    geometry/renderer/simpleRenderer.h \
    geometry/renderer/geometryIterator.h \
    geometry/conic.h \
    geometry/polygonPointIterator.h \
    geometry/raytrace/raytraceRenderer.h \
    geometry/raytrace/raytraceObjects.h \
    geometry/raytrace/perlinNoise.h \
    geometry/raytrace/sdfRenderable.h \
    geometry/raytrace/materialExamples.h \
    geometry/mesh3DDecorated.h \
    geometry/kdtree.h \
    geometry/raytrace/bspTree.h \
    geometry/twoViewOptimalTriangulation.h \
    geometry/ellipse.h \
    geometry/ellipseFit.h \
    geometry/plane3dFit.h \
    $$PWD/meshClicker.h \
    $$PWD/raytrace/sdfRenderableObjects.h \
    $$PWD/convexHull.h \
    $$PWD/convexQuickHull.h \
    $$PWD/renderer/attributedTriangleSpanIterator.h


SOURCES += \ 
    geometry/ellipticalApproximation.cpp \
    geometry/axisAlignedBox.cpp \
    geometry/rectangle.cpp \
    geometry/triangulation.cpp \
    geometry/polygons.cpp \
    geometry/mesh3d.cpp \
    geometry/convexPolyhedron.cpp \
    geometry/renderer/simpleRenderer.cpp \
    geometry/conic.cpp \
    geometry/polygonPointIterator.cpp \
    geometry/raytrace/raytraceRenderer.cpp \
    geometry/raytrace/raytraceObjects.cpp \
    geometry/raytrace/perlinNoise.cpp \
    geometry/raytrace/sdfRenderable.cpp \
    geometry/raytrace/materialExamples.cpp \
    geometry/mesh3DDecorated.cpp \
    geometry/twoViewOptimalTriangulation.cpp \
    geometry/ellipse.cpp \
    geometry/ellipseFit.cpp \
    geometry/plane3dFit.cpp \
    $$PWD/meshClicker.cpp \
    $$PWD/raytrace/sdfRenderableObjects.cpp \
    $$PWD/convexHull.cpp \
    $$PWD/convexQuickHull.cpp \
    $$PWD/renderer/attributedTriangleSpanIterator.cpp

