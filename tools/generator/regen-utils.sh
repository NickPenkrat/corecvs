#!/bin/bash

source ./helper-regen.sh

echo -n "Building generator... "
qmake && make
echo "done"

echo -n "Running generator on xml/vimouse.xml..."
./generatorConcept xml/graphPlot.xml
echo "done"

echo -n "Running generator on xml/vimouse.xml..."
./generatorConcept xml/draw3d.xml
echo "done"

echo "Makeing a copy of base classes"
./copy-base.sh

dest="../../utils/widgets/generated"
copy_if_different Generated/graphPlotParametersControlWidget.cpp  $dest/
copy_if_different Generated/graphPlotParameters.cpp               $dest/
copy_if_different Generated/graphPlotParameters.h                 $dest/
copy_if_different Generated/graphStyle.h                          $dest/
echo "graphPlot copied"

dest="../../utils/3d/generated"
copy_if_different Generated/draw3dParametersControlWidget.ui   $dest/
copy_if_different Generated/draw3dParametersControlWidget.cpp  $dest/
copy_if_different Generated/draw3dParameters.cpp               $dest/
copy_if_different Generated/draw3dParameters.h                 $dest/
copy_if_different Generated/draw3dStyle.h                      $dest/
copy_if_different Generated/draw3dTextureGen.h                 $dest/

copy_if_different Generated/draw3dCameraParametersControlWidget.ui   $dest/
copy_if_different Generated/draw3dCameraParametersControlWidget.cpp  $dest/
copy_if_different Generated/draw3dCameraParameters.cpp               $dest/
copy_if_different Generated/draw3dCameraParameters.h                 $dest/


copy_if_different Generated/viMouse3dStereoStyle.h             $dest/
copy_if_different Generated/viMouse3dFlowStyle.h               $dest/

copy_if_different Generated/draw3dViMouseParametersControlWidget.ui   $dest/
copy_if_different Generated/draw3dViMouseParametersControlWidget.cpp  $dest/
copy_if_different Generated/draw3dViMouseParameters.cpp               $dest/
copy_if_different Generated/draw3dViMouseParameters.h                 $dest/

echo "draw3d copied"


# Unified 

echo -n "Running generator on xml/opencvsgm.xml..."
./generatorConcept xml/opencvsgm.xml
echo "done"

echo -n "Running generator on xml/dummyprovider.xml..."
./generatorConcept xml/dummyprovider.xml
echo "done"

echo -n "Running generator on xml/libelas.xml..."
./generatorConcept xml/libelas.xml
echo "done"

dest="../../utils/corestructs/libWidgets"

copy_if_different Generated/openCVSGMParametersControlWidget.ui   $dest/
copy_if_different Generated/openCVSGMParametersControlWidget.cpp  $dest/
copy_if_different Generated/openCVSGMParameters.cpp               $dest/
copy_if_different Generated/openCVSGMParameters.h                 $dest/

copy_if_different Generated/openCVBMParametersControlWidget.ui    $dest/
copy_if_different Generated/openCVBMParametersControlWidget.cpp   $dest/
copy_if_different Generated/openCVBMParameters.cpp                $dest/
copy_if_different Generated/openCVBMParameters.h                  $dest/

copy_if_different Generated/libElasParametersControlWidget.ui    $dest/
copy_if_different Generated/libElasParametersControlWidget.cpp   $dest/
copy_if_different Generated/libElasParameters.cpp                $dest/
copy_if_different Generated/libElasParameters.h                  $dest/

copy_if_different Generated/dummyProviderParametersControlWidget.ui    $dest/
copy_if_different Generated/dummyProviderParametersControlWidget.cpp   $dest/
copy_if_different Generated/dummyProviderParameters.cpp                $dest/
copy_if_different Generated/dummyProviderParameters.h                  $dest/

