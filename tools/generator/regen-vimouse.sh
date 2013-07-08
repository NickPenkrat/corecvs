#!/bin/bash

source ./helper-regen.sh

dest="../../host-soft/"
fdest="../../viflow-base/"

echo -n "Building generator... "
qmake && make
echo "done"

echo -n "Running generator on xml/vimouse.xml..."
./generatorConcept xml/vimouse.xml
echo "done"

# This block will be then moved out of the application
echo -n "Running generator on xml/faceDetection.xml..."
./generatorConcept xml/faceDetection.xml
echo "done"

echo -n "Running generator on xml/openni.xml..."
./generatorConcept xml/openni.xml
echo "done"

echo "Makeing a copy of base classes"
./copy-base.sh

copy_if_different Generated/viMouseParametersControlWidget.cpp  $dest/
copy_if_different Generated/viMouseParameters.cpp               $dest/generatedParameters
copy_if_different Generated/osdStyle.h                          $dest/generatedParameters
copy_if_different Generated/mousePointerType.h                  $dest/generatedParameters
copy_if_different Generated/viMouseParameters.h                 $dest/generatedParameters

copy_if_different Generated/proc6DParametersControlWidget.ui    $fdest/ui
copy_if_different Generated/proc6DParametersControlWidget.cpp   $fdest/
copy_if_different Generated/proc6DParameters.cpp                $fdest/generatedParameters
copy_if_different Generated/proc6DParameters.h                  $fdest/generatedParameters

copy_if_different Generated/osdParametersControlWidget.cpp      $dest/
copy_if_different Generated/osdParameters.cpp                   $dest/generatedParameters
copy_if_different Generated/osdParameters.h                     $dest/generatedParameters


copy_if_different Generated/zoneParameters.cpp                       $dest/generatedParameters
copy_if_different Generated/zoneParameters.h                         $dest/generatedParameters
copy_if_different Generated/clusteringParameters.cpp                 $dest/generatedParameters
copy_if_different Generated/clusteringParameters.h                   $dest/generatedParameters
copy_if_different Generated/newClusteringParameters.cpp              $dest/generatedParameters
copy_if_different Generated/newClusteringParameters.h                $dest/generatedParameters
copy_if_different Generated/newClusteringParametersControlWidget.cpp $dest/
copy_if_different Generated/newClusteringParametersControlWidget.ui  $dest/ui


copy_if_different Generated/mouseHandlerParameters.cpp          $dest/pointerTracker/mouseHandler
copy_if_different Generated/mouseHandlerParameters.h            $dest/pointerTracker/mouseHandler
copy_if_different Generated/stabilizerType.h                    $dest/pointerTracker

copy_if_different Generated/swipeDetectorParameters.cpp         $dest/pointerTracker
copy_if_different Generated/swipeDetectorParameters.h           $dest/pointerTracker
copy_if_different Generated/swipeAlgorithmType.h                $dest/pointerTracker
copy_if_different Generated/patternType.h     			        $dest/pointerTracker

copy_if_different Generated/parametersMapperViMouse.cpp         $dest/parametersMapper
copy_if_different Generated/parametersMapperViMouse.h           $dest/parametersMapper

copy_if_different Generated/parametersMapperViMouse.cpp         $dest/parametersMapper
copy_if_different Generated/parametersMapperViMouse.h           $dest/parametersMapper

# This block is about faceDetection 

copy_if_different Generated/personGeometryParameters.h                $dest/faceDetection
copy_if_different Generated/personGeometryParameters.cpp              $dest/faceDetection
copy_if_different Generated/personGeometryParametersControlWidget.cpp $dest/faceDetection
copy_if_different Generated/personGeometryParametersControlWidget.ui  $dest/faceDetection


copy_if_different Generated/faceDetectionAlgorithmType.h              $dest/faceDetection
copy_if_different Generated/faceDetectionCascadeType.h                $dest/faceDetection
copy_if_different Generated/faceDetectionParameters.h                 $dest/faceDetection
copy_if_different Generated/faceDetectionParameters.cpp               $dest/faceDetection
copy_if_different Generated/faceDetectionParametersControlWidget.ui   $dest/faceDetection
copy_if_different Generated/faceDetectionParametersControlWidget.cpp  $dest/faceDetection
copy_if_different Generated/showcaseStateMachineParameters.h          $dest/faceDetection
copy_if_different Generated/showcaseStateMachineParameters.cpp        $dest/faceDetection

# This is a block about OpenNi Transport
copy_if_different Generated/openNITransportParameters.h                 $dest/openNiTransport
copy_if_different Generated/openNITransportParameters.cpp               $dest/openNiTransport
copy_if_different Generated/openNITransportParametersControlWidget.ui   $dest/openNiTransport
copy_if_different Generated/openNITransportParametersControlWidget.cpp  $dest/openNiTransport


echo "vimouse copied"
