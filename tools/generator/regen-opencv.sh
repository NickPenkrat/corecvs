#!/bin/bash

source ./helper-regen.sh

opencv_wrapper="../../wrappers/opencv/"

echo -n "Building generator... "
qmake && make
echo "done"

mkdir -p Generated

echo -n "Running generator on xml/merger.xml..."
${GENERATOR_BIN}  ../../wrappers/opencv/xml/opencv_wrapper.xml
echo "done"


echo "Making a copy of Opencv Wrapper classes"
copy_if_different Generated/meshFlowDrawParameters.cpp     $opencv_wrapper/xml/generated
copy_if_different Generated/meshFlowDrawParameters.h       $opencv_wrapper/xml/generated

echo "copied"
