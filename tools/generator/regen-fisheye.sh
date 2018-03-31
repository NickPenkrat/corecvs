#!/bin/bash

source ./helper-regen.sh

fisheye_egomotion="../../../../src/restricted/adas/applications/fisheye_egomotion/"

echo -n "Building generator... "
qmake && make
echo "done"

mkdir -p Generated

echo -n "Running generator on xml/fisheye.xml..."
${GENERATOR_BIN}  ../../../../src/restricted/adas/applications/fisheye_egomotion/xml/fisheye.xml
echo "done"


echo "Making a copy of Opencv Wrapper classes"
copy_if_different Generated/fisheyeEgomotionParameters.cpp     $fisheye_egomotion/xml/generated
copy_if_different Generated/fisheyeEgomotionParameters.h       $fisheye_egomotion/xml/generated

echo "copied"
