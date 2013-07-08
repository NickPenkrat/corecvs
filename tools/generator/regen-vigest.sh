#!/bin/bash

source ./helper-regen.sh

dest="../../viGest/"

echo -n "Building generator... "
qmake && make
echo "done"

echo -n "Running generator on xml/vigest.xml..."
./generatorConcept xml/vigest.xml  
echo "done"


echo "Makeing a copy of base classes"
./copy-base.sh


copy_if_different Generated/viGestParametersControlWidget.cpp   $dest/
copy_if_different Generated/viGestParameters.cpp                $dest/generatedParameters
copy_if_different Generated/presentationType.h                  $dest/generatedParameters
copy_if_different Generated/viGestParameters.h                  $dest/generatedParameters

copy_if_different Generated/parametersMapperViGest.cpp          $dest/parametersMapper
copy_if_different Generated/parametersMapperViGest.h            $dest/parametersMapper

# hand-written
echo "copied"