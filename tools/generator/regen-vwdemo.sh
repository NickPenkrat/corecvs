#!/bin/bash

source ./helper-regen.sh

base="../../host-base/"
dest="../../vwdemo/"
utils="../../utils/"

qmake && make
./generatorConcept xml/vwdemo.xml 

echo "generated"

# generated 

copy_if_different Generated/baseWidget.cpp                $base/
copy_if_different Generated/base.cpp                      $base/generatedParameters
copy_if_different Generated/base.h                        $base/generatedParameters

copy_if_different Generated/featuringWidget.cpp           $utils/corestructs/
copy_if_different Generated/featuring.cpp                 $utils/corestructs/generatedParameters
copy_if_different Generated/featuring.h                   $utils/corestructs/generatedParameters

copy_if_different Generated/filteringWidget.cpp           $utils/corestructs/
copy_if_different Generated/filtering.cpp                 $utils/corestructs/generatedParameters
copy_if_different Generated/filtering.h                   $utils/corestructs/generatedParameters

copy_if_different Generated/vWWidget.cpp                  $dest/
copy_if_different Generated/vW.cpp                        $dest/generatedParameters
copy_if_different Generated/vW.h                          $dest/generatedParameters

copy_if_different Generated/parametersMapper.cpp          $dest/parametersMapper
copy_if_different Generated/parametersMapper.h            $dest/parametersMapper

# hand-written

cp reflection.h                            $utils/corestructs/

echo "copied"
