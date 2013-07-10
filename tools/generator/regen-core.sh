#!/bin/bash

#--------------------------------------------------------------------------------------------
#
#   This script regenerates ui and classes for both core and res-core 
#
#--------------------------------------------------------------------------------------------


source ./helper-regen.sh

qmake && make

CORE_DIR="../../core"
GEN_DIR="./Generated"
GENERATOR_BIN="./generator"


#--------------------------------------------------------------------------------------------
#
#   Helper function 
#
#   Uses 
#      * XML_DIR as folder with xml sources
#      * DST_DIR
#      * PRIFILE
#      * WIDGETS_DIR
#      * WIDGETS_REL_DIR
#
#--------------------------------------------------------------------------------------------
generate_and_copy () {
    
    xmls=`ls ${XML_DIR}/*.xml`
    
    mkdir ${GEN_DIR}
    
    for xml in $xmls; do 
        echo ========== Generating for $xml ============= 
        $GENERATOR_BIN $xml
    done;
    
    cat >>${PRIFILE} <<END_TEXT
##################################################################
#  This QMAKE PRI File was autogenerated `date`
##################################################################

END_TEXT

    echo  >>${PRIFILE} "#####################################################"     
    echo  >>${PRIFILE} "# Classes Section"     
    echo  >>${PRIFILE} "#####################################################"     

    for class in $classes; do
        echo copying ${class}
        copy_if_different ${GEN_DIR}/${class}.h ${DST_DIR}
        copy_if_different ${GEN_DIR}/${class}.cpp ${DST_DIR}
        copy_if_different ${GEN_DIR}/${class}ControlWidget.cpp ${WIDGETS_DIR} 
        
        echo  >>${PRIFILE} SOURCES+= ${DST_DIR}/${class}.cpp  
        echo  >>${PRIFILE} HEADERS+= ${DST_DIR}/${class}.h
        echo  >>${PRIFILE}    
    done;
    
    echo  >>${PRIFILE} "#####################################################"     
    echo  >>${PRIFILE} "# UI Classes Section"     
    echo  >>${PRIFILE} "#####################################################"     
    
    for class in $ui_classes; do
        echo copying ${class}
        copy_if_different ${GEN_DIR}/${class}.h ${DST_DIR}
        copy_if_different ${GEN_DIR}/${class}.cpp ${DST_DIR}
    	copy_if_different ${GEN_DIR}/${class}ControlWidget.cpp ${WIDGETS_DIR} 
        
        echo  >>${PRIFILE} SOURCES+= ${DST_DIR}/${class}.cpp  
        echo  >>${PRIFILE} HEADERS+= ${DST_DIR}/${class}.h
        echo  >>${PRIFILE}    
    
        echo  >>${WPRIFILE} SOURCES+= ${WIDGETS_REL_DIR}/${class}ControlWidget.cpp  
        echo  >>${WPRIFILE} HEADERS+= ${WIDGETS_REL_DIR}/${class}ControlWidget.h
        echo  >>${WPRIFILE} FORMS  += ${WIDGETS_REL_DIR}/${class}ControlWidget.ui
        echo  >>${WPRIFILE} 
        
        headerFile="${WIDGETS_DIR}/${class}ControlWidget.h"
        if [[ ! -e $headerFile ]]; then
        	./h_stub.sh ${class}
        	mv ${class}ControlWidget.h ${WIDGETS_DIR}
        fi;
          
    done;

    echo  >>${PRIFILE} "#####################################################"     
    echo  >>${PRIFILE} "# Full UI Classes Section"     
    echo  >>${PRIFILE} "#####################################################"     
    
    for class in $full_ui_classes; do
        echo copying ${class}
        copy_if_different ${GEN_DIR}/${class}.h ${DST_DIR}
        copy_if_different ${GEN_DIR}/${class}.cpp ${DST_DIR}
        copy_if_different ${GEN_DIR}/${class}ControlWidget.cpp ${WIDGETS_DIR}
        copy_if_different ${GEN_DIR}/${class}ControlWidget.ui  ${WIDGETS_DIR}  
        
        echo  >>${PRIFILE} SOURCES+= ${DST_DIR}/${class}.cpp  
        echo  >>${PRIFILE} HEADERS+= ${DST_DIR}/${class}.h
        echo  >>${PRIFILE}    
    
        echo  >>${WPRIFILE} SOURCES+= ${WIDGETS_REL_DIR}/${class}ControlWidget.cpp  
        echo  >>${WPRIFILE} HEADERS+= ${WIDGETS_REL_DIR}/${class}ControlWidget.h
        echo  >>${WPRIFILE} FORMS  += ${WIDGETS_REL_DIR}/${class}ControlWidget.ui
        echo  >>${WPRIFILE}
        
        headerFile="${WIDGETS_DIR}/${class}ControlWidget.h"
        if [[ ! -e $headerFile ]]; then
        	./h_stub.sh ${class}
        	mv ${class}ControlWidget.h ${WIDGETS_DIR}
        fi;
    done;

    echo  >>${PRIFILE} "#####################################################"     
    echo  >>${PRIFILE} "# Enum Section"     
    echo  >>${PRIFILE} "#####################################################"     
    
    for enum in $enums; do
        copy_if_different ${GEN_DIR}/${enum}.h ${DST_DIR}
        echo  >>${PRIFILE} HEADERS+= ${DST_DIR}/${enum}.h
        echo  >>${PRIFILE}     
    done;

}

#--------------------------------------------------------------------------------------------
#
#   This part is for core filters
#
#--------------------------------------------------------------------------------------------

XML_DIR="${CORE_DIR}/xml"
DST_DIR="${CORE_DIR}/xml/generated"

UTILS_DIR="../../utils"
WIDGETS_DIR="${UTILS_DIR}/filters/ui"
WIDGETS_REL_DIR="\$\${UTILSDIR}/filters/ui"
WPRIFILE="${WIDGETS_DIR}/filterWidgets.pri"

enums="sobelMixingType openCVBinaryFilterType operation interpolationType inputType outputType"
classes=""
ui_classes=""         
full_ui_classes="
        bitSelectorParameters
        gainOffsetParameters
        sobelParameters
        cannyParameters
        openCVFilterParameters
        backgroundFilterParameters
        binarizeParameters
        thickeningParameters
        maskingParameters
        outputFilterParameters
        inputFilterParameters

        operationParameters
        "

PRIFILE="${DST_DIR}/generated.pri"

cat >${WPRIFILE} <<END_TEXT
##################################################################
#  This QMAKE PRI File was autogenerated at `date`
##################################################################
END_TEXT

# Call actual generation
echo >${PRIFILE}
generate_and_copy

#--------------------------------------------------------------------------------------------
#
#   This part is for core additional files
#
#--------------------------------------------------------------------------------------------

XML_DIR="${CORE_DIR}/xml"
DST_DIR="${CORE_DIR}/xml/generated"
UTILS_DIR="../../utils"

WIDGETS_DIR="${UTILS_DIR}/corestructs/coreWidgets"
WIDGETS_REL_DIR="\$\${UTILSDIR}/corestructs/coreWidgets"
WPRIFILE="${WIDGETS_DIR}/coreWidgets.pri"

enums="sobelMixingType openCVBinaryFilterType operation interpolationType"
classes=""
ui_classes="
        rgbColorParameters
        "
                 
full_ui_classes="   
        axisAlignedBoxParameters
        headSearchParameters
        "

PRIFILE="${DST_DIR}/generated.pri"

echo Preparing PRI file for widgets generated from CORE \"${WPRIFILE}\" 

cat >${WPRIFILE} <<END_TEXT
##################################################################
#  This QMAKE PRI File was autogenerated `date`
##################################################################
END_TEXT

# Call actual generation
generate_and_copy



