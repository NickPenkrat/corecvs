::SET PATH=%PATH%;%OPENCV_DIR%\x64\vc12\bin\
::SET PATH=%PATH%;D:\Dev\Qt\Qt5.4.1\5.4\msvc2013_64_opengl\bin

set ROOT_DIR=..\..\..\..
set DATA_DIR=%ROOT_DIR%/data/testdata/distortion

call %ROOT_DIR%\scripts\windows\_setvars.bat x64 vc

%ROOT_DIR%\bin\opencvLineDetector.exe --calcFullCheckerBoard:%DATA_DIR%/SPA0_360deg.jpg --chessW:18 --chessH:11 --use_green_channel --json_file_name:%DATA_DIR%/calib_full.json --verbose