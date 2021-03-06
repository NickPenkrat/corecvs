::SET PATH=%PATH%;%OPENCV_DIR%\x64\vc12\bin\
::SET PATH=%PATH%;D:\Dev\Qt\Qt5.4.1\5.4\msvc2013_64_opengl\bin

::opencvLineDetector.exe --apply SPA0_30deg_1.bmp SPA0_360deg_1.bmp --verbose --is_inverse --prefix:dist002_
::opencvLineDetector.exe --apply dist002_SPA0_30deg_1.bmp dist002_SPA0_360deg_1.bmp --verbose --!is_inverse --prefix:inverse002_

set ROOT_DIR=..\..\..\..
set DATA_DIR=%ROOT_DIR%/data/testdata/distortion

call %ROOT_DIR%\scripts\windows\_setvars.bat x64 vc

%ROOT_DIR%\bin\opencvLineDetector.exe --apply --json_file_name:%DATA_DIR%/calib_full.json %DATA_DIR%/SPA0_360deg.jpg %DATA_DIR%/SPA0_30deg.jpg