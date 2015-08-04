::SET PATH=%PATH%;%OPENCV_DIR%\x64\vc12\bin\
::SET PATH=%PATH%;D:\Dev\Qt\Qt5.4.1\5.4\msvc2013_64_opengl\bin

call ..\..\..\..\scripts\windows\_setvars.bat x64 vc

..\..\..\..\bin\opencvLineDetector.exe --calcFullCheckerBoard:../../../../data/testdata/distortion/SPA0_360deg.jpg --chessW:18 --chessH:11 --use_green_channel --verbose