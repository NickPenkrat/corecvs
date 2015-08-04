SET PATH=%PATH%;%OPENCV_DIR%\x64\vc12\bin\
SET PATH=%PATH%;D:\Dev\Qt\Qt5.4.1\5.4\msvc2013_64_opengl\bin

opencvLineDetector.exe --calcFullCheckerBoard:SPA0_360deg_1.bmp --chessW:18 --chessH:11 --precise:100 --use_green_channel --max_iteration_count:50 --min_accuracy:0.001