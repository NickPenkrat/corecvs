::SET PATH=%PATH%;%OPENCV_DIR%\x64\vc12\bin\
::SET PATH=%PATH%;D:\Dev\Qt\Qt5.4.1\5.4\msvc2013_64_opengl\bin

::opencvLineDetector.exe --calcPartCheckerBoard:test.bmp --chessW:17 --chessH:11 --verbose --draw_proccess --precise:50 --use_green_channel --max_iteration_count:50 --min_accuracy:0.001 --json_file_name:test_50_50_001.json --prefix:dist_5500
::rem opencvLineDetector.exe --calcFullCheckerBoard:snapshot_snapshot_See3CAMCU50__RGB_1.bmp --chessW:18 --chessH:11 --verbose --draw_proccess --precise:40 --use_green_channel --max_iteration_count:50 --min_accuracy:0.001

set ROOT_DIR=..\..\..\..
set DATA_DIR=%ROOT_DIR%/data/testdata/distortion

call %ROOT_DIR%\scripts\windows\_setvars.bat x64 vc

%ROOT_DIR%\bin\opencvLineDetector.exe --calcPartCheckerBoard:%DATA_DIR%/SPA0_30deg.jpg --chessW:18 --chessH:11 --use_green_channel --json_file_name:%DATA_DIR%/calib_part.json --verbose --draw_proccess