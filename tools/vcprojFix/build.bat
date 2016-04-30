if exist *.vcxproj*	del *.vcxproj*

call ..\..\..\..\scripts\windows\qmake.bat -tp vc vcprojFix.pro "CONFIG+=gen_vsproj"

rd debug
rd release

msbuild /t:Rebuild /p:Configuration=Release
