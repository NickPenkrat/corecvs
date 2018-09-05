if exist *.vcxproj*	del *.vcxproj*

set qmake=..\..\..\..\scripts\windows\qmake.bat
if not exist %qmake% set qmake=qmake

call %qmake% -tp vc vcprojFix.pro "CONFIG+=gen_vsproj"

rd debug
rd release

msbuild /t:Rebuild /p:Configuration=Release
