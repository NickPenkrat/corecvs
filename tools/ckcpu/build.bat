::@call ..\..\..\scripts\windows\_setvars.bat vc
if     "%VS140COMNTOOLS%" == "" call "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" x86_amd64
if not "%VS140COMNTOOLS%" == "" call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86_amd64

cl.exe /EHsc cpu_features.c
del cpu_features.obj