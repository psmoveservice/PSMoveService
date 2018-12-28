@echo off
del /Q csharp\*.cs > nul
del /Q csharp\*.c > nul
..\deps\swig\src\swig\swig.exe -csharp -namespace PSMoveService -outdir csharp -dllimport PSMoveClient_swig_csharp -I../src/psmoveprotocol -I../src/psmoveclient -o csharp/PSMoveClientCSHARP_wrap.c PSMoveClient.i
pause