@REM run as Administrator
@echo off
cd /d %~dp0
set DOWNLOADS_DIR=%USERPROFILE%\Downloads
set DOWNLOADS_DIR_LINUX=%DOWNLOADS_DIR:\=/%

SET PATH=^
%DOWNLOADS_DIR%\avr8\avr8;^
%DOWNLOADS_DIR%\avr8\avr8\avrassembler;^
%DOWNLOADS_DIR%\PortableGit\usr\bin;

%DOWNLOADS_DIR%\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\mingw32-make.exe clean
avrasm2.exe -fI -o "EIE3105_ATmega328P_Application.hex"  -m "EIE3105_ATmega328P_Application.map"  -l "EIE3105_ATmega328P_Application.lss"  -S "EIE3105_ATmega328P_Application.tmp" -W+ie -I"%DOWNLOADS_DIR%\Atmel.ATmega_DFP.2.1.506\avrasm\inc" -im328Pdef.inc -d "EIE3105_ATmega328P_Application.obj"  "main.asm"
pause


