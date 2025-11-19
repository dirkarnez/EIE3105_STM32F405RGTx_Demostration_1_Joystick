@REM run as Administrator
@echo off
cd /d %~dp0
set DOWNLOADS_DIR=%USERPROFILE%\Downloads
set DOWNLOADS_DIR_LINUX=%DOWNLOADS_DIR:\=/%


SET PATH=^
%DOWNLOADS_DIR%\avr8\avr8;^
%DOWNLOADS_DIR%\avr8\avr8\avr8-gnu-toolchain\bin;^
%DOWNLOADS_DIR%\PortableGit\usr\bin;


%DOWNLOADS_DIR%\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\mingw32-make.exe clean
%DOWNLOADS_DIR%\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\mingw32-make.exe all
pause