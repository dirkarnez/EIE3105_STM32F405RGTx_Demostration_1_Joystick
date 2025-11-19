@REM run as Administrator
@echo off

if not exist "%USERPROFILE%\Downloads\PortableGit" (
cd /d "%TEMP%" && ^
%SystemRoot%\System32\curl.exe https://github.com/git-for-windows/git/releases/download/v2.42.0.windows.2/PortableGit-2.42.0.2-64-bit.7z.exe -L -O -J  && ^
C:\PROGRA~1\7-Zip\7z.exe x PortableGit-2.42.0.2-64-bit.7z.exe -o"%USERPROFILE%\Downloads\PortableGit"  && ^
del PortableGit-2.42.0.2-64-bit.7z.exe
)

if not exist "%USERPROFILE%\Downloads\x86_64-8.1.0-release-posix-seh-rt_v6-rev0" (
cd /d "%TEMP%" && ^
%SystemRoot%\System32\curl.exe "https://github.com/dirkarnez/cpp-tools/raw/main/x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z" -L -O -J && ^
C:\PROGRA~1\7-Zip\7z.exe x x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z -o"%USERPROFILE%\Downloads\x86_64-8.1.0-release-posix-seh-rt_v6-rev0" && ^
del x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z
)

if not exist "%USERPROFILE%\Downloads\avr8" (
cd /d "%TEMP%" && ^
%SystemRoot%\System32\curl.exe "https://raw.githubusercontent.com/dirkarnez/avr8-toolchain/main/avr8.zip" -L -O -J && ^
C:\PROGRA~1\7-Zip\7z.exe x avr8.zip -o"%USERPROFILE%\Downloads\avr8" && ^
del avr8.zip
)

if not exist "%USERPROFILE%\Downloads\Atmel.ATmega_DFP.2.1.506" (
cd /d "%TEMP%" && ^
%SystemRoot%\System32\curl.exe "http://packs.download.atmel.com/Atmel.ATmega_DFP.2.1.506.atpack" -L -O -J && ^
C:\PROGRA~1\7-Zip\7z.exe x Atmel.ATmega_DFP.2.1.506.atpack -o"%USERPROFILE%\Downloads\Atmel.ATmega_DFP.2.1.506" && ^
del Atmel.ATmega_DFP.2.1.506.atpack
)

cd /d "%USERPROFILE%\Downloads" && ^
%SystemRoot%\System32\curl.exe "https://marketplace.visualstudio.com/_apis/public/gallery/publishers/ms-vscode/vsextensions/cpptools/1.22.9/vspackage?targetPlatform=win32-x64" -L -O -J

pause
