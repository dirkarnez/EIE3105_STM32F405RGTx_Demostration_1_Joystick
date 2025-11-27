REM run as Administrator
@echo off

set DOWNLOADS_DIR=%USERPROFILE%\Downloads
set DOWNLOADS_DIR_LINUX=%DOWNLOADS_DIR:\=/%
set SOFTWARES_DIR=D:\Softwares

SET PATH=^
%DOWNLOADS_DIR%\STM32CubeIDE-portable-v1.6.1\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623\tools\bin;^
%SOFTWARES_DIR%\STM32CubeIDE-portable-v1.6.1\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623\tools\bin;^
%DOWNLOADS_DIR%\STM32CubeIDE-portable-v1.6.1\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.1.300.202402091052\tools\bin;^
%SOFTWARES_DIR%\STM32CubeIDE-portable-v1.6.1\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.1.300.202402091052\tools\bin;

cd .ci && ( ( make clean && echo =========clean ok========= && make main-build && echo ok ) || echo failed ) && pause