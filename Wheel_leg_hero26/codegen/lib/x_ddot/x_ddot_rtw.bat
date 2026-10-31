@echo off

set MATLAB=D:\Program Files\MATLAB\R2023a


call  "\\LAPTOP-325KEB6N\D$\Program Files\MATLAB\R2023a\bin\win64\checkMATLABRootForDriveMap.exe" "\\LAPTOP-325KEB6N\D$\Program Files\MATLAB\R2023a"  > mlEnv.txt
for /f %%a in (mlEnv.txt) do set "%%a"\n
cd .

if "%1"=="" ("D:\Program Files\MATLAB\R2023a\bin\win64\gmake" MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  -f x_ddot_rtw.mk all) else ("D:\Program Files\MATLAB\R2023a\bin\win64\gmake" MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  -f x_ddot_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1