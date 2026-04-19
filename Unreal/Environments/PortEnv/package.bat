@echo off
REM //---------- set up variable ----------
setlocal EnableDelayedExpansion
set ROOT_DIR=%~dp0

set OutPath=%1
set ToolPath=%2
set UEVer=%3

if "%UEVer%"=="" set "UEVer=5.7"

if not "%ToolPath%"=="" goto toolpath_ready
set "_ToolPath=%PROGRAMFILES%\Epic Games\UE_%UEVer%\Engine\Build\BatchFiles"
if exist "%_ToolPath%" set "ToolPath=%_ToolPath%"
if not defined ToolPath (
	set "_ToolPath=E:\ProgramFile\UE_%UEVer%\Engine\Build\BatchFiles"
	if exist "%_ToolPath%" set "ToolPath=%_ToolPath%"
)
if not defined ToolPath (
	set "_ToolPath=E:\ProgramFile\Epic Games\UE_%UEVer%\Engine\Build\BatchFiles"
	if exist "%_ToolPath%" set "ToolPath=%_ToolPath%"
)
if "%ToolPath%"=="""" set ToolPath=
:toolpath_ready

IF NOT EXIST "%ToolPath%" (
	echo "Unreal Engine build path %ToolPath% was not found"
	goto :failed
)

if NOT EXIST "%ToolPath%\Build.bat" (
	echo "Unreal Build.bat was not found under %ToolPath%"
	goto :failed
)

if NOT EXIST "%ToolPath%\RunUAT.bat" (
	echo "Unreal RunUAT.bat was not found under %ToolPath%"
	goto :failed
)

tasklist /FI "IMAGENAME eq UnrealEditor.exe" | find /I "UnrealEditor.exe" >nul
if not errorlevel 1 (
	echo "UnrealEditor.exe is running. Close the editor and disable Live Coding before packaging."
	goto :failed
)

if "%OutPath%"=="" set "OutPath=D:\AirSimBuilds"
if NOT EXIST "%OutPath%" mkdir "%OutPath%"

echo Using ToolPath = %ToolPath%
echo Using OutPath = %OutPath%

for %%f in (*.uproject) do (
		echo Packaging: %%f
		
		call "%ToolPath%\Build.bat" "%%~nfEditor" Win64 Development -WarningsAsErrors -waitmutex "%cd%\%%f"
		if ERRORLEVEL 1 goto :failed
		
		REM "%ToolPath%\RunUAT" -ScriptsForProject="%cd%\%%f" BuildCookRun -installed -nop4 -project="%cd%\%%f" -cook -stage -archive -archivedirectory="%OutPath%" -package -clientconfig=Development -ue4exe=UE4Editor-Cmd.exe -compressed -pak -prereqs -nodebuginfo -targetplatform=Win64 -build -utf8output -nocompile -nocompileeditor 
		
		REM "%ToolPath%\RunUAT" BuildCookRun -project="%cd%\%%f" -noP4 -platform=Win64 -clientconfig=Development -serverconfig=Development -cook -rocket -allmaps -build -stage -NoCompile -nocompileeditor -pak -archive -archivedirectory="%OutPath%"
		
		if exist "%OutPath%\%%~nf" rmdir /s /q "%OutPath%\%%~nf"
		if exist "%OutPath%\WindowsNoEditor" rmdir /s /q "%OutPath%\WindowsNoEditor"
		if exist "%OutPath%\Windows" rmdir /s /q "%OutPath%\Windows"
		
		call "%ToolPath%\RunUAT.bat" BuildCookRun -project="%cd%\%%f" -noP4 -platform=Win64 -clientconfig=Development -cook -build -stage -pak -archive -archivedirectory="%OutPath%"  -utf8output -compressed -prereqs
		if ERRORLEVEL 1 goto :failed
		
		set "ArchiveDir=%OutPath%\WindowsNoEditor"
		if not exist "!ArchiveDir!" set "ArchiveDir=%OutPath%\Windows"
		if not exist "!ArchiveDir!" (
			echo "Expected archive output was not found under %OutPath%"
			goto :failed
		)
		move "!ArchiveDir!" "%OutPath%\%%~nf"
		if ERRORLEVEL 1 goto :failed
		
		@echo off
		echo start %%~nf -windowed> "%OutPath%\%%~nf\run.bat"
		if ERRORLEVEL 1 goto :failed
)

goto :done

:failed
echo "Error occured while packaging"
echo "Usage: package.bat <path\to\output> <path to Engine\Build\BatchFiles> <UE Version>"
exit /b 1

:done
if "%1"=="" pause
exit /b 0

	

