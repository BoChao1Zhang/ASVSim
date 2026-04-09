@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

set RepoRoot=%1

if "%RepoRoot%"=="" set "RepoRoot=..\..\.."

IF NOT EXIST "%RepoRoot%\Unreal\Plugins\AirSim\AirSim.uplugin" (
	echo "RepoRoot %RepoRoot% does not contain Unreal\Plugins\AirSim\AirSim.uplugin"
	goto :failed
)

echo Using RepoRoot = %RepoRoot%
echo PortEnv now resolves AirSim and UEEditorMCP through AdditionalPluginDirectories.
echo No local Plugins mirror will be created.

cmd /c clean.bat
cmd /c GenerateProjectFiles.bat

goto :done

:failed
echo Error occured while updating.
exit /b 1

:done
if "%1"=="" pause
