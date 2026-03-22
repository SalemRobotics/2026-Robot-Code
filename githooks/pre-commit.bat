@echo off
echo Running Spotless...
call gradlew.bat :spotlessApply
if errorlevel 1 exit /b 1

echo Staging Spotless changes...
git add -A .
