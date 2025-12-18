@echo off
echo Activating virtual environment...
call .venv\Scripts\activate.bat
echo Virtual environment activated!
echo Current Python: 
where python
cmd /k