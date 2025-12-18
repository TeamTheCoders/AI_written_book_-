Write-Host "Activating virtual environment..."
& .venv\Scripts\Activate.ps1
Write-Host "Virtual environment activated!"
Write-Host "Current Python:"
Get-Command python