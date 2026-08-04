# run_renode.ps1 - Launch Renode VBS RP2040 Simulation
param (
    [string]$ElfPath = "",
    [int]$Port = 4321
)

# Resolve workspace root directory
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$WorkspaceRoot = (Get-Item $ScriptDir).Parent.FullName

if ([string]::IsNullOrWhiteSpace($ElfPath)) {
    $ElfPath = Join-Path $WorkspaceRoot "build\vbs_cpp_portugal.elf"
} elseif (-not [System.IO.Path]::IsPathRooted($ElfPath)) {
    $ElfPath = Join-Path (Get-Location).Path $ElfPath
}

Write-Host "==================================================" -ForegroundColor Cyan
Write-Host " Starting VBS Renode Simulation (RP2040 Emulation) " -ForegroundColor Cyan
Write-Host " Target ELF: $ElfPath" -ForegroundColor Yellow
Write-Host " UART0 Port: $Port" -ForegroundColor Yellow
Write-Host "==================================================" -ForegroundColor Cyan

if (-not (Test-Path $ElfPath)) {
    Write-Host "[ERROR] Target ELF file not found at: $ElfPath" -ForegroundColor Red
    Write-Host "Please compile the C++ firmware first using CMake & Ninja." -ForegroundColor Yellow
}

# Clean up any lingering Renode processes using port 4321
$RunningRenode = Get-Process -Name "Renode" -ErrorAction SilentlyContinue
if ($RunningRenode) {
    Write-Host "Closing previous Renode instance(s)..." -ForegroundColor Yellow
    Stop-Process -Name "Renode" -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 1
}

# Locate Renode executable
$RenodePath = $null
if (Get-Command "renode" -ErrorAction SilentlyContinue) {
    $RenodePath = (Get-Command "renode").Source
} else {
    # Check common installation locations on Windows
    $CandidatePaths = @(
        "C:\Program Files\Renode\bin\Renode.exe",
        "C:\Program Files\Renode\renode.exe",
        "C:\Program Files (x86)\Renode\bin\Renode.exe",
        "C:\Program Files (x86)\Renode\renode.exe",
        "$env:LOCALAPPDATA\Programs\Renode\renode.exe",
        "C:\ProgramData\chocolatey\bin\renode.exe",
        "$env:USERPROFILE\scoop\apps\renode\current\renode.exe"
    )
    foreach ($path in $CandidatePaths) {
        if (Test-Path $path) {
            $RenodePath = $path
            break
        }
    }
}

$RescPath = Join-Path $WorkspaceRoot "renode\vbs_simulation.resc"

if ($RenodePath) {
    Write-Host "Launching Renode: $RenodePath (Monitor Telnet Server on Port 1234) ..." -ForegroundColor Green
    Set-Location $WorkspaceRoot
    & $RenodePath --port 1234 -e "`$bin=@`"$ElfPath`"; include @`"$RescPath`""
} else {
    Write-Host "[WARNING] Renode executable not found in PATH or standard installation paths." -ForegroundColor Red
    Write-Host "Please install Renode from https://github.com/renode/renode/releases or https://renode.io" -ForegroundColor Yellow
    Write-Host "Once installed, run this script again or execute:" -ForegroundColor Yellow
    Write-Host "  renode -e `"`$bin=@`"$ElfPath`"; include @`"$RescPath`"`"" -ForegroundColor Gray
}
