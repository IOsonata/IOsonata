#Requires -Version 5.1

<#
.SYNOPSIS
    Build IOsonata libraries for selected MCU target.

.DESCRIPTION
    Standalone script to build IOsonata libraries using Eclipse headless build.
    Can be run multiple times to build for different MCU targets.

.PARAMETER SdkHome
    Path to IOsonata SDK root (default: ~\IOcomposer)

.EXAMPLE
    .\build_iosonata_lib_win.ps1
    Build with default path

.EXAMPLE
    .\build_iosonata_lib_win.ps1 -SdkHome C:\Dev\IOcomposer
    Build with custom path

.NOTES
    Version: v1.0.0
    Platform: Windows
#>

param(
    [string]$SdkHome = "$env:USERPROFILE\IOcomposer"
)

$ErrorActionPreference = 'Stop'
$SCRIPT_VERSION = "v1.0.0"

Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "  IOsonata Library Builder (Windows)" -ForegroundColor White
Write-Host "  Version: $SCRIPT_VERSION" -ForegroundColor White
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host ""

$ROOT = $SdkHome
$ECLIPSE_DIR = "$env:ProgramFiles\Eclipse Embedded CDT"

# Check Eclipse
if (-not (Test-Path "$ECLIPSE_DIR\eclipse.exe")) {
    Write-Host "❌ ERROR: Eclipse not found at $ECLIPSE_DIR" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please install Eclipse: .\install_iocdevtools_win.ps1"
    exit 1
}

Write-Host "✓ Eclipse found at: $ECLIPSE_DIR" -ForegroundColor Green
Write-Host ""

# Check IOsonata
if (-not (Test-Path "$ROOT\IOsonata")) {
    Write-Host "❌ ERROR: IOsonata not found at $ROOT\IOsonata" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please clone: .\clone_iosonata_sdk_win.ps1 -Home $ROOT"
    exit 1
}

Write-Host "✓ IOsonata SDK found at: $ROOT\IOsonata" -ForegroundColor Green
Write-Host ""

# Discover projects
$mcuFamilies = @()
$mcuPaths = @()

Get-ChildItem -Path "$ROOT\IOsonata" -Recurse -Filter ".project" -ErrorAction SilentlyContinue | ForEach-Object {
    if ($_.DirectoryName -match "\\lib\\Eclipse$") {
        $projRoot = $_.DirectoryName
        $relPath = $projRoot.Replace("$ROOT\IOsonata\", "")
        $mcuFamilies += $relPath
        $mcuPaths += $projRoot
    }
}

if ($mcuFamilies.Count -eq 0) {
    Write-Host "⚠️  No IOsonata Eclipse projects found" -ForegroundColor Yellow
    exit 1
}

Write-Host "Available projects:" -ForegroundColor White
Write-Host ""
for ($i = 0; $i -lt $mcuFamilies.Count; $i++) {
    Write-Host ("  {0,2}) {1}" -f ($i + 1), $mcuFamilies[$i])
}
Write-Host "   0) Exit"
Write-Host ""

# User selection
do {
    $selection = Read-Host "Select project to build (0-$($mcuFamilies.Count))"
} while (-not ($selection -match '^\d+$') -or [int]$selection -lt 0 -or [int]$selection -gt $mcuFamilies.Count)

if ([int]$selection -eq 0) {
    Write-Host "Exiting."
    exit 0
}

# Get selection
$selectedIdx = [int]$selection - 1
$selectedFamily = $mcuFamilies[$selectedIdx]
$selectedPath = $mcuPaths[$selectedIdx]

# Validate
if (-not (Test-Path "$selectedPath\.project") -or -not (Test-Path "$selectedPath\.cproject")) {
    Write-Host "❌ ERROR: Not a valid Eclipse CDT project" -ForegroundColor Red
    exit 1
}

# Extract project name
$projName = ""
$projectFile = Get-Content "$selectedPath\.project" -Raw
if ($projectFile -match '<n>([^<]+)</n>') {
    $projName = $matches[1]
}

if ([string]::IsNullOrEmpty($projName)) {
    Write-Host "❌ ERROR: Could not determine project name" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host ">>> Building: $selectedFamily" -ForegroundColor Cyan
Write-Host "    Project: $projName"
Write-Host "    Path:    $selectedPath"
Write-Host ""

# Create workspace
$WS = "$env:TEMP\iosonata_build_ws_$PID"
New-Item -Path $WS -ItemType Directory -Force | Out-Null
Write-Host ">>> Workspace: $WS" -ForegroundColor Gray
Write-Host ">>> Running Eclipse headless build..." -ForegroundColor Cyan
Write-Host ""

$eclipseExe = "$ECLIPSE_DIR\eclipse.exe"
$logFile = "$env:TEMP\build_iosonata_lib.log"
if (Test-Path $logFile) { Remove-Item $logFile -Force }

# Build
$buildOutput = & $eclipseExe `
    --launcher.suppressErrors `
    -nosplash `
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
    -data $WS `
    -no-indexer `
    -import $selectedPath `
    -cleanBuild "$projName/.*" `
    -printErrorMarkers `
    2>&1

$buildOutput | Out-File -FilePath $logFile
$buildOutput | Write-Host

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "❌ Build failed (exit code $LASTEXITCODE)" -ForegroundColor Red
    Write-Host "   Log: $logFile"
    Write-Host "   Workspace: $WS"
    exit 1
}

Write-Host ""
Write-Host "✅ Build completed for $selectedFamily" -ForegroundColor Green
Write-Host ""
Write-Host "Libraries:" -ForegroundColor White
Get-ChildItem "$selectedPath\Debug\libIOsonata*.a" -ErrorAction SilentlyContinue | ForEach-Object { 
    Write-Host "  $($_.FullName) ($([math]::Round($_.Length/1KB, 1)) KB)"
}
Get-ChildItem "$selectedPath\Release\libIOsonata*.a" -ErrorAction SilentlyContinue | ForEach-Object { 
    Write-Host "  $($_.FullName) ($([math]::Round($_.Length/1KB, 1)) KB)"
}
Write-Host ""

Remove-Item -Path $WS -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "Build complete!" -ForegroundColor Green
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host ""
