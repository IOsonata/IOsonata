#Requires -Version 5.1

<#
.SYNOPSIS
    Build IOsonata libraries for selected MCU target.

.DESCRIPTION
    Standalone script to build IOsonata libraries using a headless build.
    Prefers IOcomposer (the new IOsonata installer); falls back to Eclipse
    Embedded CDT if IOcomposer is not found. IOcomposer is Eclipse-based,
    so the same headless build application is used regardless.
    Can be run multiple times to build for different MCU targets.
    Supports building all projects at once.

.PARAMETER SdkHome
    Path to IOsonata SDK root (default: ~\IOcomposer)

.PARAMETER IocomposerDir
    Path to IOcomposer install dir (default: %ProgramFiles%\IOcomposer)

.PARAMETER EclipseDir
    Path to Eclipse Embedded CDT install dir, used as fallback
    (default: %ProgramFiles%\Eclipse Embedded CDT)

.EXAMPLE
    .\build_iosonata_lib_win.ps1
    Build with default paths

.EXAMPLE
    .\build_iosonata_lib_win.ps1 -IocomposerDir "C:\Tools\IOcomposer"
    Build with custom IOcomposer location

.NOTES
    Version: v2.3.0
    Platform: Windows

    v2.3.0: Prefer IOcomposer; fall back to Eclipse if not found.
#>

param(
    [string]$SdkHome        = "$env:USERPROFILE\IOcomposer",
    [string]$IocomposerDir  = "$env:ProgramFiles\IOcomposer",
    [string]$EclipseDir     = "$env:ProgramFiles\Eclipse Embedded CDT"
)

$ErrorActionPreference = 'Stop'
$SCRIPT_VERSION = "v2.3.0"

# --- Helper: Print Banner ---
function Show-Banner {
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "  IOsonata Library Builder (Windows)" -ForegroundColor White
    Write-Host "  Version: $SCRIPT_VERSION" -ForegroundColor White
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host ""
}

# --- Helper: Find a usable launcher inside an install dir ---
# IOcomposer is Eclipse-based, so its launcher may be named
# 'iocomposer.exe' / 'iocomposerc.exe' or still 'eclipse.exe' / 'eclipsec.exe'.
function Find-IdeLauncher {
    param(
        [string]$InstallDir
    )
    if (-not (Test-Path $InstallDir)) { return $null }
    $candidates = @(
        'iocomposerc.exe',
        'iocomposer.exe',
        'IOcomposer.exe',
        'eclipsec.exe',
        'eclipse.exe'
    )
    foreach ($exe in $candidates) {
        $full = Join-Path $InstallDir $exe
        if (Test-Path $full) { return $full }
    }
    return $null
}

Show-Banner

$ROOT = $SdkHome

# --- IDE Detection (prefer IOcomposer, fall back to Eclipse) ---
$IDE_BIN  = $null
$IDE_DIR  = $null
$IDE_NAME = $null

# Candidate IOcomposer install directories
$iocomposerCandidates = @(
    $IocomposerDir,
    "$env:ProgramFiles\IOcomposer",
    "${env:ProgramFiles(x86)}\IOcomposer",
    "$env:LOCALAPPDATA\IOcomposer",
    "$env:USERPROFILE\IOcomposer\iocomposer",
    "$ROOT\iocomposer"
) | Where-Object { $_ -and ($_ -ne '') } | Select-Object -Unique

foreach ($dir in $iocomposerCandidates) {
    $launcher = Find-IdeLauncher -InstallDir $dir
    if ($launcher) {
        $IDE_BIN  = $launcher
        $IDE_DIR  = $dir
        $IDE_NAME = "IOcomposer"
        break
    }
}

# Fall back to Eclipse
if (-not $IDE_BIN) {
    $eclipseCandidates = @(
        $EclipseDir,
        "$env:ProgramFiles\Eclipse Embedded CDT",
        "${env:ProgramFiles(x86)}\Eclipse Embedded CDT",
        "$env:ProgramFiles\Eclipse",
        "$env:LOCALAPPDATA\Eclipse Embedded CDT"
    ) | Where-Object { $_ -and ($_ -ne '') } | Select-Object -Unique

    foreach ($dir in $eclipseCandidates) {
        $launcher = Find-IdeLauncher -InstallDir $dir
        if ($launcher) {
            $IDE_BIN  = $launcher
            $IDE_DIR  = $dir
            $IDE_NAME = "Eclipse"
            break
        }
    }
}

if (-not $IDE_BIN) {
    Write-Host "ERROR: Neither IOcomposer nor Eclipse found" -ForegroundColor Red
    Write-Host ""
    Write-Host "Searched IOcomposer locations:" -ForegroundColor Yellow
    foreach ($d in $iocomposerCandidates) { Write-Host "  - $d" }
    Write-Host ""
    Write-Host "Searched Eclipse locations:" -ForegroundColor Yellow
    Write-Host "  - $EclipseDir"
    Write-Host "  - $env:ProgramFiles\Eclipse Embedded CDT"
    Write-Host "  - ${env:ProgramFiles(x86)}\Eclipse Embedded CDT"
    Write-Host "  - $env:ProgramFiles\Eclipse"
    Write-Host "  - $env:LOCALAPPDATA\Eclipse Embedded CDT"
    Write-Host ""
    Write-Host "Please install IOcomposer (preferred) or Eclipse Embedded CDT:"
    Write-Host "  1. Run: .\install_iocdevtools_win.ps1"
    Write-Host "  2. Or specify location: .\build_iosonata_lib_win.ps1 -IocomposerDir <path>"
    Write-Host "  3. Or:                  .\build_iosonata_lib_win.ps1 -EclipseDir     <path>"
    exit 1
}

Write-Host "$IDE_NAME found at: $IDE_DIR" -ForegroundColor Green
Write-Host "  Launcher: $IDE_BIN" -ForegroundColor Green
Write-Host ""

# --- Check IOsonata SDK ---
if (-not (Test-Path "$ROOT\IOsonata")) {
    Write-Host "ERROR: IOsonata not found at $ROOT\IOsonata" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please clone: .\clone_iosonata_sdk_win.ps1 -SdkHome $ROOT"
    exit 1
}

Write-Host "IOsonata SDK found at: $ROOT\IOsonata" -ForegroundColor Green
Write-Host ""

# --- Discover Projects ---
$mcuFamilies = @()
$mcuPaths = @()

Get-ChildItem -Path "$ROOT\IOsonata" -Recurse -Filter ".project" -ErrorAction SilentlyContinue | ForEach-Object {
    # Look for projects inside 'lib\Eclipse' folders
    if ($_.DirectoryName -match "\\lib\\Eclipse$") {
        $projRoot = $_.DirectoryName
        # Get relative path for display (e.g., "Nordic\nRF52840\lib\Eclipse")
        $relPath = $projRoot.Replace("$ROOT\IOsonata\", "")
        
        # Filter out non-Windows projects if any exist
        if ($relPath -match "^macOS" -or $relPath -match "^Linux") {
            return
        }
        
        $mcuFamilies += $relPath
        $mcuPaths += $projRoot
    }
}

if ($mcuFamilies.Count -eq 0) {
    Write-Host "No IOsonata Eclipse projects found." -ForegroundColor Yellow
    exit 1
}

Write-Host "Available projects:" -ForegroundColor White
Write-Host ""
for ($i = 0; $i -lt $mcuFamilies.Count; $i++) {
    Write-Host ("  {0,2}) {1}" -f ($i + 1), $mcuFamilies[$i])
}
Write-Host "   A) Build All"
Write-Host "   0) Exit"
Write-Host ""

# --- User Selection ---
do {
    $selection = Read-Host "Select project to build (0-$($mcuFamilies.Count) or A)"
    $selectionUpper = $selection.ToUpper()
} while (-not (
    ($selectionUpper -eq "A") -or
    (($selection -match '^\d+$') -and ([int]$selection -ge 0) -and ([int]$selection -le $mcuFamilies.Count))
))

if ($selectionUpper -eq "0") {
    Write-Host "Exiting."
    exit 0
}

# --- Function: Build Single Project ---
function Build-Project {
    param(
        [string]$ProjectPath,
        [string]$ProjectFamily
    )
    
    # Validate project files exist
    if (-not (Test-Path "$ProjectPath\.project") -or -not (Test-Path "$ProjectPath\.cproject")) {
        Write-Host "ERROR: Not a valid Eclipse CDT project at $ProjectPath" -ForegroundColor Red
        return $false
    }
    
    Write-Host ""
    Write-Host ">>> Building: $ProjectFamily" -ForegroundColor Cyan
    Write-Host "    Path: $ProjectPath"
    Write-Host ""
    
    # Create temporary workspace
    $WS = "$env:TEMP\iosonata_build_ws_$PID"
    New-Item -Path $WS -ItemType Directory -Force | Out-Null
    Write-Host ">>> Workspace: $WS" -ForegroundColor Gray
    Write-Host ">>> Running $IDE_NAME headless build..." -ForegroundColor Cyan
    Write-Host ""
    
    $logFile = "$env:TEMP\build_iosonata_lib_$PID.log"
    if (Test-Path $logFile) { Remove-Item $logFile -Force }
    
    # Run Headless Build
    try {
        $buildOutput = & $IDE_BIN `
            --launcher.suppressErrors `
            -nosplash `
            -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
            -data $WS `
            -no-indexer `
            -import $ProjectPath `
            -cleanBuild all `
            -printErrorMarkers `
            -vmargs "-Dorg.eclipse.equinox.p2.reconciler.dropins.directory=" `
            2>&1
        
        # Output handling
        $buildOutput | Out-File -FilePath $logFile
        $buildOutput | Write-Host
        
        # Cleanup Workspace
        Remove-Item -Path $WS -Recurse -Force -ErrorAction SilentlyContinue
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host ""
            Write-Host "Build failed for $ProjectFamily (exit code $LASTEXITCODE)" -ForegroundColor Red
            Write-Host "   Log: $logFile"
            return $false
        }
        
        Write-Host ""
        Write-Host "Build completed for $ProjectFamily" -ForegroundColor Green
        
        # Show produced libraries
        Write-Host "Libraries:" -ForegroundColor White
        Get-ChildItem "$ProjectPath\Debug\libIOsonata*.a" -ErrorAction SilentlyContinue | ForEach-Object { 
            Write-Host "  $($_.FullName) ($([math]::Round($_.Length/1KB, 1)) KB)"
        }
        Get-ChildItem "$ProjectPath\Release\libIOsonata*.a" -ErrorAction SilentlyContinue | ForEach-Object { 
            Write-Host "  $($_.FullName) ($([math]::Round($_.Length/1KB, 1)) KB)"
        }
        Write-Host ""
        
        return $true
        
    } catch {
        Write-Host ""
        Write-Host "Build exception for $ProjectFamily" -ForegroundColor Red
        Write-Host $_.Exception.Message -ForegroundColor Red
        Remove-Item -Path $WS -Recurse -Force -ErrorAction SilentlyContinue
        return $false
    }
}

# --- Execution Logic ---

if ($selectionUpper -eq "A") {
    # --- BUILD ALL ---
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "Building ALL projects ($($mcuFamilies.Count) total)" -ForegroundColor White
    Write-Host "=========================================================" -ForegroundColor Blue
    
    # Handle Ctrl-C gracefully
    $interrupted = $false
    $handler = {
        $script:interrupted = $true
        Write-Host ""
        Write-Host "!!! Build interrupted by user (Ctrl-C) !!!" -ForegroundColor Yellow
    }
    $null = Register-EngineEvent -SourceIdentifier PowerShell.Exiting -Action $handler
    
    $failedBuilds = @()
    $successfulBuilds = @()
    
    try {
        for ($i = 0; $i -lt $mcuFamilies.Count; $i++) {
            if ($interrupted) { break }
            
            Write-Host ""
            Write-Host "---------------------------------------------------------" -ForegroundColor DarkGray
            Write-Host "Building [$($i+1)/$($mcuFamilies.Count)]: $($mcuFamilies[$i])" -ForegroundColor White
            Write-Host "---------------------------------------------------------" -ForegroundColor DarkGray
            
            if (Build-Project -ProjectPath $mcuPaths[$i] -ProjectFamily $mcuFamilies[$i]) {
                $successfulBuilds += $mcuFamilies[$i]
            } else {
                $failedBuilds += $mcuFamilies[$i]
            }
        }
    } catch {
        $interrupted = $true
    } finally {
        Unregister-Event -SourceIdentifier PowerShell.Exiting -ErrorAction SilentlyContinue
    }
    
    # Summary
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    if ($interrupted) {
        Write-Host "Build All Summary (INTERRUPTED)" -ForegroundColor Yellow
    } else {
        Write-Host "Build All Summary" -ForegroundColor White
    }
    Write-Host "=========================================================" -ForegroundColor Blue
    
    Write-Host "Successful: $($successfulBuilds.Count)/$($mcuFamilies.Count)" -ForegroundColor Green
    foreach ($proj in $successfulBuilds) {
        Write-Host "   + $proj" -ForegroundColor Green
    }
    
    if ($failedBuilds.Count -gt 0) {
        Write-Host ""
        Write-Host "Failed: $($failedBuilds.Count)/$($mcuFamilies.Count)" -ForegroundColor Red
        foreach ($proj in $failedBuilds) {
            Write-Host "   - $proj" -ForegroundColor Red
        }
        Write-Host ""
        Write-Host "Check individual log files in $env:TEMP" -ForegroundColor Yellow
        exit 1
    }
    
    if ($interrupted) { exit 130 }

} else {
    # --- BUILD SINGLE ---
    $selectedIdx = [int]$selection - 1
    $selectedFamily = $mcuFamilies[$selectedIdx]
    $selectedPath = $mcuPaths[$selectedIdx]
    
    if (-not (Build-Project -ProjectPath $selectedPath -ProjectFamily $selectedFamily)) {
        exit 1
    }
}

Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "Build complete!" -ForegroundColor Green
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host ""
