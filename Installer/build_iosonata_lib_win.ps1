#Requires -Version 5.1

<#
.SYNOPSIS
    Build IOsonata libraries for selected MCU target, then build
    all TaktOS ARM + RISCV libraries.

.DESCRIPTION
    Standalone script to build IOsonata libraries using a headless build,
    then automatically build every TaktOS ARM and RISCV library project.
    Lib projects only, Benchmark/KVB/test projects are skipped.
    Prefers IOcomposer (the new IOsonata installer); falls back to Eclipse
    Embedded CDT if IOcomposer is not found. IOcomposer is Eclipse-based,
    so the same headless build application is used regardless.

.PARAMETER SdkHome
    Path to SDK root (default: ~\IOcomposer)

.PARAMETER IocomposerDir
    Path to IOcomposer install dir (default: %ProgramFiles%\IOcomposer)

.PARAMETER EclipseDir
    Path to Eclipse Embedded CDT install dir, used as fallback
    (default: %ProgramFiles%\Eclipse Embedded CDT)

.PARAMETER TaktosDir
    Path to TaktOS source dir (default: <SdkHome>\TaktOS)

.PARAMETER NoTaktos
    Do not build TaktOS, IOsonata only

.EXAMPLE
    .\build_iosonata_lib_win.ps1
    Build with default paths

.EXAMPLE
    .\build_iosonata_lib_win.ps1 -IocomposerDir "C:\Tools\IOcomposer"
    Build with custom IOcomposer location

.NOTES
    Version: v2.4.0
    Platform: Windows

    v2.4.0: Build all TaktOS lib projects (ARM + RISCV) by default
            after the IOsonata build. Added -TaktosDir and -NoTaktos.
    v2.3.0: Prefer IOcomposer; fall back to Eclipse if not found.
#>

param(
    [string]$SdkHome        = "$env:USERPROFILE\IOcomposer",
    [string]$IocomposerDir  = "$env:ProgramFiles\IOcomposer",
    [string]$EclipseDir     = "$env:ProgramFiles\Eclipse Embedded CDT",
    [string]$TaktosDir      = "",
    [switch]$NoTaktos
)

$ErrorActionPreference = 'Stop'
$SCRIPT_VERSION = "v2.4.0"

# --- Helper: Print Banner ---
function Show-Banner {
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "  IOsonata + TaktOS Library Builder (Windows)" -ForegroundColor White
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

# Derive TaktOS dir from SdkHome unless set explicitly
if (-not $TaktosDir -or $TaktosDir -eq '') {
    $TaktosDir = Join-Path $ROOT "TaktOS"
}

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

# --- Discover IOsonata Projects ---
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

# --- Discover TaktOS Projects (lib only: ARM + RISCV) ---
# TaktOS library projects live at:
#   <TaktOS>\ARM\<core>\Eclipse
#   <TaktOS>\RISCV\<core>\Eclipse
# Everything under Benchmark\, KVB\ and test\ is skipped because
# those relative paths do not match the predicate.
$taktosFamilies = @()
$taktosPaths = @()

if ($NoTaktos) {
    Write-Host "TaktOS build disabled (-NoTaktos)" -ForegroundColor Yellow
    Write-Host ""
} elseif (-not (Test-Path $TaktosDir)) {
    Write-Host "WARNING: TaktOS not found at $TaktosDir" -ForegroundColor Yellow
    Write-Host "   IOsonata will still be built. Use -TaktosDir <path> to set it," -ForegroundColor Yellow
    Write-Host "   or -NoTaktos to silence this warning." -ForegroundColor Yellow
    Write-Host ""
} else {
    Get-ChildItem -Path $TaktosDir -Recurse -Filter ".project" -ErrorAction SilentlyContinue | ForEach-Object {
        $projRoot = $_.DirectoryName
        $rel = $projRoot.Replace("$TaktosDir\", "")
        # Keep only top level ARM\<core>\Eclipse or RISCV\<core>\Eclipse
        if ($rel -match '^(ARM|RISCV)\\[^\\]+\\Eclipse$') {
            $taktosFamilies += $rel
            $taktosPaths += $projRoot
        }
    }

    if ($taktosFamilies.Count -eq 0) {
        Write-Host "WARNING: No TaktOS ARM/RISCV library projects found in $TaktosDir" -ForegroundColor Yellow
        Write-Host ""
    } else {
        Write-Host "TaktOS found at: $TaktosDir ($($taktosFamilies.Count) lib projects)" -ForegroundColor Green
        Write-Host ""
    }
}

Write-Host "Available IOsonata library projects:" -ForegroundColor White
Write-Host ""
for ($i = 0; $i -lt $mcuFamilies.Count; $i++) {
    Write-Host ("  {0,2}) {1}" -f ($i + 1), $mcuFamilies[$i])
}
Write-Host "   A) Build All"
Write-Host "   0) Exit"
Write-Host ""
if ($taktosFamilies.Count -gt 0) {
    Write-Host "Note: all $($taktosFamilies.Count) TaktOS ARM/RISCV libraries are built automatically." -ForegroundColor White
    Write-Host ""
}

# --- User Selection ---
do {
    $selection = Read-Host "Select IOsonata project to build (0-$($mcuFamilies.Count) or A)"
    $selectionUpper = $selection.ToUpper()
} while (-not (
    ($selectionUpper -eq "A") -or
    (($selection -match '^\d+$') -and ([int]$selection -ge 0) -and ([int]$selection -le $mcuFamilies.Count))
))

if ($selectionUpper -ne "A" -and $selection -eq "0") {
    Write-Host "Exiting."
    exit 0
}

# --- Function: Build Single Project ---
function Build-Project {
    param(
        [string]$ProjectPath,
        [string]$ProjectFamily,
        [string]$LibGlob = "libIOsonata*.a"
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

    $logFile = "$env:TEMP\build_lib_$PID.log"
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
        Get-ChildItem "$ProjectPath\Debug\$LibGlob" -ErrorAction SilentlyContinue | ForEach-Object {
            Write-Host "  $($_.FullName) ($([math]::Round($_.Length/1KB, 1)) KB)"
        }
        Get-ChildItem "$ProjectPath\Release\$LibGlob" -ErrorAction SilentlyContinue | ForEach-Object {
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

# --- Build Work List ---
# IOsonata selection (single or all) followed by every
# TaktOS ARM/RISCV library project.
$workNames = @()
$workPaths = @()
$workGlobs = @()

if ($selectionUpper -eq "A") {
    for ($i = 0; $i -lt $mcuFamilies.Count; $i++) {
        $workNames += "IOsonata\$($mcuFamilies[$i])"
        $workPaths += $mcuPaths[$i]
        $workGlobs += "libIOsonata*.a"
    }
} else {
    $selIdx = [int]$selection - 1
    $workNames += "IOsonata\$($mcuFamilies[$selIdx])"
    $workPaths += $mcuPaths[$selIdx]
    $workGlobs += "libIOsonata*.a"
}

for ($i = 0; $i -lt $taktosFamilies.Count; $i++) {
    $workNames += "TaktOS\$($taktosFamilies[$i])"
    $workPaths += $taktosPaths[$i]
    $workGlobs += "libTaktOS*.a"
}

# --- Build Execution ---
Write-Host ""
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "Building $($workNames.Count) project(s)" -ForegroundColor White
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
    for ($i = 0; $i -lt $workNames.Count; $i++) {
        if ($interrupted) { break }

        Write-Host ""
        Write-Host "---------------------------------------------------------" -ForegroundColor DarkGray
        Write-Host "Building [$($i+1)/$($workNames.Count)]: $($workNames[$i])" -ForegroundColor White
        Write-Host "---------------------------------------------------------" -ForegroundColor DarkGray

        if (Build-Project -ProjectPath $workPaths[$i] -ProjectFamily $workNames[$i] -LibGlob $workGlobs[$i]) {
            $successfulBuilds += $workNames[$i]
        } else {
            $failedBuilds += $workNames[$i]
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
    Write-Host "Build Summary (INTERRUPTED)" -ForegroundColor Yellow
} else {
    Write-Host "Build Summary" -ForegroundColor White
}
Write-Host "=========================================================" -ForegroundColor Blue

Write-Host "Successful: $($successfulBuilds.Count)/$($workNames.Count)" -ForegroundColor Green
foreach ($proj in $successfulBuilds) {
    Write-Host "   + $proj" -ForegroundColor Green
}

if ($failedBuilds.Count -gt 0) {
    Write-Host ""
    Write-Host "Failed: $($failedBuilds.Count)/$($workNames.Count)" -ForegroundColor Red
    foreach ($proj in $failedBuilds) {
        Write-Host "   - $proj" -ForegroundColor Red
    }
    Write-Host ""
    Write-Host "Check the build log in $env:TEMP" -ForegroundColor Yellow
    exit 1
}

if ($interrupted) { exit 130 }

Write-Host ""
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host "Build complete!" -ForegroundColor Green
Write-Host "=========================================================" -ForegroundColor Blue
Write-Host ""
