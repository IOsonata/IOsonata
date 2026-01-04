#Requires -Version 5.1

<#
.SYNOPSIS
    Build IOsonata libraries for selected MCU target.

.DESCRIPTION
    Standalone script to build IOsonata libraries using Eclipse headless build.
    Can be run multiple times to build for different MCU targets.
    Now supports building all projects at once.

.PARAMETER SdkHome
    Path to IOsonata SDK root (default: ~\IOcomposer)

.EXAMPLE
    .\build_iosonata_lib_win.ps1
    Build with default path

.EXAMPLE
    .\build_iosonata_lib_win.ps1 -SdkHome C:\Dev\IOcomposer
    Build with custom path

.NOTES
    Version: v2.2.0
    Platform: Windows
#>

param(
    [string]$SdkHome = "$env:USERPROFILE\IOcomposer"
)

$ErrorActionPreference = 'Stop'
$SCRIPT_VERSION = "v2.2.0"

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

# Discover projects (filter platform-specific)
$mcuFamilies = @()
$mcuPaths = @()

Get-ChildItem -Path "$ROOT\IOsonata" -Recurse -Filter ".project" -ErrorAction SilentlyContinue | ForEach-Object {
    if ($_.DirectoryName -match "\\lib\\Eclipse$") {
        $projRoot = $_.DirectoryName
        $relPath = $projRoot.Replace("$ROOT\IOsonata\", "")
        
        # Filter out macOS/Linux specific lib projects
        if ($relPath -match "^macOS\\lib\\Eclipse$" -or 
            $relPath -match "\\macOS\\lib\\Eclipse$" -or
            $relPath -match "^Linux\\lib\\Eclipse$" -or
            $relPath -match "\\Linux\\lib\\Eclipse$") {
            return
        }
        
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
Write-Host "   A) Build All"
Write-Host "   0) Exit"
Write-Host ""

# User selection
do {
    $selection = Read-Host "Select project to build (0-$($mcuFamilies.Count) or A)"
    $selectionUpper = $selection.ToUpper()
} while (-not (
    ($selectionUpper -eq "A") -or
    (($selection -match '^\d+$') -and ([int]$selection -ge 0) -and ([int]$selection -le $mcuFamilies.Count))
))

if ($selectionUpper -eq "A") {
    # Build All selected, continue to build loop
} elseif ($selectionUpper -eq "0") {
    Write-Host "Exiting."
    exit 0
}

# Function to build a single project
function Build-Project {
    param(
        [string]$ProjectPath,
        [string]$ProjectFamily
    )
    
    # Validate
    if (-not (Test-Path "$ProjectPath\.project") -or -not (Test-Path "$ProjectPath\.cproject")) {
        Write-Host "❌ ERROR: Not a valid Eclipse CDT project" -ForegroundColor Red
        return $false
    }
    
    Write-Host ""
    Write-Host ">>> Building: $ProjectFamily" -ForegroundColor Cyan
    Write-Host "    Path: $ProjectPath"
    Write-Host ""
    
    # Create workspace
    $WS = "$env:TEMP\iosonata_build_ws_$PID"
    New-Item -Path $WS -ItemType Directory -Force | Out-Null
    Write-Host ">>> Workspace: $WS" -ForegroundColor Gray
    Write-Host ">>> Running Eclipse headless build..." -ForegroundColor Cyan
    Write-Host ""
    
    $eclipseExe = "$ECLIPSE_DIR\eclipse.exe"
    $logFile = "$env:TEMP\build_iosonata_lib_$PID.log"
    if (Test-Path $logFile) { Remove-Item $logFile -Force }
    
    # Build
    try {
        $buildOutput = & $eclipseExe `
            --launcher.suppressErrors `
            -nosplash `
            -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
            -data $WS `
            -no-indexer `
            -import $ProjectPath `
            -cleanBuild all `
            -printErrorMarkers `
            2>&1
        
        $buildOutput | Out-File -FilePath $logFile
        $buildOutput | Write-Host
        
        Remove-Item -Path $WS -Recurse -Force -ErrorAction SilentlyContinue
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host ""
            Write-Host "❌ Build failed for $ProjectFamily (exit code $LASTEXITCODE)" -ForegroundColor Red
            Write-Host "   Log: $logFile"
            return $false
        }
        
        Write-Host ""
        Write-Host "✅ Build completed for $ProjectFamily" -ForegroundColor Green
        Write-Host ""
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
        Write-Host "❌ Build exception for $ProjectFamily" -ForegroundColor Red
        Write-Host $_.Exception.Message -ForegroundColor Red
        Remove-Item -Path $WS -Recurse -Force -ErrorAction SilentlyContinue
        return $false
    }
}

# Build All or Single
if ($selectionUpper -eq "A") {
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "Building ALL projects ($($mcuFamilies.Count) total)" -ForegroundColor White
    Write-Host "=========================================================" -ForegroundColor Blue
    
    # Set up Ctrl-C handler
    $interrupted = $false
    $handler = {
        $script:interrupted = $true
        Write-Host ""
        Write-Host "=========================================================" -ForegroundColor Yellow
        Write-Host "Build interrupted by user (Ctrl-C)" -ForegroundColor Yellow
        Write-Host "=========================================================" -ForegroundColor Yellow
    }
    
    # Register the event handler
    $null = Register-EngineEvent -SourceIdentifier PowerShell.Exiting -Action $handler
    
    $failedBuilds = @()
    $successfulBuilds = @()
    
    try {
        for ($i = 0; $i -lt $mcuFamilies.Count; $i++) {
            # Check if interrupted
            if ($interrupted) {
                break
            }
            
            Write-Host ""
            Write-Host "─────────────────────────────────────────────────────────" -ForegroundColor DarkGray
            Write-Host "Building [$($i+1)/$($mcuFamilies.Count)]: $($mcuFamilies[$i])" -ForegroundColor White
            Write-Host "─────────────────────────────────────────────────────────" -ForegroundColor DarkGray
            
            if (Build-Project -ProjectPath $mcuPaths[$i] -ProjectFamily $mcuFamilies[$i]) {
                $successfulBuilds += $mcuFamilies[$i]
            } else {
                $failedBuilds += $mcuFamilies[$i]
            }
        }
    } catch {
        # Catch Ctrl-C or other interruptions
        $interrupted = $true
    } finally {
        # Unregister the event handler
        Unregister-Event -SourceIdentifier PowerShell.Exiting -ErrorAction SilentlyContinue
    }
    
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    if ($interrupted) {
        Write-Host "Build All Summary (INTERRUPTED)" -ForegroundColor Yellow
    } else {
        Write-Host "Build All Summary" -ForegroundColor White
    }
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "✅ Successful: $($successfulBuilds.Count)/$($mcuFamilies.Count)" -ForegroundColor Green
    foreach ($proj in $successfulBuilds) {
        Write-Host "   ✓ $proj" -ForegroundColor Green
    }
    
    if ($failedBuilds.Count -gt 0) {
        Write-Host ""
        Write-Host "❌ Failed: $($failedBuilds.Count)/$($mcuFamilies.Count)" -ForegroundColor Red
        foreach ($proj in $failedBuilds) {
            Write-Host "   ✗ $proj" -ForegroundColor Red
        }
        Write-Host ""
        Write-Host "Check individual log files in $env:TEMP" -ForegroundColor Yellow
    }
    
    if ($interrupted) {
        Write-Host ""
        Write-Host "Build process was interrupted by user." -ForegroundColor Yellow
        exit 130  # Standard exit code for Ctrl-C
    }
    
    if ($failedBuilds.Count -gt 0) {
        exit 1
    }
    
} else {
    # Build single project
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
