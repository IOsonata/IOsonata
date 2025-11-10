#Requires -RunAsAdministrator

<#
.SYNOPSIS
    Installs IOcomposer MCU Development Tools on Windows.
.DESCRIPTION
    Installs Python, build tools, and Eclipse to Program Files, creating necessary shortcuts.
    It must be run with Administrator privileges.

    Launch : powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File "install_iocdevtools_win.ps1" %1 %2
#>
param (
    [string]$SdkHome = "$env:USERPROFILE\IOcomposer",
    [switch]$ForceUpdate,
    [switch]$Uninstall,
    [switch]$Version,
    [switch]$Help
)

$ErrorActionPreference = 'Stop'

# --- Script Configuration ---
$SCRIPT_NAME = "install_iocdevtools_windows.ps1"
$SCRIPT_VERSION = "v1.0.75-win" # Perf: Use 7-Zip for faster Eclipse extraction

# --- CLI Option Handling ---
function Show-Help {
    @"
Usage: .\$SCRIPT_NAME [OPTION]

Options:
  -Help            Show help and exit
  -Version         Show version and exit
  -SdkHome <path>  Set custom SDK installation root (default: ~\IOcomposer)
  -ForceUpdate     Force reinstall
  -Uninstall       Remove toolchains + Eclipse (keep repos/workspaces)
"@
}

if ($Help) { Show-Help; exit 0 }
if ($Version) { Write-Host "$($SCRIPT_NAME.Split('.')[0]) $SCRIPT_VERSION"; exit 0 }

$ROOT = $SdkHome
$TOOLS = "$env:ProgramFiles\xPacks"
$ECLIPSE_DIR = "$env:ProgramFiles\Eclipse Embedded CDT"
$SEVENZIP_PATH = "C:\Program Files\7-Zip\7z.exe"
$PythonExecutablePath = $null

# --- Header ---
Write-Host "==============================================" -ForegroundColor Green
Write-Host "   IOcomposer MCU Dev Tools Installer (Windows) " -ForegroundColor Green
Write-Host "   Script: $($SCRIPT_NAME.Split('.')[0])"
Write-Host "   Version: $SCRIPT_VERSION"
Write-Host "=============================================="
Write-Host

# --- Mode Detection ---
$MODE = "install"
if ($ForceUpdate) { $MODE = "force"; Write-Host ">>> Force update mode enabled" -ForegroundColor Yellow }
if ($Uninstall) { $MODE = "uninstall"; Write-Host ">>> Uninstall mode enabled" -ForegroundColor Yellow }

if ($MODE -ne "uninstall") {
    Write-Host ">>> Using SDK home folder: $ROOT"
    Write-Host ">>> Tools will be installed in: $TOOLS"
    Write-Host ">>> Eclipse will be installed in: $ECLIPSE_DIR"
}

# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
function Find-PythonExecutable {
    $pythonInPath = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonInPath -and $pythonInPath.Source -notlike "*\Microsoft\WindowsApps\*") {
        Write-Host "[OK] Found Python in PATH: $($pythonInPath.Source)" -ForegroundColor Green
        return $pythonInPath.Source
    }
    $searchPaths = @( "$env:LOCALAPPDATA\Programs\Python", "$env:ProgramFiles\Python" )
    $foundPythons = foreach ($path in $searchPaths) {
        if (Test-Path $path) {
            Get-ChildItem -Path $path -Directory -Filter "Python3*" |
            Where-Object { Test-Path (Join-Path $_.FullName "python.exe") } |
            Sort-Object -Property Name -Descending
        }
    }
    if ($foundPythons) {
        $latestPythonPath = Join-Path $foundPythons[0].FullName "python.exe"
        Write-Host "[OK] Found Python at: $latestPythonPath" -ForegroundColor Green
        return $latestPythonPath
    }
    return $null
}

function Java-Hash {
    param([string]$InputString)
    $script = 'import sys; s=sys.argv[1]; h=0; [h := (31*h + ord(c)) & 0xFFFFFFFF for c in s]; print(h - 2**32 if h >= 2**31 else h)'
    return (& $PythonExecutablePath -c $script $InputString).Trim()
}

function Get-ToolVersion { 
    param([string]$ToolPath, [string]$ToolName)
    if ([string]::IsNullOrEmpty($ToolPath)) { return "Not found" }
    $exePath = Join-Path $ToolPath "bin\$ToolName.exe"
    if (-not (Test-Path $exePath)) { return "Not found" }

    $previousErrorAction = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $rawOutput = & $exePath --version *>&1
    $ErrorActionPreference = $previousErrorAction

    $versionOutput = $rawOutput | Select-Object -First 1
    $versionMatch = [regex]::Match($versionOutput, '(\d+\.\d+\.\d+)')
    
    if ($versionMatch.Success) {
        return $versionMatch.Groups[1].Value
    } else {
        return "Unknown"
    }
}

# --- Pre-flight Checks ---
if ($MODE -ne "uninstall") {
    $PythonExecutablePath = Find-PythonExecutable
    if (-not $PythonExecutablePath) {
        Write-Host "[INFO] Python not found. Attempting to download and install directly from python.org..." -ForegroundColor Yellow
        try {
            $pythonInstallerUrl = "https://www.python.org/ftp/python/3.12.4/python-3.12.4-amd64.exe"
            $installerPath = Join-Path $env:TEMP "python-installer.exe"
            Write-Host ">>> Downloading Python installer..."
            curl.exe -L -o $installerPath $pythonInstallerUrl
            Write-Host ">>> Running Python installer silently..."
            $arguments = "/quiet InstallAllUsers=1 PrependPath=1"
            Start-Process -FilePath $installerPath -ArgumentList $arguments -Wait -PassThru
            Write-Host ">>> Verifying installation..."
            $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")
            $PythonExecutablePath = Find-PythonExecutable
            if (-not $PythonExecutablePath) { Throw "Python installation seemed to succeed, but it could not be found. Please restart your terminal and run the script again." }
            Write-Host "[OK] Python was installed successfully." -ForegroundColor Green
        } catch {
            Write-Host "[ERROR] Failed to download or install Python automatically: $_" -ForegroundColor Red
            Write-Host "[INFO] Please install Python manually from python.org, ensuring you check 'Add python.exe to PATH'." -ForegroundColor Yellow
            exit 1
        }
    }
    if (-not (Test-Path -Path $SEVENZIP_PATH)) { Write-Host "[ERROR] 7-Zip is not found at '$SEVENZIP_PATH'." -ForegroundColor Red; exit 1 }
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Write-Host "[ERROR] Git is not installed or not in your PATH." -ForegroundColor Red; exit 1 }
}

# --- Directory Setup ---
if (-not (Test-Path -Path $ROOT)) { New-Item -Path $ROOT -ItemType Directory -Force | Out-Null }
$EXT = "$ROOT\external"; if (-not (Test-Path -Path $EXT)) { New-Item -Path $EXT -ItemType Directory -Force | Out-Null }
if (-not (Test-Path -Path $TOOLS)) { New-Item -Path $TOOLS -ItemType Directory -Force | Out-Null }


# ---------------------------------------------------------
# Defender exclusion
# ---------------------------------------------------------
function Add-DefenderExclusion {
    param([string]$Path)
    
    try {
        $currentExclusions = (Get-MpPreference).ExclusionPath
        if ($currentExclusions -notcontains $Path) {
            Write-Host ">>> Adding Windows Defender exclusion for: $Path"
            Set-MpPreference -ExclusionPath $Path
            Write-Host "[OK] Folder added to Windows Defender exclusion list." -ForegroundColor Green
        } else {
            Write-Host "[OK] Windows Defender exclusion for '$Path' already exists." -ForegroundColor Green
        }
    } catch {
        Write-Host "[WARN] Could not set Windows Defender exclusion for '$Path'." -ForegroundColor Yellow
        Write-Host "[INFO] Error: $($_.Exception.Message)" -ForegroundColor Yellow
        Write-Host "[INFO] This is optional, but recommended for performance." -ForegroundColor Yellow
    }
}

# --- Directory Setup ---
if (-not (Test-Path -Path $ROOT)) { New-Item -Path $ROOT -ItemType Directory -Force | Out-Null }

# --- ADD THIS LINE ---
Add-DefenderExclusion -Path $ROOT
# --- END OF ADDED LINE ---

$EXT = "$ROOT\external"; if (-not (Test-Path -Path $EXT)) { New-Item -Path $EXT -ItemType Directory -Force | Out-Null }
if (-not (Test-Path -Path $TOOLS)) { New-Item -Path $TOOLS -ItemType Directory -Force | Out-Null }

# ---------------------------------------------------------
# UNINSTALL
# ---------------------------------------------------------
if ($MODE -eq "uninstall") {
    Write-Host "[INFO] Removing toolchains + Eclipse..." -ForegroundColor Yellow
    $response = Read-Host "Proceed? (y/N)"
    if ($response -ne 'y') { Write-Host "Uninstall cancelled."; exit 0 }
    if (Test-Path $TOOLS) { Write-Host ">>> Removing toolchains in $TOOLS..."; Remove-Item -Path $TOOLS -Recurse -Force }
    if (Test-Path $ECLIPSE_DIR) { Write-Host ">>> Removing Eclipse installation in $ECLIPSE_DIR..."; Remove-Item -Path $ECLIPSE_DIR -Recurse -Force }
    Write-Host ">>> Removing Eclipse shortcuts..."
    $desktopShortcut = Join-Path ([System.Environment]::GetFolderPath('Desktop')) "Eclipse Embedded.lnk"
    if (Test-Path $desktopShortcut) { Remove-Item $desktopShortcut -Force }
    $startMenuShortcut = Join-Path "$env:ProgramData\Microsoft\Windows\Start Menu\Programs" "Eclipse Embedded.lnk"
    if (Test-Path $startMenuShortcut) { Remove-Item $startMenuShortcut -Force }
    Write-Host ">>> Removing Eclipse user settings..."; Remove-Item -Path "$env:USERPROFILE\.p2", "$env:USERPROFILE\.eclipse" -Recurse -Force -ErrorAction SilentlyContinue
    Write-Host ">>> Removing toolchains from system PATH..."
    $oldPath = [Environment]::GetEnvironmentVariable('Path', 'Machine'); $newPath = ($oldPath.Split(';') | Where-Object { $_ -notlike "*\xPacks\*" }) -join ';'
    if ($newPath -ne $oldPath) { [Environment]::SetEnvironmentVariable('Path', $newPath, 'Machine'); Write-Host "   - PATH updated." } else { Write-Host "   - No toolchain entries found in PATH." }
    Write-Host ">>> Uninstall complete!" -ForegroundColor Green
    exit 0
}

# ---------------------------------------------------------
# Arch detect
# ---------------------------------------------------------
$ARCH = $env:PROCESSOR_ARCHITECTURE
$PLATFORM = if ($ARCH -eq "ARM64") { "win32-arm64" } else { "win32-x64" }
Write-Host ">>> Detected architecture: $ARCH"

# ---------------------------------------------------------
# Install xPack toolchain
# ---------------------------------------------------------
function Add-ToSystemPath {
    param([string]$Directory)
    $currentPath = [Environment]::GetEnvironmentVariable('Path', 'Machine'); if (-not ($currentPath -split ';' -contains $Directory)) { $newPath = "$currentPath;$Directory"; [Environment]::SetEnvironmentVariable('Path', $newPath, 'Machine'); Write-Host "[OK] Added '$Directory' to system PATH." }
}
function Install-xPack {
    param([string]$Repo, [string]$Tool, [string]$Name)
    Write-Host ">>> Checking $Name..."; $latestJson = Invoke-RestMethod -Uri "https://api.github.com/repos/xpack-dev-tools/$Repo/releases/latest"; $latestTag = $latestJson.tag_name; $latestNorm = $latestTag.TrimStart('v'); $latestBase = ($latestNorm -split '-')[0]
    $latestUrl = ($latestJson.assets | Where-Object { $_.name -like "*$PLATFORM.zip" }).browser_download_url; if (-not $latestUrl) { Write-Host "[ERROR] Could not find download URL for $Name on platform $PLATFORM." -ForegroundColor Red; return $null }
    $installedVer = ""; $toolExe = "$Tool.exe"; $installedTool = Get-Command $toolExe -ErrorAction SilentlyContinue
    if ($installedTool) { try { $versionOutput = (& $installedTool.Source --version 2>&1 | Select-Object -First 1); $installedVer = [regex]::Match($versionOutput, '(\d+\.\d+\.\d+)').Groups[1].Value } catch { $installedVer = "" } }
    $installedVerDisplay = if ([string]::IsNullOrEmpty($installedVer)) { "none" } else { $installedVer }
    Write-Host ">>> Installed: $installedVerDisplay | Latest: $latestBase"
    if ($MODE -ne "force" -and $installedVer -eq $latestBase) { Write-Host "[OK] $Name already up-to-date ($installedVer)" -ForegroundColor Green; return (Split-Path (Split-Path $installedTool.Source -Parent) -Parent) }
    
    Write-Host ">>> Installing $Name $latestNorm..."; $zipPath = Join-Path $env:TEMP (Split-Path $latestUrl -Leaf); curl.exe -L -o $zipPath $latestUrl
    
    $tmpExtractDir = Join-Path $env:TEMP "xpack-extract-$($Name.Replace(' ',''))"; if (Test-Path $tmpExtractDir) { Remove-Item $tmpExtractDir -Recurse -Force }; & $SEVENZIP_PATH x $zipPath -o"$tmpExtractDir" -y | Out-Null
    
    $extractedRoot = Get-ChildItem -Path $tmpExtractDir -Directory | Select-Object -First 1
    
    $nameParts = $extractedRoot.Name.Split('-'); if ($nameParts.Count -gt 4) { $newNameBase = $nameParts[0..($nameParts.Count-3)] -join '-'; $newFolderName = "$newNameBase-$latestBase" } else { $newFolderName = $extractedRoot.Name }
    
    $targetDir = Join-Path $TOOLS $newFolderName
    if (Test-Path $targetDir) { Remove-Item $targetDir -Recurse -Force }
    Move-Item -Path $extractedRoot.FullName -Destination $targetDir
    
    Remove-Item $zipPath, $tmpExtractDir -Recurse -Force
    Add-ToSystemPath -Directory "$targetDir\bin"
    Write-Host "[OK] $Name installed at $targetDir" -ForegroundColor Green
    return $targetDir
}

# Install all toolchains
$BUILDTOOLS_DIR = (Install-xPack "windows-build-tools-xpack" "make" "Windows Build Tools").Trim()
$ARM_DIR = (Install-xPack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC").Trim()
$RISCV_DIR = (Install-xPack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC").Trim()
$OPENOCD_DIR = (Install-xPack "openocd-xpack" "openocd" "OpenOCD").Trim()

# ---------------------------------------------------------
# Install Eclipse Embedded CDT
# ---------------------------------------------------------
Write-Host
Write-Host ">>> Installing Eclipse Embedded CDT IDE..." -ForegroundColor Cyan
$MIRROR = "https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"

# --- START: Modified Section ---
# Get the top 10 most recent release names.
# This searches older archives for the specific packages we want.
$RECENT_RELEASES = (Invoke-WebRequest -Uri "$MIRROR/" -UseBasicParsing).Content -split '\r?\n' | Where-Object { $_ -match '20[0-9]{2}-[0-9]{2}' } | ForEach-Object { ([regex]::Match($_, '20[0-9]{2}-[0-9]{2}')).Value } | Sort-Object -Descending | Get-Unique | Select-Object -First 10

if (-not $RECENT_RELEASES) { Write-Host "[ERROR] Could not detect any Eclipse release versions." -ForegroundColor Red; exit 1 }

$ECLIPSE_URL = ""
$LATEST = "" # This will be set to the version we actually find

# Loop through the recent releases until we find a valid download
# This logic now mirrors the working macOS script
foreach ($RELEASE_VERSION in $RECENT_RELEASES) {
    Write-Host ">>> Checking release: $RELEASE_VERSION..."
    
    # --- KEY CHANGE: Use the filenames from the original script & working macOS script ---
    $URL_EMBEDCDT = "$MIRROR/$RELEASE_VERSION/R/eclipse-embedcdt-$RELEASE_VERSION-R-win32-x86_64.zip"
    $URL_EMBEDCPP = "$MIRROR/$RELEASE_VERSION/R/eclipse-embedcpp-$RELEASE_VERSION-R-win32-x86_64.zip"

    try {
        # --- PRIORITY 1: Check for 'eclipse-embedcdt' (no hyphen) ---
        Write-Host "   - Checking for 'eclipse-embedcdt' package..."
        Invoke-WebRequest -Uri $URL_EMBEDCDT -Method Head -ErrorAction Stop -UseBasicParsing | Out-Null
        $ECLIPSE_URL = $URL_EMBEDCDT
        $LATEST = $RELEASE_VERSION
        Write-Host "[OK] Found 'eclipse-embedcdt' package for $LATEST." -ForegroundColor Green
        break # Found it, exit loop
    } catch {
        Write-Host "   - 'eclipse-embedcdt' not found. Checking for 'eclipse-embedcpp'..."
        try {
            # --- PRIORITY 2: Check for 'eclipse-embedcpp' ---
            Invoke-WebRequest -Uri $URL_EMBEDCPP -Method Head -ErrorAction Stop -UseBasicParsing | Out-Null
            $ECLIPSE_URL = $URL_EMBEDCPP
            $LATEST = $RELEASE_VERSION
            Write-Host "[OK] Found 'eclipse-embedcpp' package for $LATEST." -ForegroundColor Green
            break # Found it, exit loop
        } catch {
            Write-Host "   - No valid 'embed' packages found for $RELEASE_VERSION. Trying next..." -ForegroundColor Yellow
        }
    }
}

# Check if we ever found a URL after looping
if (-not $ECLIPSE_URL) {
     Write-Host "[ERROR] Could not find 'eclipse-embedcdt' or 'eclipse-embedcpp' in any of the top 10 recent Eclipse releases." -ForegroundColor Red
     Write-Host "[INFO] Please check the mirror '$MIRROR' manually." -ForegroundColor Yellow
     exit 1
}
# --- END: Modified Section ---


Write-Host ">>> Downloading Eclipse ($LATEST) from: $ECLIPSE_URL"; $zipPath = "$env:TEMP\eclipse.zip"; curl.exe -L -o $zipPath $ECLIPSE_URL
if (Test-Path $ECLIPSE_DIR) { Remove-Item $ECLIPSE_DIR -Recurse -Force }
$tmpExtractDir = Join-Path $env:TEMP "eclipse-extract"; if (Test-Path $tmpExtractDir) { Remove-Item $tmpExtractDir -Recurse -Force }

# Use 7-Zip for faster extraction
& $SEVENZIP_PATH x $zipPath -o"$tmpExtractDir" -y | Out-Null

Move-Item -Path (Join-Path $tmpExtractDir "eclipse") -Destination $ECLIPSE_DIR
Remove-Item $zipPath, $tmpExtractDir -Recurse -Force
Write-Host "[OK] Eclipse installed at $ECLIPSE_DIR" -ForegroundColor Green

# ---------------------------------------------------------
# Create Eclipse Shortcuts
# ---------------------------------------------------------
Write-Host ">>> Creating Eclipse shortcuts..."
$WshShell = New-Object -ComObject WScript.Shell
$desktopShortcutPath = Join-Path ([System.Environment]::GetFolderPath('Desktop')) "Eclipse Embedded.lnk"
$desktopShortcut = $WshShell.CreateShortcut($desktopShortcutPath); $desktopShortcut.TargetPath = Join-Path $ECLIPSE_DIR "eclipse.exe"; $desktopShortcut.Save()
Write-Host "[OK] Desktop shortcut created."
$startMenuShortcutPath = Join-Path "$env:ProgramData\Microsoft\Windows\Start Menu\Programs" "Eclipse Embedded.lnk"
$startMenuShortcut = $WshShell.CreateShortcut($startMenuShortcutPath); $startMenuShortcut.TargetPath = Join-Path $ECLIPSE_DIR "eclipse.exe"; $startMenuShortcut.Save()
Write-Host "[OK] Start Menu shortcut created."

# ---------------------------------------------------------
# Seed Eclipse prefs
# ---------------------------------------------------------
$ECLIPSE_SETTINGS = "$ECLIPSE_DIR\configuration\.settings"; New-Item -Path $ECLIPSE_SETTINGS -ItemType Directory -Force | Out-Null
$ARM_HASH = [long](Java-Hash "xPack GNU Arm Embedded GCC"); if ($ARM_HASH -lt 0) { $ARM_HASH += 4294967296 }
$RISCV_HASH = [long](Java-Hash "xPack GNU RISC-V Embedded GCC"); if ($RISCV_HASH -lt 0) { $RISCV_HASH += 4294967296 }; $RISCV_HASH += 1
$ARM_DIR_ECLIPSE = $ARM_DIR -replace '\\', '/'; $RISCV_DIR_ECLIPSE = $RISCV_DIR -replace '\\', '/'; $OPENOCD_DIR_ECLIPSE = $OPENOCD_DIR -replace '\\', '/'; $ROOT_ECLIPSE = $ROOT -replace '\\', '/'; $EXT_ECLIPSE = $EXT -replace '\\', '/'
Set-Content -Path "$ECLIPSE_SETTINGS\org.eclipse.embedcdt.core.prefs" -Value "eclipse.preferences.version=1`nxpack.arm.toolchain.path=$ARM_DIR_ECLIPSE/bin`nxpack.riscv.toolchain.path=$RISCV_DIR_ECLIPSE/bin`nxpack.openocd.path=$OPENOCD_DIR_ECLIPSE/bin`nxpack.strict=true"
Set-Content -Path "$ECLIPSE_SETTINGS\org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" -Value "toolchain.path.$ARM_HASH=$ARM_DIR_ECLIPSE/bin`ntoolchain.path.strict=true"
Set-Content -Path "$ECLIPSE_SETTINGS\org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" -Value "toolchain.path.$RISCV_HASH=$RISCV_DIR_ECLIPSE/bin`ntoolchain.path.strict=true"
Set-Content -Path "$ECLIPSE_SETTINGS\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" -Value "install.folder=$OPENOCD_DIR_ECLIPSE/bin`ninstall.folder.strict=true"
$BuildToolsBinPath = Join-Path $BUILDTOOLS_DIR "bin"; $BuildToolsPrefsPath = $BuildToolsBinPath.Replace('\', '\\'); Set-Content -Path "$ECLIPSE_SETTINGS\org.eclipse.embedcdt.managedbuild.cross.core.prefs" -Value "buildTools.path=$BuildToolsPrefsPath`neclipse.preferences.version=1"
Set-Content -Path "$ECLIPSE_SETTINGS\org.eclipse.core.runtime.prefs" -Value "eclipse.preferences.version=1`nenvironment/project/IOCOMPOSER_HOME/value=$ROOT_ECLIPSE`nenvironment/project/ARM_GCC_HOME/value=$ARM_DIR_ECLIPSE/bin`nenvironment/project/RISCV_GCC_HOME/value=$RISCV_DIR_ECLIPSE/bin`nenvironment/project/OPENOCD_HOME/value=$OPENOCD_DIR_ECLIPSE/bin`nenvironment/project/NRFX_HOME/value=$EXT_ECLIPSE/nrfx`nenvironment/project/NRFXLIB_HOME/value=$EXT_ECLIPSE/sdk-nrfxlib`nenvironment/project/NRF5_SDK_HOME/value=$EXT_ECLIPSE/nRF5_SDK`nenvironment/project/NRF5_SDK_MESH_HOME/value=$EXT_ECLIPSE/nRF5_SDK_Mesh`nenvironment/project/BSEC_HOME/value=$EXT_ECLIPSE/BSEC"
Write-Host "[OK] Eclipse preferences seeded (Build Tools, ARM, RISC-V, OpenOCD, macros)." -ForegroundColor Green

# ---------------------------------------------------------
# Clone repos
# ---------------------------------------------------------
function Sync-Repo { 
  param([string]$Url, [string]$Destination); 
  if (Test-Path $Destination) { 
    if ($MODE -eq 'force') { 
      Write-Host "   - Force-updating repo..."; Remove-Item $Destination -Recurse -Force; git clone --depth=1 $Url $Destination 
    } else { 
      Write-Host "   - Pulling latest changes..."; Push-Location $Destination; try { git pull } finally { Pop-Location } 
    } 
  } else { 
    Write-Host "   - Cloning repo..."; git clone --depth=1 $Url $Destination 
  } 
}
Sync-Repo "https://github.com/IOsonata/IOsonata.git" "$ROOT\IOsonata"; $repos = @{ 
  "https://github.com/NordicSemiconductor/nrfx.git" = "$EXT\nrfx"; 
  "https://github.com/nrfconnect/sdk-nrfxlib.git" = "$EXT\sdk-nrfxlib"; 
  "https://github.com/IOsonata/nRF5_SDK.git" = "$EXT\nRF5_SDK"; 
  "https://github.com/IOsonata/nRF5_SDK_Mesh.git" = "$EXT\nRF5_SDK_Mesh"; 
  "https://github.com/boschsensortec/Bosch-BSEC2-Library.git" = "$EXT\BSEC"; 
  "https://github.com/xioTechnologies/Fusion.git" = "$EXT\Fusion";
  "https://github.com/lvgl/lvgl.git" = "$EXT\lvgl"};
  foreach ($repo in $repos.GetEnumerator()) { 
    Sync-Repo $repo.Name $repo.Value 
  }


# ---------------------------------------------------------
# Summary
# ---------------------------------------------------------
Write-Host; Write-Host "==============================================" -ForegroundColor Green; Write-Host " Installation Summary" -ForegroundColor Green; Write-Host "==============================================" -ForegroundColor Green
$ECLIPSE_VER = "Not installed"; if (Test-Path "$ECLIPSE_DIR\eclipse.exe") { try { $ECLIPSE_VER = (Get-Item "$ECLIPSE_DIR\eclipse.exe").VersionInfo.ProductVersion } catch { $ECLIPSE_VER = "Unknown" } }
$BUILDTOOLS_VER = Get-ToolVersion $BUILDTOOLS_DIR "make"; $ARM_VER = Get-ToolVersion $ARM_DIR "arm-none-eabi-gcc"; $RISCV_VER = Get-ToolVersion $RISCV_DIR "riscv-none-elf-gcc"; $OPENOCD_VER = Get-ToolVersion $OPENOCD_DIR "openocd"
"{0,-25} {1,-15} {2}" -f "Component", "Version", "Path"; "{0,-25} {1,-15} {2}" -f "---------", "-------", "----"
"{0,-25} {1,-15} {2}" -f "Eclipse Embedded CDT:", $ECLIPSE_VER, $ECLIPSE_DIR
"{0,-25} {1,-15} {2}" -f "Windows Build Tools:", $BUILDTOOLS_VER, $BUILDTOOLS_DIR
"{0,-25} {1,-15} {2}" -f "ARM GCC:", $ARM_VER, $ARM_DIR
"{0,-25} {1,-15} {2}" -f "RISC-V GCC:", $RISCV_VER, $RISCV_DIR
"{0,-25} {1,-15} {2}" -f "OpenOCD:", $OPENOCD_VER, $OPENOCD_DIR
Write-Host "==============================================" -ForegroundColor Green; Write-Host " Installation complete!" -ForegroundColor Green; Write-Host " NOTE: A terminal restart may be required for PATH changes." -ForegroundColor Yellow; Write-Host "==============================================" -ForegroundColor Green