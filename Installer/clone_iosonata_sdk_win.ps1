<#
=========================================================
 clone_iosonata_sdk_win.ps1
---------------------------------------------------------
 Purpose: Clone IOsonata SDK and dependencies for Windows
 Platform: Windows PowerShell 5+ / PowerShell 7+
=========================================================
#>

# --- Default values ---
$homePath = "$env:USERPROFILE\IOcomposer"
$mode = "normal"
$help = $false

# --- Parse CLI-style arguments (like macOS/Linux) ---
for ($i = 0; $i -lt $args.Count; $i++) {
    switch ($args[$i]) {
        '--home' {
            if ($i + 1 -lt $args.Count) {
                $homePath = $args[$i + 1]
                $i++
            } else {
                Write-Host "Missing value for --home" -ForegroundColor Red
                exit 1
            }
        }
        '--mode' {
            if ($i + 1 -lt $args.Count) {
                $mode = $args[$i + 1]
                $i++
            } else {
                Write-Host "Missing value for --mode" -ForegroundColor Red
                exit 1
            }
        }
        '--help' {
            $help = $true
        }
        default {
            Write-Host " Unknown argument: $($args[$i])" -ForegroundColor Red
            Write-Host "Use '--help' for usage information." -ForegroundColor Yellow
            exit 1
        }
    }
}

# --- Functions ---
function Show-Banner {
    Write-Host ""
    Write-Host "=========================================================" -ForegroundColor Blue
    Write-Host "      IOsonata SDK Cloning Utility for Windows 🪟" -ForegroundColor Cyan
    Write-Host "=========================================================" -ForegroundColor Blue
}

function Show-Help {
    Show-Banner
    Write-Host "Usage: clone_iosonata_sdk_win.ps1 [--home <path>] [--mode <normal|force>] [--help]" -ForegroundColor White
    Write-Host ""
    Write-Host "Options:" -ForegroundColor Yellow
    Write-Host "  --home <path>     Set custom root directory (default: $env:USERPROFILE\IOcomposer)"
    Write-Host "  --mode <mode>     Clone mode: 'normal' (default) or 'force'"
    Write-Host "  --help            Show this help message and exit"
    Write-Host ""
    Write-Host "Examples:" -ForegroundColor Yellow
    Write-Host "  ./clone_iosonata_sdk_win.ps1"
    Write-Host "  ./clone_iosonata_sdk_win.ps1 --home 'D:\Projects\IOcomposer'"
    Write-Host "  ./clone_iosonata_sdk_win.ps1 --mode force"
    Write-Host ""
    Write-Host "Repositories cloned (during normal operation):"
    Write-Host "  - IOsonata main repository"
    Write-Host "  - Nordic Semiconductor nrfx"
    Write-Host "  - nrfconnect SDKs (sdk-nrf-bm, sdk-nrfxlib)"
    Write-Host "  - IOsonata nRF5 SDK & Mesh"
    Write-Host "  - Bosch BSEC2 library"
    Write-Host ""
    exit
}

if ($help) { Show-Help }

# --- Prepare directories safely ---
if (!(Test-Path -LiteralPath $homePath)) {
    New-Item -ItemType Directory -Force -Path $homePath | Out-Null
}
$homePath = (Resolve-Path -LiteralPath $homePath).Path
$externalPath = Join-Path $homePath "external"
if (!(Test-Path -LiteralPath $externalPath)) {
    New-Item -ItemType Directory -Force -Path $externalPath | Out-Null
}

Show-Banner
Write-Host "Root directory : $homePath" -ForegroundColor White
Write-Host "Mode           : $mode" -ForegroundColor White
Write-Host "External path  : $externalPath" -ForegroundColor White
Write-Host "---------------------------------------------------------" -ForegroundColor Blue
Write-Host ""

# --- Function: Clone or update repo ---
function Clone-Or-UpdateRepo([string]$RepoUrl, [string]$TargetDir) {
    if (Test-Path -LiteralPath $TargetDir) {
        if ($mode -eq "force") {
            Write-Host "  Re-cloning $TargetDir (force mode)..." -ForegroundColor Yellow
            Remove-Item -Recurse -Force -LiteralPath $TargetDir
            git clone --depth=1 "$RepoUrl" "$TargetDir"
        } else {
            Write-Host " Updating $TargetDir..." -ForegroundColor Yellow
            Push-Location -LiteralPath $TargetDir
            git pull --rebase
            Pop-Location
        }
    } else {
        Write-Host "  Cloning $TargetDir..." -ForegroundColor Cyan
        git clone --depth=1 "$RepoUrl" "$TargetDir"
    }
}

# ---------------------------------------------------------
# Clone IOsonata
# ---------------------------------------------------------
Write-Host " Cloning IOsonata..." -ForegroundColor Green
$IOsonataDir = Join-Path $homePath "IOsonata"
Clone-Or-UpdateRepo "https://github.com/IOsonata/IOsonata.git" "$IOsonataDir"

# ---------------------------------------------------------
# Clone External Repos
# ---------------------------------------------------------
Write-Host ""
Write-Host " Cloning dependencies into: $externalPath" -ForegroundColor Blue
Write-Host ""

$Repos = @(
    "https://github.com/NordicSemiconductor/nrfx.git",
    "https://github.com/nrfconnect/sdk-nrf-bm.git",
    "https://github.com/nrfconnect/sdk-nrfxlib.git",
    "https://github.com/IOsonata/nRF5_SDK.git",
    "https://github.com/IOsonata/nRF5_SDK_Mesh.git",
    "https://github.com/boschsensortec/Bosch-BSEC2-Library.git"
	"https://github.com/xioTechnologies/Fusion.git"
    "https://github.com/lvgl/lvgl.git"
)

foreach ($Repo in $Repos) {
    $Name = [System.IO.Path]::GetFileNameWithoutExtension($Repo)
    if ($Name -eq "Bosch-BSEC2-Library") { $Name = "BSEC" }
    $Target = Join-Path $externalPath $Name
    Clone-Or-UpdateRepo "$Repo" "$Target"
}

Write-Host ""
Write-Host " All repositories cloned successfully!" -ForegroundColor Green
Write-Host "SDK Root      : $homePath" -ForegroundColor White
Write-Host "External Repos: $externalPath" -ForegroundColor White
Write-Host "---------------------------------------------------------" -ForegroundColor Blue
Write-Host "Done. Happy building" -ForegroundColor Green
Write-Host ""
