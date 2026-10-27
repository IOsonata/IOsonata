#Requires -RunAsAdministrator

<#
.SYNOPSIS
    Installs IOcomposer MCU Development Tools on Windows.
.DESCRIPTION
    Installs Python, 7-Zip, build tools, and IOcomposer to Program Files.
    Refined for parity with macOS preference seeding (path hashing).
#>
param (
    [string]$SdkHome = "$env:USERPROFILE\IOcomposer",
	[string]$IocomposerDir = "$env:ProgramFiles\IOcomposer",
	[switch]$WithLLVM,
    [switch]$ForceUpdate,
    [switch]$Uninstall,
    [switch]$Version,
    [switch]$Help
)

$ErrorActionPreference = 'Stop'
$SCRIPT_NAME = "install_iocdevtools_win.ps1"
$SCRIPT_VERSION = "v1.1.0-win"

function Show-Help {
@"
Usage: .\$SCRIPT_NAME [OPTION]

Options:
  -WithLLVM        Install optional LLVM compiler (failure does not stop setup)
  -Help            Show help and exit
  -Version         Show version and exit
  -SdkHome <path>  Set custom SDK installation root
  -IocomposerDir <path> Set custom IOcomposer installation path
  -ForceUpdate     Force reinstall
  -Uninstall       Remove toolchains + IOcomposer
"@
}

if ($Help) { Show-Help; exit 0 }
if ($Version) { Write-Host "$($SCRIPT_NAME.Split('.')[0]) $SCRIPT_VERSION"; exit 0 }

$ROOT = $SdkHome
$PYOCD_DIR = "$ROOT\Tools\Python"
$TOOL_BIN = "$ROOT\Tools\bin"
$PYOCD = "$PYOCD_DIR\Scripts\pyocd.exe"
$ManagedPython = "$PYOCD_DIR\Scripts\python.exe"
$LLVM_DIR = "$ROOT\Tools\LLVM"
$TOOLS = "$env:ProgramFiles\xPacks"
$ECLIPSE_DIR = $IocomposerDir

$SEVENZIP_EXE = $null
$PythonExecutablePath = $null

# --- Header ---
Write-Host "==============================================" -ForegroundColor Green
Write-Host "   IOcomposer MCU Dev Tools Installer (Windows) " -ForegroundColor Green
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
}

# --- Helpers ---

function Refresh-EnvVars {
    $machine = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $user    = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machine;$user"
}

function Test-Python39 {
	param([string]$Executable)
	if (-not $Executable -or -not (Test-Path $Executable)) { return $false }
	try {
		& $Executable -c 'import sys; sys.exit(sys.version_info < (3, 9))'
		return ($LASTEXITCODE -eq 0)
	} catch {
		return $false
	}
}

function Find-PythonExecutable {
	Write-Host "   [DEBUG] Searching for Python..." -ForegroundColor DarkGray

	$inPath = Get-Command python -ErrorAction SilentlyContinue
	if ($inPath -and $inPath.Source -notlike "*\Microsoft\WindowsApps\*" -and (Test-Python39 $inPath.Source)) {
		return $inPath.Source
	}

	$pyLauncher = Get-Command py -ErrorAction SilentlyContinue
	if ($pyLauncher) {
		try {
			$path = (& $pyLauncher.Source -c "import sys; print(sys.executable)") 2>$null
			if (Test-Python39 $path) { return $path }
		} catch {}
	}

	$regPaths = @("HKLM:\SOFTWARE\Python\PythonCore", "HKCU:\SOFTWARE\Python\PythonCore")
	foreach ($hkey in $regPaths) {
		if (Test-Path $hkey) {
			$versions = Get-ChildItem $hkey -ErrorAction SilentlyContinue | Sort-Object Name -Descending
			foreach ($v in $versions) {
				$installPathKey = Join-Path $v.PSPath "InstallPath"
				if (Test-Path $installPathKey) {
					$p = (Get-ItemProperty $installPathKey).'(default)'
					$exe = Join-Path $p "python.exe"
					if (Test-Python39 $exe) { return $exe }
				}
			}
		}
	}
	return $null
}

function Find-7Zip {
    Write-Host "   [DEBUG] Searching for 7-Zip..." -ForegroundColor DarkGray
    $regKeys = @("HKLM:\SOFTWARE\7-Zip", "HKLM:\SOFTWARE\WOW6432Node\7-Zip")
    foreach ($key in $regKeys) {
        if (Test-Path $key) {
            $path = (Get-ItemProperty $key -Name "Path" -ErrorAction SilentlyContinue).Path
            if ($path) { $exe = Join-Path $path "7z.exe"; if (Test-Path $exe) { return $exe } }
        }
    }
    $candidates = @("$env:ProgramFiles\7-Zip\7z.exe", "${env:ProgramFiles(x86)}\7-Zip\7z.exe", "C:\Program Files\7-Zip\7z.exe")
    foreach ($c in $candidates) { if (Test-Path $c) { return $c } }
    return $null
}

function Java-Hash {
    param([string]$InputString)
    # Python script mirrors the exact Java String.hashCode() logic used by macOS script
    # Returns signed 32-bit integer (can be negative) - DO NOT adjust afterward
    $script = @'
import sys
s = sys.argv[1]
h = 0
for c in s:
    h = (31 * h + ord(c)) & 0xFFFFFFFF
if h >= 2**31:
    h -= 2**32
print(h)
'@
    return (& $PythonExecutablePath -c $script $InputString).Trim()
}

function Set-ContentNoBom {
    param([string]$Path, [string]$Value)
    # Write UTF-8 without Byte Order Mark (BOM) compatible with Eclipse
    $enc = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $Value, $enc)
}

# --- Pre-flight Checks ---
if ($MODE -ne "uninstall") {
	if (Test-Python39 $ManagedPython) {
		$PythonExecutablePath = $ManagedPython
	}
	if (-not $PythonExecutablePath) {
		$PythonExecutablePath = Find-PythonExecutable
	}
	if (-not $PythonExecutablePath) {
		Write-Host "[INFO] Python 3.9 or newer not found. Installing..." -ForegroundColor Yellow
		$pyStage = Join-Path $env:TEMP (".python-stage-" + [guid]::NewGuid().ToString('N'))
		New-Item $pyStage -ItemType Directory | Out-Null
		try {
			$pyArch = if ($env:PROCESSOR_ARCHITECTURE -eq "ARM64") { "arm64" } else { "amd64" }
			$url = "https://www.python.org/ftp/python/3.12.4/python-3.12.4-$pyArch.exe"
			$inst = Join-Path $pyStage "python-installer.exe"
			curl.exe -fL -o $inst $url
			if ($LASTEXITCODE -ne 0 -or -not (Test-Path $inst) -or (Get-Item $inst).Length -le 0) {
				throw "Python installer download failed."
			}
			$process = Start-Process -FilePath $inst -ArgumentList "/quiet InstallAllUsers=1 PrependPath=1" -Wait -PassThru
			if ($process.ExitCode -ne 0) { throw "Python installer failed: $($process.ExitCode)" }
			Refresh-EnvVars
			$PythonExecutablePath = Find-PythonExecutable
			if (-not $PythonExecutablePath) { throw "Python 3.9 or newer was not found after installation." }
		} catch {
			Write-Host "[ERROR] $_" -ForegroundColor Red
			exit 1
		} finally {
			Remove-Item $pyStage -Recurse -Force -ErrorAction SilentlyContinue
		}
	}

	$SEVENZIP_EXE = Find-7Zip
	if (-not $SEVENZIP_EXE) {
		Write-Host "[INFO] 7-Zip not found. Installing..." -ForegroundColor Yellow
		$sevenZipStage = Join-Path $env:TEMP (".7zip-stage-" + [guid]::NewGuid().ToString('N'))
		New-Item $sevenZipStage -ItemType Directory | Out-Null
		try {
			$7zArch = if ($env:PROCESSOR_ARCHITECTURE -eq "ARM64") { "arm64" } else { "x64" }
			$7zUrl = "https://www.7-zip.org/a/7z2407-$7zArch.exe"
			$7zInst = Join-Path $sevenZipStage "7z-installer.exe"
			curl.exe -fL -o $7zInst $7zUrl
			if ($LASTEXITCODE -ne 0 -or -not (Test-Path $7zInst) -or (Get-Item $7zInst).Length -le 0) {
				throw "7-Zip installer download failed."
			}
			$process = Start-Process -FilePath $7zInst -ArgumentList "/S" -Wait -PassThru
			if ($process.ExitCode -ne 0) { throw "7-Zip installer failed: $($process.ExitCode)" }
			$SEVENZIP_EXE = Find-7Zip
			if (-not $SEVENZIP_EXE) {
				$def = "C:\Program Files\7-Zip\7z.exe"
				if (Test-Path $def) { $SEVENZIP_EXE = $def } else { throw "7-Zip was not found after installation." }
			}
		} catch {
			Write-Host "[ERROR] $_" -ForegroundColor Red
			exit 1
		} finally {
			Remove-Item $sevenZipStage -Recurse -Force -ErrorAction SilentlyContinue
		}
	}

    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Write-Host "[ERROR] Git not found. Please install Git for Windows." -ForegroundColor Red; exit 1 }
}

# --- Directory Setup ---
if (-not (Test-Path $ROOT)) { New-Item -Path $ROOT -ItemType Directory -Force | Out-Null }
$EXT = "$ROOT\external"; if (-not (Test-Path $EXT)) { New-Item -Path $EXT -ItemType Directory -Force | Out-Null }
if (-not (Test-Path $TOOLS)) { New-Item -Path $TOOLS -ItemType Directory -Force | Out-Null }

function Add-DefenderExclusion {
    param([string]$Path)
    try {
        if ((Get-MpPreference).ExclusionPath -notcontains $Path) {
            Set-MpPreference -ExclusionPath $Path
            Write-Host "   [OK] Added Defender exclusion: $Path" -ForegroundColor Green
        }
    } catch { Write-Host "[WARN] Failed to set Defender exclusion (requires Admin)." -ForegroundColor Yellow }
}
Add-DefenderExclusion -Path $ROOT

# --- Uninstall ---
if ($MODE -eq "uninstall") {
    $ans = Read-Host "Remove toolchains + IOcomposer? (y/N)"
    if ($ans -ne 'y') { exit 0 }
    
    # Remove xPacks toolchains
    if (Test-Path $TOOLS) {
        Write-Host ">>> Removing toolchains from $TOOLS..."
        try {
            # Try to stop any processes that might be locking files
            Get-Process | Where-Object { $_.Path -like "$TOOLS*" } | Stop-Process -Force -ErrorAction SilentlyContinue
            Start-Sleep -Milliseconds 500
            Remove-Item $TOOLS -Recurse -Force -ErrorAction Stop
            Write-Host "   [OK] Toolchains removed." -ForegroundColor Green
        } catch {
            Write-Host "   [WARN] Could not fully remove $TOOLS - some files may be in use." -ForegroundColor Yellow
            Write-Host "   Try closing any applications using these tools and run uninstall again." -ForegroundColor Yellow
        }
    }
    
    # Remove Eclipse
    if (Test-Path $ECLIPSE_DIR) {
        Write-Host ">>> Removing IOcomposer from $ECLIPSE_DIR..."
        try {
            Get-Process eclipse -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq "$ECLIPSE_DIR\eclipse.exe" } | Stop-Process -Force -ErrorAction SilentlyContinue
            Start-Sleep -Milliseconds 500
            Remove-Item $ECLIPSE_DIR -Recurse -Force -ErrorAction Stop
            Write-Host "   [OK] IOcomposer removed." -ForegroundColor Green
        } catch {
            Write-Host "   [WARN] Could not fully remove IOcomposer - it may be running." -ForegroundColor Yellow
        }
    }
    
	# Preserve shared profiles belonging to other CDT installations.
	Remove-Item "$([Environment]::GetFolderPath('Desktop'))\IOcomposer.lnk", "$env:ProgramData\Microsoft\Windows\Start Menu\Programs\IOcomposer.lnk" -Force -ErrorAction SilentlyContinue
    
	Remove-Item $PYOCD_DIR -Recurse -Force -ErrorAction SilentlyContinue
	if (Test-Path "$LLVM_DIR\.iocomposer-managed") {
		Remove-Item $LLVM_DIR -Recurse -Force -ErrorAction SilentlyContinue
	}
    # Remove IDAP tools
    Write-Host ">>> Removing IDAP tools..."
    $IDAP_DIR = "$TOOL_BIN"
    if (Test-Path $IDAP_DIR) {
        Remove-Item "$IDAP_DIR\IDAPnRFProg.exe", "$IDAP_DIR\IDAPSetTarget.exe" -Force -ErrorAction SilentlyContinue
        Write-Host "   [OK] IDAP tools removed." -ForegroundColor Green
    }
    
    # Prompt for IOsonata and external SDK removal
    $ans2 = Read-Host "Also remove IOsonata and external SDK folders? (y/N)"
    if ($ans2 -eq 'y') {
        if (Test-Path "$ROOT\IOsonata") { 
            Remove-Item "$ROOT\IOsonata" -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "   [OK] IOsonata removed." -ForegroundColor Green
        }
        if (Test-Path "$ROOT\external") { 
            Remove-Item "$ROOT\external" -Recurse -Force -ErrorAction SilentlyContinue 
            Write-Host "   [OK] External SDK removed." -ForegroundColor Green
        }
        if (Test-Path "$ROOT\.iocomposer") { 
            Remove-Item "$ROOT\.iocomposer" -Recurse -Force -ErrorAction SilentlyContinue 
            Write-Host "   [OK] .iocomposer removed." -ForegroundColor Green
        }
    } else {
        Write-Host ">>> Repositories under $ROOT and workspace dirs were kept."
    }
    
    # If the ROOT folder is now empty, remove it (otherwise keep it).
    if (Test-Path $ROOT) {
        $items = Get-ChildItem $ROOT -Force -ErrorAction SilentlyContinue
        if ($items.Count -eq 0) {
            Remove-Item $ROOT -Force -ErrorAction SilentlyContinue
            Write-Host "   [OK] Removed empty root folder: $ROOT" -ForegroundColor Green
        } else {
            Write-Host "   [INFO] Keeping root folder (not empty): $ROOT" -ForegroundColor Yellow
        }
    }
    
    Write-Host ">>> Uninstall complete!" -ForegroundColor Green
    exit 0
}

# --- Arch Detection ---
$ARCH = $env:PROCESSOR_ARCHITECTURE
$XPACK_PLATFORM = if ($ARCH -eq "ARM64") { "win32-arm64" } else { "win32-x64" }
$ECLIPSE_ARCH = if ($ARCH -eq "ARM64") { "aarch64" } else { "x86_64" }
Write-Host ">>> Detected architecture: $ARCH -> Eclipse arch=$ECLIPSE_ARCH"

# --- Install xPack ---
function Add-ToSystemPath {
    param([string]$Dir)
    $curr = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    if ($curr -split ';' -notcontains $Dir) {
        [Environment]::SetEnvironmentVariable('Path', "$curr;$Dir", 'Machine')
        Write-Host "   [OK] Added to PATH: $Dir"
    }
}

function Install-xPack {
	param([string]$Repo, [string]$Tool, [string]$Name)
	Write-Host ">>> Checking $Name..."
	try {
		$json = Invoke-RestMethod -Uri "https://api.github.com/repos/xpack-dev-tools/$Repo/releases/latest"
	} catch {
		throw "Failed to query GitHub API for $Name. $_"
	}
	if (-not $json.tag_name) { throw "GitHub returned no release tag for $Name." }
	$tag = $json.tag_name
	$ver = $tag.TrimStart('v')
	$base = ($ver -split '-')[0]

	$toolExe = Get-Command "$Tool.exe" -ErrorAction SilentlyContinue
	if ($toolExe) {
		$currVerOutput = try { (& $toolExe.Source --version 2>&1) | Out-String -Stream | Select-Object -First 1 } catch { "" }
		if ($currVerOutput -match '(\d+\.\d+\.\d+)') {
			$currVer = $Matches[1]
			if ($currVer -eq $base -and $MODE -ne "force") {
				Write-Host "   [OK] $Name up-to-date ($currVer)." -ForegroundColor Green
				return (Split-Path (Split-Path $toolExe.Source -Parent) -Parent)
			}
		}
	}

	$url = ($json.assets | Where-Object { $_.name -like "*$XPACK_PLATFORM.zip" } |
		Select-Object -First 1).browser_download_url
	if (-not $url) { throw "No download URL found for $Name ($XPACK_PLATFORM)." }

	Write-Host ">>> Installing $Name $ver..."
	$stage = Join-Path $env:TEMP (".xp-stage-" + [guid]::NewGuid().ToString('N'))
	$backup = $null
	$target = $null
	New-Item $stage -ItemType Directory | Out-Null
	try {
		$zip = Join-Path $stage "archive.zip"
		$unpacked = Join-Path $stage "unpacked"
		New-Item $unpacked -ItemType Directory | Out-Null

		curl.exe -fL -o $zip $url
		if ($LASTEXITCODE -ne 0) { throw "$Name download failed." }

		& $SEVENZIP_EXE x $zip -o"$unpacked" -y | Out-Null
		if ($LASTEXITCODE -ne 0) { throw "$Name extraction failed." }

		$root = Get-ChildItem $unpacked -Directory | Select-Object -First 1
		if (-not $root) { throw "$Name archive contains no tool directory." }
		$stagedTool = Join-Path $root.FullName "bin\$Tool.exe"
		if (-not (Test-Path $stagedTool) -or (Get-Item $stagedTool).Length -le 0) {
			throw "$Name archive is missing bin\$Tool.exe."
		}

		$target = Join-Path $TOOLS $root.Name
		if (Test-Path $target) {
			$backup = "$target.backup-$([guid]::NewGuid().ToString('N'))"
			Move-Item $target $backup
		}
		try {
			Move-Item $root.FullName $target
			Add-ToSystemPath "$target\bin"
		} catch {
			Remove-Item $target -Recurse -Force -ErrorAction SilentlyContinue
			if ($backup -and (Test-Path $backup)) {
				Move-Item $backup $target
				$backup = $null
			}
			throw
		}

		if ($backup -and (Test-Path $backup)) {
			Remove-Item $backup -Recurse -Force
			$backup = $null
		}
		Write-Host "   [OK] Installed at $target" -ForegroundColor Green
		return $target
	} finally {
		Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
	}
}

$BUILDTOOLS_DIR = Install-xPack "windows-build-tools-xpack" "make" "Windows Build Tools"
function Test-LLVM {
	param([string]$Directory)
	foreach ($tool in @('clang', 'clang++', 'ld.lld', 'llvm-ar', 'llvm-objcopy', 'llvm-size')) {
		if (-not (Test-Path "$Directory\bin\$tool.exe")) { throw "Missing LLVM tool: $tool" }
	}
	$stage = Join-Path $env:TEMP ([guid]::NewGuid().ToString('N'))
	New-Item $stage -ItemType Directory | Out-Null
	try {
		Set-Content "$stage\probe.c" 'int probe(void) { return 7; }' -Encoding ASCII
		Set-Content "$stage\probe.cpp" 'extern "C" int probe(void) { return 7; }' -Encoding ASCII
		foreach ($target in @('arm-none-eabi', 'riscv32-unknown-elf')) {
			$flags = if ($target -eq 'arm-none-eabi') { @('-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft') } else { @('-march=rv32imac', '-mabi=ilp32') }
			foreach ($language in @('c', 'cpp')) {
				$compiler = if ($language -eq 'c') { 'clang' } else { 'clang++' }
				& "$Directory\bin\$compiler.exe" "--target=$target" @flags -ffreestanding -fno-exceptions -fno-rtti -nostdlib "--ld-path=$Directory\bin\ld.lld.exe" '-Wl,-e,probe' "$stage\probe.$language" -o "$stage\$target-$language.elf"
				if ($LASTEXITCODE -ne 0) { throw "LLVM $target $language compile/link check failed." }
				& "$Directory\bin\llvm-objcopy.exe" -O ihex "$stage\$target-$language.elf" "$stage\$target-$language.hex"
				if ($LASTEXITCODE -ne 0) { throw "LLVM $target HEX output check failed." }
			}
		}
		& "$Directory\bin\clang.exe" --version
		if ($LASTEXITCODE -ne 0) { throw 'LLVM version check failed.' }
	} finally { Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue }
}

function Install-LLVM {
	if (Test-Path "$LLVM_DIR\bin\clang.exe") { Test-LLVM $LLVM_DIR; return }
	if (Test-Path $LLVM_DIR) { throw "LLVM destination already exists; inspect before retrying: $LLVM_DIR" }
	$arch = $env:PROCESSOR_ARCHITECTURE
	if ($env:PROCESSOR_ARCHITEW6432) { $arch = $env:PROCESSOR_ARCHITEW6432 }
	switch ($arch) {
		'AMD64' { $package = 'win64'; $hash = '15e49aca3540991a6f28a0e57babab67da82155c4f73f79dd5e8666a851f9127' }
		'Arm64' { $package = 'woa64'; $hash = 'b484f54264400f0a75b63704b1068da8086f0b14625575f6efea08813109b068' }
		default { throw "No LLVM installer configured for Windows $arch." }
	}
	$stage = Join-Path $env:TEMP ([guid]::NewGuid().ToString('N'))
	New-Item $stage -ItemType Directory | Out-Null
	try {
		$archive = Join-Path $stage 'llvm.msi'
		$url = "https://github.com/llvm/llvm-project/releases/download/llvmorg-23.1.4/LLVM-23.1.4-$package.msi"
		Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $archive
		if ((Get-FileHash $archive -Algorithm SHA256).Hash -ne $hash) { throw 'LLVM checksum mismatch.' }
		# Administrative extraction provides a portable tree, without PATH changes.
		$unpacked = Join-Path $stage 'unpacked'
		$process = Start-Process msiexec.exe -ArgumentList @('/a', "`"$archive`"", '/qn', "TARGETDIR=`"$unpacked`"") -Wait -PassThru
		if ($process.ExitCode -ne 0) { throw "LLVM extraction failed: $($process.ExitCode)" }
		$distribution = Join-Path $unpacked 'LLVM'
		Test-LLVM $distribution
		New-Item (Join-Path $distribution '.iocomposer-managed') -ItemType File | Out-Null
		New-Item (Split-Path $LLVM_DIR -Parent) -ItemType Directory -Force | Out-Null
		Move-Item $distribution $LLVM_DIR
	} finally { Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue }
}

function Install-PythonTools {
	if (Test-Path $ManagedPython) {
		& $ManagedPython -c 'import sys; sys.exit(sys.version_info < (3, 9))'
		if ($LASTEXITCODE -ne 0) { throw "Managed Python 3.9 or newer is required for pyOCD: $ManagedPython" }
	} else {
		& $PythonExecutablePath -c 'import sys; sys.exit(sys.version_info < (3, 9))'
		if ($LASTEXITCODE -ne 0) { throw 'Python 3.9 or newer is required to create the Tools Python environment.' }
		& $PythonExecutablePath -m venv $PYOCD_DIR
		if ($LASTEXITCODE -ne 0) { throw 'Failed to create the Tools Python environment.' }
	}
	$PythonExecutablePath = $ManagedPython
	& $ManagedPython -m pip install --upgrade pyocd intelhex==2.3.0
	if ($LASTEXITCODE -ne 0) { throw 'pyOCD/IntelHex installation failed.' }
	& $ManagedPython -c 'from intelhex import IntelHex'
	if ($LASTEXITCODE -ne 0) { throw 'IntelHex verification failed.' }
	New-Item $TOOL_BIN -ItemType Directory -Force | Out-Null
	foreach ($tool in @('python', 'pyocd', 'hexmerge')) {
		$exe = if ($tool -eq 'hexmerge') { 'hexmerge.py' } elseif ($tool -eq 'python') { 'python.exe' } else { 'pyocd.exe' }
		$command = '@echo off' + [Environment]::NewLine + '"' + (Join-Path $PYOCD_DIR "Scripts\$exe") + '" %*' + [Environment]::NewLine
		[IO.File]::WriteAllText((Join-Path $TOOL_BIN "$tool.cmd"), $command, [Text.Encoding]::ASCII)
	}
	& $PYOCD --version
	if ($LASTEXITCODE -ne 0) { throw 'pyOCD verification failed.' }
}


Install-PythonTools
$LLVM_STATUS = 'skipped (optional; use -WithLLVM)'
if ($WithLLVM) {
	try {
		Install-LLVM
		$LLVM_STATUS = "$LLVM_DIR (verified Arm and RISC-V)"
	} catch {
		$LLVM_STATUS = 'unavailable (optional installation failed)'
		Write-Warning "LLVM installation failed; continuing with GCC and remaining tools. $_"
	}
}
$env:Path = "$TOOL_BIN;$env:Path"

$ARM_DIR = Install-xPack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC"
$RISCV_DIR = Install-xPack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC"
$OPENOCD_DIR = Install-xPack "openocd-xpack" "openocd" "OpenOCD"

# Validate critical tools were installed
if (-not $ARM_DIR) { Write-Host "[ERROR] ARM GCC installation failed" -ForegroundColor Red; exit 1 }
if (-not $RISCV_DIR) { Write-Host "[ERROR] RISC-V GCC installation failed" -ForegroundColor Red; exit 1 }
if (-not $OPENOCD_DIR) { Write-Host "[ERROR] OpenOCD installation failed" -ForegroundColor Red; exit 1 }

# --- Install Eclipse ---
Write-Host; Write-Host ">>> Checking Eclipse..." -ForegroundColor Cyan
$MIRROR = "https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"

# Enumerate available releases from mirror
Write-Host "   Probing for latest Eclipse release..." -NoNewline
$ECLIPSE_URL = ""
$LATEST = ""

try {
    $releasePage = Invoke-WebRequest -Uri "$MIRROR/" -UseBasicParsing -TimeoutSec 30
    $releases = [regex]::Matches($releasePage.Content, 'href="(20\d{2}-\d{2})/"') | 
                ForEach-Object { $_.Groups[1].Value } | 
                Sort-Object -Descending -Unique
    
    foreach ($rel in $releases) {
        if ($ECLIPSE_URL) { break }
        # Use architecture-appropriate URL
        $U1 = "$MIRROR/$rel/R/eclipse-embedcdt-$rel-R-win32-$ECLIPSE_ARCH.zip"
        $U2 = "$MIRROR/$rel/R/eclipse-embedcpp-$rel-R-win32-$ECLIPSE_ARCH.zip"
        
        try {
            $resp = Invoke-WebRequest $U1 -Method Head -UseBasicParsing -TimeoutSec 10
            if ($resp.StatusCode -eq 200) { $ECLIPSE_URL = $U1; $LATEST = $rel; break }
        } catch {}
        try {
            $resp = Invoke-WebRequest $U2 -Method Head -UseBasicParsing -TimeoutSec 10
            if ($resp.StatusCode -eq 200) { $ECLIPSE_URL = $U2; $LATEST = $rel; break }
        } catch {}
    }
} catch {
    Write-Host " Failed to enumerate releases." -ForegroundColor Yellow
}

# Fallback to year/month probing if enumeration failed
if (-not $ECLIPSE_URL) {
    $years = 0..3 | ForEach-Object { (Get-Date).Year - $_ }
    $months = "12", "09", "06", "03"
    foreach ($y in $years) {
        if ($ECLIPSE_URL) { break }
        foreach ($m in $months) {
            $REL = "$y-$m"
            $U1 = "$MIRROR/$REL/R/eclipse-embedcdt-$REL-R-win32-$ECLIPSE_ARCH.zip"
            $U2 = "$MIRROR/$REL/R/eclipse-embedcpp-$REL-R-win32-$ECLIPSE_ARCH.zip"
            try { if ((Invoke-WebRequest $U1 -Method Head -UseBasicParsing -EA SilentlyContinue).StatusCode -eq 200) { $ECLIPSE_URL=$U1; $LATEST=$REL; break } } catch {}
            try { if ((Invoke-WebRequest $U2 -Method Head -UseBasicParsing -EA SilentlyContinue).StatusCode -eq 200) { $ECLIPSE_URL=$U2; $LATEST=$REL; break } } catch {}
        }
    }
}
Write-Host " Done."

if (-not $ECLIPSE_URL) { Write-Host "`n[ERROR] No Eclipse download found for architecture $ECLIPSE_ARCH." -ForegroundColor Red; exit 1 }

if ($MODE -eq "force" -or -not (Test-Path "$ECLIPSE_DIR\eclipse.exe")) {
	Write-Host ">>> Installing IOcomposer runtime (Eclipse $LATEST)..."
	$stage = Join-Path $env:TEMP (".eclipse-stage-" + [guid]::NewGuid().ToString('N'))
	$backup = "$ECLIPSE_DIR.backup-$([guid]::NewGuid().ToString('N'))"
	New-Item $stage -ItemType Directory | Out-Null
	try {
		$zip = Join-Path $stage "eclipse.zip"
		$tmp = Join-Path $stage "unpacked"
		curl.exe -fL -o $zip $ECLIPSE_URL
		if ($LASTEXITCODE -ne 0) { throw "IOcomposer runtime download failed" }
		New-Item $tmp -ItemType Directory | Out-Null
		& $SEVENZIP_EXE x $zip -o"$tmp" -y | Out-Null
		if ($LASTEXITCODE -ne 0 -or -not (Test-Path "$tmp\eclipse\eclipse.exe")) {
			throw "IOcomposer runtime extraction failed"
		}
		if (Test-Path $ECLIPSE_DIR) {
			Move-Item $ECLIPSE_DIR $backup
		}
		try {
			Move-Item "$tmp\eclipse" $ECLIPSE_DIR
		} catch {
			Remove-Item $ECLIPSE_DIR -Recurse -Force -ErrorAction SilentlyContinue
			if (Test-Path $backup) { Move-Item $backup $ECLIPSE_DIR }
			throw
		}
		Remove-Item $backup -Recurse -Force -ErrorAction SilentlyContinue
		Write-Host "   [OK] Installed at $ECLIPSE_DIR" -ForegroundColor Green
	} finally {
		Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
	}
} else { 
    Write-Host "   [OK] IOcomposer runtime already installed (use -ForceUpdate to reinstall)." -ForegroundColor Green 
}

# --- Eclipse Shortcuts ---
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\IOcomposer.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()
$sc = $WshShell.CreateShortcut("$env:ProgramData\Microsoft\Windows\Start Menu\Programs\IOcomposer.lnk")
$sc.TargetPath = "$ECLIPSE_DIR\eclipse.exe"; $sc.Save()

# --- Seed Prefs to BOTH locations (required for Eclipse version compatibility) ---
# Location 1: Eclipse installation directory (works immediately)
# Location 2: User directory ~/.eclipse (required by some Eclipse versions)

# IMPORTANT: Windows paths must be forward-slashed for Eclipse prefs
$AD = $ARM_DIR -replace '\\','/'
$RD = $RISCV_DIR -replace '\\','/'
$OD = $OPENOCD_DIR -replace '\\','/'
$RT = $ROOT -replace '\\','/'

# Build tools path (may be null if not installed)
$BT = ""
if ($BUILDTOOLS_DIR) {
    $BT = (Join-Path $BUILDTOOLS_DIR "bin") -replace '\\','/'
}

# Hash calculation - use toolchain NAME strings like macOS does (not paths)
# The Java-Hash function returns signed 32-bit integer
# Convert negative to unsigned for Eclipse prefs
$ARM_HASH_RAW = [long](Java-Hash "xPack GNU Arm Embedded GCC")
if ($ARM_HASH_RAW -lt 0) { $ARM_HASH_RAW = $ARM_HASH_RAW + 4294967296 }
$AH = $ARM_HASH_RAW

$RISCV_HASH_RAW = [long](Java-Hash "xPack GNU RISC-V Embedded GCC")
if ($RISCV_HASH_RAW -lt 0) { $RISCV_HASH_RAW = $RISCV_HASH_RAW + 4294967296 }
$RH = $RISCV_HASH_RAW + 1  # +1 matches macOS behavior

# Function to write ALL preference files to installation directory
function Write-EclipseInstallPrefs {
    param([string]$SettingsDir)
    
    New-Item $SettingsDir -ItemType Directory -Force | Out-Null

    # 1. org.eclipse.core.runtime.prefs (with environment variables like macOS)
    $runtime_prefs = @"
eclipse.preferences.version=1
environment/project/LLVM_HOME/value=$RT/Tools/LLVM
environment/project/ARM_LLVM_HOME/value=$RT/Tools/LLVM
environment/project/RISCV_LLVM_HOME/value=$RT/Tools/LLVM
environment/project/IOCOMPOSER_HOME/value=$RT
environment/project/PATH/value=$RT/Tools/bin;$RT/Tools/bin
environment/project/PATH/operation=prepend
environment/project/PATH/delimiter=;
environment/project/ARM_GCC_HOME/value=$AD/bin
environment/project/RISCV_GCC_HOME/value=$RD/bin
environment/project/OPENOCD_HOME/value=$OD/bin
"@
    Set-ContentNoBom -Path "$SettingsDir\org.eclipse.core.runtime.prefs" -Value $runtime_prefs

    # 2. org.eclipse.cdt.core.prefs
    $cdt_prefs = @"
eclipse.preferences.version=1
environment/buildEnvironmentInclude=true
org.eclipse.cdt.core.parser.taskTags=TODO,FIXME,XXX
"@
    Set-Content "$SettingsDir\org.eclipse.cdt.core.prefs" $cdt_prefs -Encoding UTF8

    # 3. org.eclipse.embedcdt.core.prefs (xPack paths)
    $embed_prefs = @"
eclipse.preferences.version=1
xpack.arm.toolchain.path=$AD
xpack.riscv.toolchain.path=$RD
xpack.openocd.path=$OD/bin
xpack.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.core.prefs" $embed_prefs -Encoding UTF8

    # 4a. org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs (ARM-specific)
    $arm_prefs = @"
eclipse.preferences.version=1
toolchain.path.$AH=$AD
toolchain.path.1287942917=$AD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" $arm_prefs -Encoding UTF8

    # 4b. org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs (RISC-V-specific)
    $riscv_prefs = @"
eclipse.preferences.version=1
toolchain.path.$RH=$RD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" $riscv_prefs -Encoding UTF8

    # 5. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
    $ocd_prefs = @"
eclipse.preferences.version=1
executable=openocd.exe
install.folder=$OD/bin
install.folder.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" $ocd_prefs -Encoding UTF8

    # 7. org.eclipse.embedcdt.managedbuild.cross.core.prefs (Windows specific: needs Build Tools for make)
    if ($BT) {
        $cross_prefs = @"
eclipse.preferences.version=1
buildTools.path=$BT
"@
        Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.core.prefs" $cross_prefs -Encoding UTF8
    }
}

# Function to write user prefs to user directory (matching macOS behavior)
function Write-EclipseUserPrefs {
    param([string]$SettingsDir)
    
    New-Item $SettingsDir -ItemType Directory -Force | Out-Null

    # User prefs files (matching macOS behavior)

    # 1. org.eclipse.embedcdt.core.prefs
    $embed_prefs = @"
eclipse.preferences.version=1
xpack.arm.toolchain.path=$AD
xpack.riscv.toolchain.path=$RD
xpack.openocd.path=$OD/bin
xpack.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.core.prefs" $embed_prefs -Encoding UTF8

    # 2. org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs
    $arm_prefs = @"
eclipse.preferences.version=1
toolchain.path.$AH=$AD
toolchain.path.1287942917=$AD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" $arm_prefs -Encoding UTF8

    # 3. org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs
    $riscv_prefs = @"
eclipse.preferences.version=1
toolchain.path.$RH=$RD
toolchain.path.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" $riscv_prefs -Encoding UTF8

    # 4. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
    $ocd_prefs = @"
eclipse.preferences.version=1
executable=openocd.exe
install.folder=$OD/bin
install.folder.strict=true
"@
    Set-Content "$SettingsDir\org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" $ocd_prefs -Encoding UTF8

    # 5. org.eclipse.core.runtime.prefs
    $runtime_prefs = @"
eclipse.preferences.version=1
environment/project/LLVM_HOME/value=$RT/Tools/LLVM
environment/project/ARM_LLVM_HOME/value=$RT/Tools/LLVM
environment/project/RISCV_LLVM_HOME/value=$RT/Tools/LLVM
environment/project/IOCOMPOSER_HOME/value=$RT
environment/project/PATH/value=$RT/Tools/bin;$RT/Tools/bin
environment/project/PATH/operation=prepend
environment/project/PATH/delimiter=;
environment/project/ARM_GCC_HOME/value=$AD/bin
environment/project/RISCV_GCC_HOME/value=$RD/bin
environment/project/OPENOCD_HOME/value=$OD/bin
environment/project/NRFX_HOME/value=$RT/external/nrfx
environment/project/NRFXLIB_HOME/value=$RT/external/sdk-nrfxlib
environment/project/NRF5_SDK_HOME/value=$RT/external/nRF5_SDK
environment/project/NRF5_SDK_MESH_HOME/value=$RT/external/nRF5_SDK_Mesh
environment/project/BSEC_HOME/value=$RT/external/BSEC
"@
    Set-ContentNoBom -Path "$SettingsDir\org.eclipse.core.runtime.prefs" -Value $runtime_prefs
}

# --- Location 1: Eclipse Installation Directory ---
Write-Host; Write-Host ">>> Seeding Eclipse MCU preferences (installation directory)..." -ForegroundColor Cyan
$INSTALL_SET = "$ECLIPSE_DIR\configuration\.settings"
Write-EclipseInstallPrefs $INSTALL_SET
Write-Host "   [OK] Preferences seeded in $INSTALL_SET" -ForegroundColor Green

# --- Location 2: User Directory (~/.eclipse) ---
# First, initialize Eclipse to create the user configuration directory (like macOS)
Write-Host; Write-Host ">>> Initializing Eclipse to create instance configuration..." -ForegroundColor Cyan
$eclipseExe = "$ECLIPSE_DIR\eclipse.exe"
if (Test-Path $eclipseExe) {
    try {
        $initProcess = Start-Process -FilePath $eclipseExe -ArgumentList "-nosplash", "-initialize" -Wait -PassThru -WindowStyle Hidden
        if ($initProcess.ExitCode -eq 0) {
            Write-Host "   [OK] Eclipse initialized." -ForegroundColor Green
        } else {
            Write-Host "   [WARN] Eclipse initialization returned exit code $($initProcess.ExitCode)" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "   [WARN] Eclipse initialization failed: $_" -ForegroundColor Yellow
    }
}

Write-Host; Write-Host ">>> Seeding Eclipse MCU preferences (user directory)..." -ForegroundColor Cyan
$USER_ECLIPSE = "$env:USERPROFILE\.eclipse"

# Find existing Eclipse platform configuration directories
$userConfigs = @()
if (Test-Path $USER_ECLIPSE) {
    $userConfigs = Get-ChildItem $USER_ECLIPSE -Directory -Filter "org.eclipse.platform_*" -ErrorAction SilentlyContinue |
                   Sort-Object Name -Descending
}

if ($userConfigs.Count -gt 0) {
    foreach ($cfg in $userConfigs) {
        $userSet = Join-Path $cfg.FullName "configuration\.settings"
        Write-EclipseUserPrefs $userSet
        Write-Host "   [OK] Preferences seeded in $userSet" -ForegroundColor Green
    }
} else {
    Write-Host "   [INFO] User Eclipse config not found (Eclipse not yet run)." -ForegroundColor Yellow
    Write-Host "   [INFO] Installation-level prefs are set; user prefs will be created on next run." -ForegroundColor Yellow
}

# --- eclipse.ini and Path Variables ---
Write-Host; Write-Host ">>> Configuring eclipse.ini and Path Variables..." -ForegroundColor Cyan
$INI = "$ECLIPSE_DIR\eclipse.ini"
$CUSTOM_INI = "$ECLIPSE_DIR\plugin_customization.ini"
$CUSTOM_INI_JAVA = $CUSTOM_INI -replace '\\', '/'

# Backup before modifying
Copy-Item $INI "$INI.bak" -Force

Write-Host "   ⚙️  Generating plugin_customization.ini for Workspace Path Variables..." -ForegroundColor Cyan

# 1. Create plugin_customization.ini content
$customIniContent = @"
org.eclipse.core.resources/pathvariable.IOSONATA_LOC=$RT
org.eclipse.core.resources/pathvariable.IOCOMPOSER_HOME=$RT
"@
Set-Content -Path $CUSTOM_INI -Value $customIniContent

# Read eclipse.ini content
$TXT = Get-Content $INI

# 2. Remove old properties and hooks if they exist
$TXT = $TXT | Where-Object { 
    $_ -notmatch '^-Diosonata\.home=' -and 
    $_ -notmatch '^-Diosonata_loc=' -and 
    $_ -notmatch '^-Diocomposer_home=' -and
    $_ -notmatch '^-pluginCustomization' -and
    $_ -notmatch 'plugin_customization\.ini'
}

$IDX = [array]::IndexOf($TXT, '-vmargs')

# 3. Inject new properties
if ($IDX -ge 0) {
    # Insert BEFORE and AFTER -vmargs
    $before = if ($IDX -gt 0) { $TXT[0..($IDX - 1)] } else { @() }
    $after = if ($IDX -lt $TXT.Count - 1) { $TXT[($IDX+1)..($TXT.Count-1)] } else { @() }
    
    $NEW = $before + "-pluginCustomization" + "plugin_customization.ini" + "-vmargs" + "-Diosonata_loc=$RT" + "-Diocomposer_home=$RT" + $after
} else {
    # No -vmargs section, add it at the end
    $NEW = $TXT + "-pluginCustomization" + "plugin_customization.ini" + "-vmargs" + "-Diosonata_loc=$RT" + "-Diocomposer_home=$RT"
}

Set-Content $INI $NEW
Write-Host "   [OK] eclipse.ini and Path Variables configured:" -ForegroundColor Green
Write-Host "        iosonata_loc = $RT" -ForegroundColor White
Write-Host "        iocomposer_home = $RT" -ForegroundColor White
Write-Host "        IOSONATA_LOC = $RT" -ForegroundColor White

# IOcomposer setup belongs to this installer, not the website launcher.
function Install-IOcomposer {
	param([string]$Runtime)
	$stage = Join-Path $env:TEMP ([guid]::NewGuid().ToString('N'))
	New-Item $stage -ItemType Directory | Out-Null
	try {
		$plugins = Invoke-RestMethod -Uri 'https://api.github.com/repos/I-SYST/iocomposer.io/contents/plugin?ref=main' -Headers @{
			Accept = 'application/vnd.github+json'
			'User-Agent' = 'iocomposer-installer'
		}
		$ids = @('com.iocomposer.embedcdt.ai', 'com.iocomposer.embedcdt.ui')
		foreach ($id in $ids) {
			$url = $null
			if ($id -eq 'com.iocomposer.embedcdt.ai') { $url = $env:IOCOMPOSER_AI_PLUGIN_URL }
			if (-not $url) {
				$candidates = foreach ($item in $plugins) {
					if ($item.name -match ('^' + [regex]::Escape($id) + '_([0-9]+(?:\.[0-9]+)*)\.jar$')) {
						$key = ($Matches[1].Split('.') | ForEach-Object { $_.PadLeft(10, '0') }) -join '.'
						[pscustomobject]@{ Key = $key; Name = $item.name }
					}
				}
				$latest = $candidates | Sort-Object Key -Descending | Select-Object -First 1
				if (-not $latest) { throw "No published IOcomposer plugin found: $id" }
				$url = "https://raw.githubusercontent.com/I-SYST/iocomposer.io/main/plugin/$($latest.Name)"
			}
			$jar = Join-Path $stage "$id.jar"
			Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $jar
			& $SEVENZIP_EXE t $jar | Out-Null
			if ($LASTEXITCODE -ne 0) { throw "Invalid IOcomposer plugin archive: $id" }
		}
		# Stage both required plugins before replacing either installed plugin.
		$dropins = Join-Path $Runtime 'dropins'
		New-Item $dropins -ItemType Directory -Force | Out-Null
		foreach ($id in $ids) {
			Copy-Item (Join-Path $stage "$id.jar") (Join-Path $dropins "$id.jar") -Force
			Get-ChildItem $dropins -Filter "${id}_*.jar" | Remove-Item -Force
		}
		$ini = Join-Path $Runtime 'eclipse.ini'
		$lines = Get-Content $ini
		$out = [System.Collections.Generic.List[string]]::new()
		$skip = $false
		foreach ($line in $lines) {
			if ($skip) { $skip = $false; continue }
			if ($line -in @('-name', '-pluginCustomization')) { $skip = $true; continue }
			$out.Add($line)
		}
		if (-not $out.Contains('-vmargs')) { $out.Add('-vmargs') }
		$index = $out.IndexOf('-vmargs')
		$out.InsertRange($index, [string[]]@('-name', 'IOcomposer', '-pluginCustomization', 'plugin_customization.ini'))
		[IO.File]::WriteAllLines($ini, $out, [Text.UTF8Encoding]::new($false))
		$custom = Join-Path $Runtime 'plugin_customization.ini'
		$preferences = [ordered]@{
			'org.eclipse.ui/showIntro' = 'false'
			'org.eclipse.ui/defaultPerspectiveId' = 'com.iocomposer.embedcdt.ui.perspective'
			'org.eclipse.epp.package.embedcpp/showNewsOnStartup' = 'false'
			'org.eclipse.epp.package.embedcpp.ui/showNewsOnStartup' = 'false'
			'org.eclipse.epp.package.cpp/showNewsOnStartup' = 'false'
			'org.eclipse.epp.package.common/showNewsOnStartup' = 'false'
			'org.eclipse.epp.mpc.ui/showNewsOnStartup' = 'false'
		}
		$existing = @(Get-Content $custom | Where-Object { -not $preferences.Contains(($_ -split '=', 2)[0]) })
		foreach ($key in $preferences.Keys) { $existing += "$key=$($preferences[$key])" }
		[IO.File]::WriteAllLines($custom, [string[]]$existing, [Text.UTF8Encoding]::new($false))
		try {
			$splash = Join-Path $stage 'splash.bmp'
			Invoke-WebRequest -UseBasicParsing -Uri 'https://raw.githubusercontent.com/I-SYST/iocomposer.io/main/plugin/splash.bmp' -OutFile $splash
			Copy-Item $splash (Join-Path $Runtime 'splash.bmp') -Force
			Get-ChildItem (Join-Path $Runtime 'plugins') -Recurse -Filter splash.bmp | ForEach-Object {
				Copy-Item $splash $_.FullName -Force
			}
		} catch { Write-Warning "IOcomposer splash installation failed: $_" }
		Write-Host "IOcomposer AI and UI plugins installed at $Runtime"
	} finally {
		Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
	}
}

function Build-IOcomposerIndex {
	$index = Join-Path $ROOT 'IOsonata\Installer\build_external_index.py'
	if ($PythonExecutablePath -and (Test-Path $index)) {
		& "$PYOCD_DIR\Scripts\python.exe" $index --sdk-root "$ROOT\external"
		if ($LASTEXITCODE -ne 0) { Write-Warning "SDK index failed. Retry: python $index --sdk-root $ROOT\external" }
	} else { Write-Warning "Python or SDK index script missing; index was not built." }
}

# --- Plugin ---
function Install-Plugin {
    Write-Host; Write-Host ">>> Installing IOsonata Eclipse Plugin..." -ForegroundColor Cyan
    $PDIR = "$ROOT\IOsonata\Installer\eclipse_plugin"
    if (-not (Test-Path $PDIR)) { 
        Write-Host "   [WARN] Plugin directory not found at $PDIR" -ForegroundColor Yellow
        return 
    }
    $JAR = Get-ChildItem $PDIR "org.iosonata.embedcdt.templates.wizard_*.jar" -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1
    if ($JAR) {
        $DEST = "$ECLIPSE_DIR\dropins"
        New-Item $DEST -ItemType Directory -Force | Out-Null
        # Remove old versions
        Get-ChildItem "$DEST\org.iosonata.embedcdt.templates.wizard_*.jar" -ErrorAction SilentlyContinue | Remove-Item -Force
        Copy-Item $JAR.FullName $DEST -Force
        Write-Host "   [OK] Plugin installed: $($JAR.Name)" -ForegroundColor Green
    } else {
        Write-Host "   [WARN] No plugin jar found in $PDIR" -ForegroundColor Yellow
    }
}

# ---------------------------------------------------------
# Sync IOsonata + external SDKs (single source of truth)
# ---------------------------------------------------------
Write-Host; Write-Host ">>> Syncing IOsonata + external SDKs..." -ForegroundColor Cyan

$installerRef = $env:IOSONATA_INSTALLER_REF
if (-not $installerRef) { $installerRef = "main" }
$CLONE_URL = "https://raw.githubusercontent.com/IOsonata/IOsonata/$installerRef/Installer/clone_iosonata_sdk_win.ps1"
$TMP_CLONE = Join-Path $env:TEMP "clone_iosonata_sdk_win.ps1"
$CLONE_LOG = Join-Path $env:TEMP "clone_iosonata_sdk_win.log"

try {
    curl.exe -fsSL $CLONE_URL -o $TMP_CLONE
    if ($LASTEXITCODE -ne 0) { throw "Download failed" }
} catch {
    Write-Host "[ERROR] Failed to download clone script:" -ForegroundColor Red
    Write-Host "   $CLONE_URL" -ForegroundColor Red
    exit 1
}

# Prefer -NoBuild when supported. Capture output to detect whether the builder ran.
$CLONE_INVOKED_BUILD = $false

# First, check if the clone script supports -NoBuild by examining its content
$cloneScriptContent = Get-Content $TMP_CLONE -Raw -ErrorAction SilentlyContinue
$supportsNoBuild = $cloneScriptContent -match '\$NoBuild|\-NoBuild'

$cloneArgs = @("-SdkHome", $ROOT)
if ($supportsNoBuild) {
    $cloneArgs += "-NoBuild"
}
if ($MODE -eq "force") {
    $cloneArgs += @("-Mode", "force")
}

# Run clone script and capture output
$ErrorActionPreference = 'Continue'
$cloneOutput = & powershell.exe -ExecutionPolicy Bypass -File $TMP_CLONE @cloneArgs 2>&1 | Tee-Object -FilePath $CLONE_LOG
$cloneExitCode = $LASTEXITCODE
$cloneOutput | Write-Host
$ErrorActionPreference = 'Stop'

if ($cloneExitCode -ne 0) {
	throw "SDK synchronization failed with exit code $cloneExitCode. See $CLONE_LOG"
}

Remove-Item $TMP_CLONE -Force -ErrorAction SilentlyContinue

# Detect whether the clone script already invoked the IOsonata library builder.
if (Test-Path $CLONE_LOG) {
    $logContent = Get-Content $CLONE_LOG -Raw -ErrorAction SilentlyContinue
    if ($logContent -match "IOsonata Library (Auto-)?Build|IOsonata Library Builder|Available IOsonata library projects|Select project to build") {
        $CLONE_INVOKED_BUILD = $true
    }
    Remove-Item $CLONE_LOG -Force -ErrorAction SilentlyContinue
}

# --- Install IDAP Tools ---
$IDAP_DIR = "$TOOL_BIN"
$IDAP_PROG = "$IDAP_DIR\IDAPnRFProg.exe"
$IDAP_PARENT = Split-Path $IDAP_DIR -Parent
New-Item -Path $IDAP_PARENT -ItemType Directory -Force | Out-Null

if (-not (Test-Path $IDAP_PROG) -or $MODE -eq "force") {
	Write-Host
	Write-Host ">>> Downloading IDAPnRFProg tool..."
	$IDAP_URL = "https://sourceforge.net/projects/idaplinkfirmware/files/Windows/IDAPnRFProg_Win_2_1_240807.zip/download"
	$IDAP_STAGE = Join-Path $IDAP_PARENT (".idap-stage-" + [guid]::NewGuid().ToString('N'))
	$IDAP_BACKUP = Join-Path $IDAP_PARENT (".idap-backup-" + [guid]::NewGuid().ToString('N'))
	New-Item $IDAP_STAGE -ItemType Directory | Out-Null

	try {
		$archive = Join-Path $IDAP_STAGE "archive.zip"
		curl.exe -fL $IDAP_URL -o $archive --silent --show-error
		if ($LASTEXITCODE -ne 0) { throw "IDAPnRFProg download failed." }

		$candidate = Join-Path $IDAP_STAGE "candidate"
		New-Item $candidate -ItemType Directory | Out-Null
		& $SEVENZIP_EXE e $archive -o"$candidate" -y | Out-Null
		if ($LASTEXITCODE -ne 0) { throw "IDAPnRFProg extraction failed." }
		$stagedProgram = Join-Path $candidate "IDAPnRFProg.exe"
		if (-not (Test-Path $stagedProgram) -or (Get-Item $stagedProgram).Length -le 0) {
			throw "Invalid IDAPnRFProg archive: missing or empty IDAPnRFProg.exe."
		}
		foreach ($tool in @('IDAPnRFProg.exe', 'IDAPSetTarget.exe')) {
			$stagedTool = Join-Path $candidate $tool
			if (Test-Path $stagedTool) { Copy-Item $stagedTool $IDAP_DIR -Force }
		}
		Write-Host "   [OK] IDAPnRFProg installed at: $IDAP_PROG" -ForegroundColor Green
	} finally {
		Remove-Item $IDAP_STAGE -Recurse -Force -ErrorAction SilentlyContinue
	}
} else {
	Write-Host "   [OK] IDAPnRFProg already installed (skipping)" -ForegroundColor Green
}

Install-Plugin
Install-IOcomposer -Runtime $ECLIPSE_DIR

# --- Makefile (aligned with macOS/Linux) ---
Write-Host; Write-Host ">>> Generating makefile_path.mk..." -ForegroundColor Cyan
$MK = "$ROOT\IOsonata\makefile_path.mk"
$MK_CONTENT = @"
# makefile_path.mk
# Auto-generated by install_iocdevtools_win.ps1 $SCRIPT_VERSION
# This file contains all path macros required to compile IOsonata projects using Makefiles
# Include this file in your project Makefile: include `$(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# Toolchain Paths
# ============================================
LLVM_ROOT = $RT/Tools/LLVM
ARM_LLVM_ROOT = $RT/Tools/LLVM
RISCV_LLVM_ROOT = $RT/Tools/LLVM
CLANG = $RT/Tools/LLVM/bin/clang.exe
CLANGXX = $RT/Tools/LLVM/bin/clang++.exe
LLVM_LD = $RT/Tools/LLVM/bin/ld.lld.exe
LLVM_AR = $RT/Tools/LLVM/bin/llvm-ar.exe
LLVM_OBJCOPY = $RT/Tools/LLVM/bin/llvm-objcopy.exe
ARM_LLVM_TARGET = --target=arm-none-eabi
RISCV_LLVM_TARGET = --target=riscv32-unknown-elf
SDK_TOOLS_ROOT = $RT/Tools
PYTHON = $RT/Tools/bin/python.cmd
HEXMERGE = $RT/Tools/bin/hexmerge.cmd
PYOCD_ROOT = $RT/Tools/Python
PYOCD = $RT/Tools/bin/pyocd.cmd
ARM_GCC_ROOT = $AD
ARM_GCC_BIN = $AD/bin
ARM_GCC = `$(ARM_GCC_BIN)/arm-none-eabi-gcc
ARM_GPP = `$(ARM_GCC_BIN)/arm-none-eabi-g++
ARM_AS = `$(ARM_GCC_BIN)/arm-none-eabi-as
ARM_LD = `$(ARM_GCC_BIN)/arm-none-eabi-ld
ARM_AR = `$(ARM_GCC_BIN)/arm-none-eabi-ar
ARM_OBJCOPY = `$(ARM_GCC_BIN)/arm-none-eabi-objcopy
ARM_OBJDUMP = `$(ARM_GCC_BIN)/arm-none-eabi-objdump
ARM_SIZE = `$(ARM_GCC_BIN)/arm-none-eabi-size
ARM_GDB = `$(ARM_GCC_BIN)/arm-none-eabi-gdb

RISCV_GCC_ROOT = $RD
RISCV_GCC_BIN = $RD/bin
RISCV_GCC = `$(RISCV_GCC_BIN)/riscv-none-elf-gcc
RISCV_GPP = `$(RISCV_GCC_BIN)/riscv-none-elf-g++
RISCV_AS = `$(RISCV_GCC_BIN)/riscv-none-elf-as
RISCV_LD = `$(RISCV_GCC_BIN)/riscv-none-elf-ld
RISCV_AR = `$(RISCV_GCC_BIN)/riscv-none-elf-ar
RISCV_OBJCOPY = `$(RISCV_GCC_BIN)/riscv-none-elf-objcopy
RISCV_OBJDUMP = `$(RISCV_GCC_BIN)/riscv-none-elf-objdump
RISCV_SIZE = `$(RISCV_GCC_BIN)/riscv-none-elf-size
RISCV_GDB = `$(RISCV_GCC_BIN)/riscv-none-elf-gdb

OPENOCD_ROOT = $OD
OPENOCD = $OD/bin/openocd

# ============================================
# IOsonata Paths
# ============================================
# IOCOMPOSER_HOME must be set to your IOcomposer root directory
ifndef IOCOMPOSER_HOME
`$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

IOSONATA_ROOT = `$(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = `$(IOSONATA_ROOT)/include
IOSONATA_SRC = `$(IOSONATA_ROOT)/src

# ============================================
# ARM-specific Paths
# ============================================
ARM_ROOT = `$(IOSONATA_ROOT)/ARM
ARM_CMSIS = `$(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = `$(ARM_CMSIS)/Core/Include
ARM_INCLUDE = `$(ARM_ROOT)/include
ARM_SRC = `$(ARM_ROOT)/src
ARM_LDSCRIPT = `$(ARM_ROOT)/ldscript

# Vendor-specific paths
ARM_NORDIC = `$(ARM_ROOT)/Nordic
ARM_NXP = `$(ARM_ROOT)/NXP
ARM_ST = `$(ARM_ROOT)/ST
ARM_MICROCHIP = `$(ARM_ROOT)/Microchip
ARM_RENESAS = `$(ARM_ROOT)/Renesas

# ============================================
# RISC-V-specific Paths
# ============================================
RISCV_ROOT = `$(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = `$(RISCV_ROOT)/include
RISCV_SRC = `$(RISCV_ROOT)/src
RISCV_LDSCRIPT = `$(RISCV_ROOT)/ldscript

# Vendor-specific paths
RISCV_ESPRESSIF = `$(RISCV_ROOT)/Espressif
RISCV_NORDIC = `$(RISCV_ROOT)/Nordic
RISCV_RENESAS = `$(RISCV_ROOT)/Renesas

# ============================================
# External Libraries
# ============================================
EXTERNAL_ROOT = `$(IOCOMPOSER_HOME)/external
NRFX_ROOT = `$(EXTERNAL_ROOT)/nrfx
SDK_NRF_BM_ROOT = `$(EXTERNAL_ROOT)/sdk-nrf-bm
SDK_NRFXLIB_ROOT = `$(EXTERNAL_ROOT)/sdk-nrfxlib
NRF5_SDK_ROOT = `$(EXTERNAL_ROOT)/nRF5_SDK
NRF5_SDK_MESH_ROOT = `$(EXTERNAL_ROOT)/nRF5_SDK_Mesh
BSEC_ROOT = `$(EXTERNAL_ROOT)/BSEC
FUSION_ROOT = `$(EXTERNAL_ROOT)/Fusion
VQF_ROOT = `$(EXTERNAL_ROOT)/vqf
LVGL_ROOT = `$(EXTERNAL_ROOT)/lvgl
LWIP_ROOT = `$(EXTERNAL_ROOT)/lwip
FREERTOS_KERNEL_ROOT = `$(EXTERNAL_ROOT)/FreeRTOS-Kernel
TINYUSB_ROOT = `$(EXTERNAL_ROOT)/tinyusb

# ============================================
# Additional IOsonata Modules
# ============================================
FATFS_ROOT = `$(IOSONATA_ROOT)/fatfs
LITTLEFS_ROOT = `$(IOSONATA_ROOT)/littlefs
MICRO_ECC_ROOT = `$(IOSONATA_ROOT)/micro-ecc

# ============================================
# Common Include Paths (for -I flags)
# ============================================
IOSONATA_INCLUDES = -I`$(IOSONATA_INCLUDE) \
                    -I`$(IOSONATA_INCLUDE)/bluetooth \
                    -I`$(IOSONATA_INCLUDE)/audio \
                    -I`$(IOSONATA_INCLUDE)/converters \
                    -I`$(IOSONATA_INCLUDE)/coredev \
                    -I`$(IOSONATA_INCLUDE)/display \
                    -I`$(IOSONATA_INCLUDE)/imu \
                    -I`$(IOSONATA_INCLUDE)/miscdev \
                    -I`$(IOSONATA_INCLUDE)/pwrmgnt \
                    -I`$(IOSONATA_INCLUDE)/sensors \
                    -I`$(IOSONATA_INCLUDE)/storage \
                    -I`$(IOSONATA_INCLUDE)/sys \
                    -I`$(IOSONATA_INCLUDE)/usb

ARM_INCLUDES = -I`$(ARM_INCLUDE) \
               -I`$(ARM_CMSIS_INCLUDE)

RISCV_INCLUDES = -I`$(RISCV_INCLUDE)

# ============================================
# Environment Variables (optional)
# ============================================
export ARM_GCC_HOME := $AD/bin
export RISCV_GCC_HOME := $RD/bin
export OPENOCD_HOME := $OD/bin
export NRFX_HOME := `$(NRFX_ROOT)
export NRFXLIB_HOME := `$(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := `$(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := `$(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := `$(BSEC_ROOT)
"@
Set-Content $MK $MK_CONTENT -Encoding UTF8
Write-Host "   [OK] makefile_path.mk generated." -ForegroundColor Green
Write-Host "   Projects can include it with: include `$(IOSONATA_ROOT)/makefile_path.mk" -ForegroundColor Green

# ---------------------------------------------------------
# IOsonata Library Build (run exactly once)
# ---------------------------------------------------------
# If the clone script already invoked the builder, do NOT run it again.
# If it did NOT invoke the builder (e.g., clone supports -NoBuild), run it here.
if (-not $CLONE_INVOKED_BUILD) {
    $BS = "$ROOT\IOsonata\Installer\build_iosonata_lib_win.ps1"
    if (Test-Path $BS) {
        Write-Host
        Write-Host "=========================================================" -ForegroundColor Cyan
        Write-Host "  IOsonata Library Build" -ForegroundColor Cyan
        Write-Host "=========================================================" -ForegroundColor Cyan
        Write-Host
        & $BS -SdkHome $ROOT -IocomposerDir $ECLIPSE_DIR
		if ($LASTEXITCODE -ne 0) { throw "Library build failed with exit code $LASTEXITCODE" }
    } else { 
        Write-Host
        Write-Host "   [INFO] IOsonata library builder not found:" -ForegroundColor Yellow
        Write-Host "   $BS" -ForegroundColor Yellow
        Write-Host "   Library build not run: builder script is missing." -ForegroundColor Yellow
        Write-Host
    }
} else {
    Write-Host
    Write-Host "   IOsonata library build was invoked by SDK synchronization; not running it again."
    Write-Host
}

Build-IOcomposerIndex

# --- Summary ---
Write-Host
Write-Host "==============================================" -ForegroundColor Green
Write-Host " IOcomposer MCU Dev Tools Installation Summary" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green

$ECLIPSE_VER = "Not installed"
$eclipseProduct = "$ECLIPSE_DIR\.eclipseproduct"
if (Test-Path $eclipseProduct) {
    $prodContent = Get-Content $eclipseProduct -Raw
    if ($prodContent -match 'version=(.+)') { $ECLIPSE_VER = $Matches[1].Trim() }
}

$ARM_VER = "Not found"
if ($ARM_DIR -and (Test-Path "$ARM_DIR\bin\arm-none-eabi-gcc.exe")) {
    try {
        $verOut = (& "$ARM_DIR\bin\arm-none-eabi-gcc.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $ARM_VER = $Matches[1] }
    } catch {
        $ARM_VER = "Installed"
    }
}

$RISCV_VER = "Not found"
if ($RISCV_DIR -and (Test-Path "$RISCV_DIR\bin\riscv-none-elf-gcc.exe")) {
    try {
        $verOut = (& "$RISCV_DIR\bin\riscv-none-elf-gcc.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $RISCV_VER = $Matches[1] }
    } catch {
        $RISCV_VER = "Installed"
    }
}

$OPENOCD_VER = "Not found"
if ($OPENOCD_DIR -and (Test-Path "$OPENOCD_DIR\bin\openocd.exe")) {
    try {
        $verOut = (& "$OPENOCD_DIR\bin\openocd.exe" --version 2>&1) | Out-String -Stream | Select-Object -First 1
        if ($verOut -match '(\d+\.\d+\.\d+)') { $OPENOCD_VER = $Matches[1] }
    } catch {
        $OPENOCD_VER = "Installed"
    }
}

Write-Host ("{0,-25} {1}" -f "IOcomposer runtime:", $ECLIPSE_VER)
Write-Host ("{0,-25} {1}" -f "ARM GCC:", $ARM_VER)
Write-Host ("{0,-25} {1}" -f "RISC-V GCC:", $RISCV_VER)
Write-Host ("{0,-25} {1}" -f "OpenOCD:", $OPENOCD_VER)
Write-Host
Write-Host ("{0,-25} {1}" -f "iocomposer_home:", $RT)
Write-Host ("{0,-25} {1}" -f "iosonata_loc:", $RT)

Write-Host "==============================================" -ForegroundColor Green
Write-Host " IOcomposer development tools setup completed" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green
Write-Host
Write-Host "Usage in .cproject files:"
Write-Host "  `${system_property:iosonata_loc}/IOsonata/include"
Write-Host "  `${system_property:iosonata_loc}/IOsonata/ARM/include"
Write-Host "  `${system_property:iosonata_loc}/IOsonata/ARM/CMSIS/Core/Include"
Write-Host
Write-Host "Python environment: $PYOCD_DIR"
Write-Host "pyOCD: $PYOCD"
Write-Host "IntelHex (HEX merge): $TOOL_BIN\hexmerge.cmd"
Write-Host "Tool binaries: $TOOL_BIN"
Write-Host "LLVM (Arm + RISC-V): $LLVM_STATUS"
exit 0
