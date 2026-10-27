# IOcomposer website launcher. Installation is owned by IOsonata/Installer.
param(
	[string]$SdkHome = "$env:USERPROFILE\IOcomposer",
	[string]$IocomposerDir = "$env:ProgramFiles\IOcomposer",
	[switch]$ForceUpdate,
	[switch]$Uninstall,
	[switch]$Version,
	[switch]$Help
)
$ErrorActionPreference = 'Stop'
$ref = $env:IOSONATA_INSTALLER_REF
if (-not $ref) { $ref = 'main' }
$url = "https://raw.githubusercontent.com/IOsonata/IOsonata/$ref/Installer/install_iocdevtools_win.ps1"
$stage = Join-Path $env:TEMP ([guid]::NewGuid().ToString('N'))
New-Item $stage -ItemType Directory | Out-Null
try {
	$installer = Join-Path $stage 'install.ps1'
	Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $installer
	& $installer @PSBoundParameters
	if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} finally {
	Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
}
