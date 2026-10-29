# IOcomposer installation quick start

IOcomposer uses Eclipse Embedded CDT managed projects. Install the IDE,
toolchains and SDK checkouts, then build the MCU library used by your application.
The [Getting Started guide](../docs/getting-started.md) continues through
Blinky, board configuration, flashing and debugging.

## Install

Download and run the development-tool installer for your platform.

### macOS

```sh
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh -o /tmp/install_iocdevtools_macos.sh
bash /tmp/install_iocdevtools_macos.sh
```

### Linux

```sh
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh -o /tmp/install_iocdevtools_linux.sh
bash /tmp/install_iocdevtools_linux.sh
```

Linux needs Python 3.9 or newer with venv support. See the
[installer requirements](README.md) for distribution dependencies.

### Windows

Run PowerShell as Administrator:

```powershell
Invoke-WebRequest https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 -OutFile "$env:TEMP\install_iocdevtools_win.ps1"
powershell -NoProfile -ExecutionPolicy Bypass -File "$env:TEMP\install_iocdevtools_win.ps1"
```

The website launchers at iocomposer.io delegate to these scripts. IOcomposer,
plugins and preferences are set up before library compilation. On macOS,
signing precedes instance initialization and MCU preference setup.
A reported library compilation failure does not prevent IDE installation from
completing. Keep the reported log, fix the failed target and rerun its builder.

## Default locations

| Platform | IDE | SDK root |
|---|---|---|
| macOS | `/Applications/IOcomposer.app` | `~/IOcomposer` |
| Linux | `~/iocomposer` | `~/IOcomposer` |
| Windows | `C:\Program Files\IOcomposer` | `%USERPROFILE%\IOcomposer` |

The SDK root contains `IOsonata`, `external`, `TaktOS` when installed, and
`Tools`. Firmware utilities are under `Tools/bin`; managed Python packages
are under `Tools/Python`. HEX merging uses IntelHex. LLVM and SRecord are
not installed. Use the actual paths printed by the installer for a custom setup.

## Build or rebuild libraries

Run the builder from any directory:

```sh
# macOS
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_macos.sh

# Linux
bash ~/IOcomposer/IOsonata/Installer/build_iosonata_lib_linux.sh
```

```powershell
# Windows
& "$env:USERPROFILE\IOcomposer\IOsonata\Installer\build_iosonata_lib_win.ps1"
```

Select the MCU library you need. The builder discovers managed projects under
`lib/ioc` and reports each configuration's result. It also builds supported
TaktOS library projects; cm55 and esp32c3 are skipped. The macOS builder
requires TaktOS and builds it before the IOsonata selection menu. Linux and
Windows accept `--no-taktos` and `-NoTaktos`, respectively, to omit that step.
A listed MCU project does not establish hardware support for every peripheral;
check [Supported Targets](../docs/supported-targets.md).

Libraries are written under each project configuration, for example
`ARM/Nordic/nRF52/nRF52840/lib/ioc/Release/`. Match the application's target,
Debug/Release profile and any stack or modem OS suffix to the library.

## Update an existing checkout

```sh
cd ~/IOcomposer/IOsonata
git pull
bash Installer/build_iosonata_lib_macos.sh
```

Use the Linux or Windows builder on that platform. Then clean and rebuild
applications that link the changed library. For local modifications, resolve
Git's update result before building; do not discard application changes.

## Troubleshooting

| Problem | Action |
|---|---|
| IDE or wizard missing | Rerun the development-tool installer |
| Compiler missing | Check the installed tool paths and IOcomposer MCU preferences |
| Vendor header or library missing | Check the selected project's dependencies under `external` |
| Library build failed | Read the exact log path printed by the builder; rebuild that target |
| Firmware links against an older API | Rebuild the library, clean the application and relink |
| Debug output stops outside the debugger | Check whether the configuration uses rdimon semihosting; use Release for standalone tests where provided |

For custom paths and all script options, use [Installer README](README.md).
