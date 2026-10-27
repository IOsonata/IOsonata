# IOcomposer website launchers

These three small scripts are the copies deployed to the root of
`I-SYST/iocomposer.io`. All installation logic lives in the parent `Installer`
directory. The website continues to publish plugin JARs and the splash asset.

Default handoff:

1. The website launcher downloads the corresponding IOsonata installer from main.
2. The installer sets up IOcomposer, toolchains, the SDK and library builds.
3. The launcher returns the installer's status and removes its temporary files.

Arguments are forwarded unchanged. Windows uses PowerShell parameters such as
`-SdkHome`, `-IocomposerDir`, `-ForceUpdate` and `-Help`, not Unix-style options.
For custom Windows arguments, download the launcher and invoke the file rather
than evaluating its contents with `Invoke-Expression`.

## Test before rollout

Set `IOSONATA_INSTALLER_REF` to `release_0.13_readiness` or an exact IOsonata
commit SHA before invoking a launcher. This also selects the downloaded SDK
clone script. It does not pin the cloned SDK repositories or plugin versions.
Unset the variable to return to main.

```bash
IOSONATA_INSTALLER_REF=release_0.13_readiness bash install_ioc_linux.sh --help
IOSONATA_INSTALLER_REF=release_0.13_readiness bash install_ioc_macos.sh --help
```

```powershell
$env:IOSONATA_INSTALLER_REF = 'release_0.13_readiness'
.\install_ioc_windows.ps1 -Help
Remove-Item Env:IOSONATA_INSTALLER_REF
```

Run the offline checks from the IOsonata repository root:

```bash
python3 tests/installer/test_installers.py
python3 tests/installer/test_iocomposer.py
```

Native fresh-install, update, custom-path, uninstall and build testing is still
required on Linux, macOS and Windows. Windows PowerShell execution and macOS
Gatekeeper/signing cannot be established by the Linux offline tests.

## Deployment order

Merge the IOsonata installer changes before merging the website launcher
changes. Otherwise the new thin website launchers would call the old installer
that only installs the Eclipse runtime. Keep these launcher copies and the
three website files identical when changing the handoff.

The rollout does not require changes to the website's existing install commands.
