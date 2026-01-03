# IOsonata Installation Methods - Choose Your Path

## Three Ways to Install IOsonata

```
┌─────────────────────────────────────────────────────────────────┐
│                   Installation Method Options                    │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ 🌐 METHOD 1: Direct from GitHub (FASTEST)                       │
├─────────────────────────────────────────────────────────────────┤
│ Time: < 1 minute to start                                       │
│ Expertise: Beginner                                             │
│                                                                  │
│ macOS:                                                          │
│   curl -fsSL https://raw.githubusercontent.com/IOsonata/...     │
│     ...main/Installer/install_iocdevtools_macos.sh | bash          │
│                                                                  │
│ Linux:                                                          │
│   curl -fsSL https://raw.githubusercontent.com/IOsonata/...     │
│     ...main/Installer/install_iocdevtools_linux.sh | bash          │
│                                                                  │
│ Windows:                                                        │
│   iwr -useb https://raw.githubusercontent.com/IOsonata/...      │
│     ...main/Installer/install_iocdevtools_win.ps1 | iex            │
│                                                                  │
│ ✅ Pros: Fastest, one command, always latest                    │
│ ⚠️ Cons: Requires internet, trust GitHub                        │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ 📥 METHOD 2: Download Then Run (RECOMMENDED)                    │
├─────────────────────────────────────────────────────────────────┤
│ Time: 2-3 minutes to start                                      │
│ Expertise: Beginner                                             │
│                                                                  │
│ Step 1: Download                                                │
│   curl -O https://raw.githubusercontent.com/IOsonata/...        │
│                                                                  │
│ Step 2: Review (IMPORTANT!)                                     │
│   cat install_iocdevtools_macos.sh | less                       │
│                                                                  │
│ Step 3: Run                                                     │
│   chmod +x install_iocdevtools_macos.sh                         │
│   ./install_iocdevtools_macos.sh                                │
│                                                                  │
│ ✅ Pros: Can review before running, secure, offline capable     │
│ ⚠️ Cons: Extra step, slightly slower                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ 📦 METHOD 3: Clone Repository (TRADITIONAL)                     │
├─────────────────────────────────────────────────────────────────┤
│ Time: 5-10 minutes to start                                     │
│ Expertise: Familiar with Git                                    │
│                                                                  │
│ Step 1: Clone entire repository                                 │
│   git clone https://github.com/IOsonata/IOsonata.git            │
│                                                                  │
│ Step 2: Navigate to Installer                                       │
│   cd IOsonata/Installer                                             │
│                                                                  │
│ Step 3: Run installer                                           │
│   ./install_iocdevtools_macos.sh                                │
│                                                                  │
│ ✅ Pros: Full repo access, version control, offline use         │
│ ⚠️ Cons: Slower, more data to download                          │
└─────────────────────────────────────────────────────────────────┘
```

## Quick Comparison

| Factor | Direct from GitHub | Download Then Run | Clone Repository |
|--------|-------------------|-------------------|------------------|
| **Speed** | ⚡⚡⚡ Fastest | ⚡⚡ Fast | ⚡ Slower |
| **Security** | ⚠️ Trust needed | ✅ Can verify | ✅ Full control |
| **Offline use** | ❌ No | ✅ After download | ✅ Yes |
| **Expertise** | Beginner | Beginner | Git knowledge |
| **Review code** | ⚠️ Harder | ✅ Easy | ✅ Easy |
| **Disk space** | Minimal | Minimal | ~2 GB |
| **Best for** | Quick testing | Production use | Development |

## Detailed Instructions by Method

### METHOD 1: Direct from GitHub 🌐

**Perfect for**: Quick testing, demos, personal machines

**macOS - Complete command:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash
```

**What this does:**
1. Downloads the latest installer script from GitHub
2. Pipes it directly to bash for execution
3. Starts installation immediately
4. No files left on disk (except what installer creates)

**With custom path:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh | bash -s -- --home /custom/path
```

**Linux - Complete command:**
```bash
curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh | bash
```

**Windows - Complete command (PowerShell as Administrator):**
```powershell
iwr -useb https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 | iex
```

**⚠️ Security Consideration:**
This method requires trusting:
- GitHub's infrastructure
- The IOsonata repository
- Network security (HTTPS)

---

### METHOD 2: Download Then Run 📥

**Perfect for**: Production systems, team environments, security-conscious users

**macOS - Step by step:**
```bash
# 1. Download
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh

# 2. Review the script (IMPORTANT!)
cat install_iocdevtools_macos.sh | less
# or
nano install_iocdevtools_macos.sh
# or
code install_iocdevtools_macos.sh  # If you have VS Code

# 3. Make executable
chmod +x install_iocdevtools_macos.sh

# 4. Run
./install_iocdevtools_macos.sh
```

**Linux - Step by step:**
```bash
# Using curl
curl -O https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh

# Or using wget
wget https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh

# Review, make executable, run (same as macOS)
cat install_iocdevtools_linux.sh | less
chmod +x install_iocdevtools_linux.sh
./install_iocdevtools_linux.sh
```

**Windows - Step by step:**
```powershell
# 1. Download
Invoke-WebRequest -Uri https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1 -OutFile install_iocdevtools_win.ps1

# 2. Review (IMPORTANT!)
notepad install_iocdevtools_win.ps1
# or
code install_iocdevtools_win.ps1

# 3. Run
.\install_iocdevtools_win.ps1
```

**✅ Security Benefits:**
- You can review every line before execution
- Script is saved locally (can be used offline)
- Can share reviewed script with team
- Can keep for auditing/compliance

---

### METHOD 3: Clone Repository 📦

**Perfect for**: Developers, contributors, offline environments

**Step by step:**
```bash
# 1. Clone the entire IOsonata repository
git clone https://github.com/IOsonata/IOsonata.git

# 2. Navigate to Installer directory
cd IOsonata/Installer

# 3. Review scripts (optional but recommended)
ls -la
cat install_iocdevtools_macos.sh | less

# 4. Run installer
./install_iocdevtools_macos.sh
```

**Using sparse checkout (only tools directory):**
```bash
# Clone with sparse checkout (saves bandwidth/space)
git clone --filter=blob:none --sparse https://github.com/IOsonata/IOsonata.git
cd IOsonata
git sparse-checkout add tools

# Now you have just the Installer/ directory
cd Installer
./install_iocdevtools_macos.sh
```

**✅ Benefits:**
- Full repository access
- Can contribute changes
- Version control (can switch branches/tags)
- Completely offline after initial clone
- See commit history

---

## Decision Helper

### Choose Method 1 (Direct) if:
- ✅ You want the absolute fastest setup
- ✅ You're on a personal/development machine
- ✅ You trust GitHub and the IOsonata project
- ✅ You're doing a quick test or demo
- ✅ You have internet connection

### Choose Method 2 (Download Then Run) if:
- ✅ You're setting up a production system
- ✅ Security is a concern
- ✅ You want to review the code first
- ✅ You need to keep a copy of the script
- ✅ You might run it offline later
- ✅ You're following company security policies

### Choose Method 3 (Clone Repository) if:
- ✅ You want the full IOsonata source code
- ✅ You plan to contribute to IOsonata
- ✅ You need version control
- ✅ You want to work offline
- ✅ You need to track changes
- ✅ You're comfortable with Git

---

## Recommendation by Use Case

| Use Case | Recommended Method |
|----------|-------------------|
| **First-time user, just exploring** | Method 1 (Direct) |
| **Production server setup** | Method 2 (Download) |
| **Team standardization** | Method 2 (Download) |
| **CI/CD pipeline** | Method 1 or 2 |
| **Contributing to IOsonata** | Method 3 (Clone) |
| **Offline environment** | Method 2 or 3 |
| **Air-gapped system** | Method 3 |
| **Quick demo** | Method 1 (Direct) |
| **Corporate environment** | Method 2 (Download) |
| **Development workstation** | Method 3 (Clone) |

---

## Time Breakdown

### Method 1: Direct from GitHub
```
└─ Run command: 30 seconds
   ├─ Download script: 1 second
   ├─ Execute: immediately
   └─ Full installation: 15-20 minutes

Total time to running: ~30 seconds
```

### Method 2: Download Then Run
```
└─ Download script: 1 second
└─ Review script: 2-5 minutes (depends on how thorough)
└─ Run script: immediate
   └─ Full installation: 15-20 minutes

Total time to running: 2-5 minutes
```

### Method 3: Clone Repository
```
└─ Clone repository: 2-5 minutes (depends on connection)
└─ Navigate to Installer: 10 seconds
└─ Review (optional): 2-5 minutes
└─ Run script: immediate
   └─ Full installation: 15-20 minutes

Total time to running: 5-10 minutes
```

---

## All Installation URLs

### Installers
```
# macOS
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_macos.sh

# Linux
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_linux.sh

# Windows
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/install_iocdevtools_win.ps1
```

### Clone Scripts (if you have Eclipse already)
```
# macOS
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/clone_iosonata_sdk_macos.sh

# Linux
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/clone_iosonata_sdk_linux.sh

# Windows
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/clone_iosonata_sdk_win.ps1
```

### Build Scripts (if you have Eclipse + IOsonata already)
```
# macOS
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/build_iosonata_lib_macos.sh

# Linux
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/build_iosonata_lib_linux.sh

# Windows
https://raw.githubusercontent.com/IOsonata/IOsonata/main/Installer/build_iosonata_lib_win.ps1
```

---

## Still Unsure?

**For most users, we recommend Method 2 (Download Then Run)** because it:
- ✅ Is still very fast (2-3 minutes)
- ✅ Allows you to review the script
- ✅ Provides better security
- ✅ Gives you a local copy
- ✅ Works offline after download

**Try Method 1 if**: You just want to test IOsonata quickly and don't mind the security tradeoff.

**Try Method 3 if**: You're a developer who wants the full repository.

---

## More Information

- [ONLINE_INSTALLATION.md](ONLINE_INSTALLATION.md) - Detailed online installation guide
- [README.md](README.md) - Complete documentation
- [QUICKSTART.md](QUICKSTART.md) - 5-minute quick start
- [WHICH_SCRIPT.md](WHICH_SCRIPT.md) - Which script should I use?

---

**Ready to install? Pick your method and get started!** 🚀
