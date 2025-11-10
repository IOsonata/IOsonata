# IOsonata Documentation

This directory contains the source for the IOsonata documentation site.

## 🌐 Live Site

**Production**: https://iosonata.github.io/IOsonata/ (after deployment)

## 🚀 Quick Commands

```bash
# Install dependencies
pip install mkdocs-material mkdoxy pymdown-extensions

# Serve locally (with live reload)
mkdocs serve

# Build site
mkdocs build

# Clean build
rm -rf site api && mkdocs build --clean

# Generate only Doxygen
cd docs && doxygen Doxyfile
```

## 📁 Directory Structure

```
docs/
├── index.md              # Homepage
├── getting-started/      # Beginner tutorials
├── architecture/         # Design philosophy
├── api/                  # API reference (Doxygen generated)
├── examples/             # Example guides
├── porting/              # Porting guides
├── contributing/         # Contribution guides
├── logo/                 # Images
├── Doxyfile              # Doxygen configuration
└── README.md             # This file
```

## ✏️ Writing Documentation

### Create New Page

1. Create file: `docs/section/page.md`
2. Add to `mkdocs.yml` navigation
3. Test: `mkdocs serve`
4. View: http://127.0.0.1:8000

### Markdown Extensions

#### Admonitions
```markdown
!!! tip "Pro Tip"
    Helpful information

!!! warning
    Important warning

!!! failure "Error"
    Common mistake
```

#### Tabs
```markdown
=== "C++"
    ```cpp
    code here
    ```

=== "C"
    ```c
    code here
    ```
```

#### Code Blocks with Highlighting
````markdown
```cpp title="main.cpp" linenums="1" hl_lines="3 4"
#include <stdio.h>

int main() {
    printf("Hello!\n");
    return 0;
}
```
````

## 🔧 Configuration Files

- `../mkdocs.yml` - Site configuration
- `Doxyfile` - API doc configuration
- `../.github/workflows/documentation.yml` - Auto-deploy workflow

## 🤝 Contributing

1. Fork repo
2. Create branch: `git checkout -b docs/topic`
3. Edit markdown files
4. Test locally: `mkdocs serve`
5. Commit: `git commit -m "docs: description"`
6. Push and create PR

## 📚 Resources

- [MkDocs Documentation](https://www.mkdocs.org/)
- [Material Theme](https://squidfunk.github.io/mkdocs-material/)
- [Markdown Guide](https://www.markdownguide.org/)
- [Doxygen Manual](https://www.doxygen.nl/manual/)

## 🐛 Troubleshooting

### Local Build Fails

**Missing module**: `pip install mkdocs-material mkdoxy`

**Missing Doxygen**: Install via brew/apt/chocolatey

### Links Broken

Use relative paths: `[Link](../page.md)` not `/page.md`

### Search Not Working

Ensure `search` plugin in `mkdocs.yml`

## 📞 Support

- **Documentation Issues**: Edit this repo
- **Framework Issues**: https://github.com/IOsonata/IOsonata/issues
- **Email**: info@i-syst.com
