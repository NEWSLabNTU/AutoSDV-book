# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- Setup: `just setup` - Installs MkDocs and all dependencies
- Build: `just build` - Compiles the HTML documentation using MkDocs (both English and Chinese versions)
- Serve: `just serve` - Starts development server at http://0.0.0.0:3000 with live reload
- Lint: `just lint` - Validates documentation format (MkDocs strict build) and checks translation sync
- **Audit Translations**: `just audit-translations` - AI-based semantic translation audit using Claude CLI (with intelligent caching, 18-50x faster on subsequent runs)
- Clean: `just clean` - Removes all build outputs (site/ directory)
- Check: `just check` - Verifies all dependencies are installed
- Deploy: `just deploy` - Deploys to GitHub Pages (manual deployment)

## Framework: MkDocs with Material Theme

This project uses **MkDocs** with the **Material for MkDocs** theme, following Autoware documentation conventions.

**Key Features**:
- ✅ Material Design theme
- ✅ Multi-language support (English + 繁體中文)
- ✅ Mermaid diagram support (built-in)
- ✅ Math rendering (MathJax)
- ✅ Code syntax highlighting
- ✅ Search functionality
- ✅ Mobile responsive
- ✅ Dark/light mode

## Development Practices

### Temporary Files
- **ALWAYS** create temporary files in the project's `tmp/` directory (e.g., `book/tmp/`)
- **NEVER** use system `/tmp/` or shell heredoc (`cat << EOF`)
- **ALWAYS** use Write/Edit tools instead of bash commands for creating/modifying files
- Avoid bash `cat > file << 'EOF'` - use Write tool instead
- Example path: `/home/aeon/repos/AutoSDV/book/tmp/script_name.py`

## GitHub Actions

The repository uses two GitHub Actions workflows:

### 1. Test Workflow (`.github/workflows/test.yml`)
- **Triggers**: On every commit to any branch and on pull requests
- **Purpose**: Continuous Integration - audits and tests documentation builds
- **Actions**:
  - Builds documentation with `mkdocs build --strict`
  - Checks for broken links
  - Validates navigation structure
  - Does NOT deploy (test-only)

### 2. Deploy Workflow (`.github/workflows/deploy.yml`)
- **Triggers**: Only when tags matching `book-v*` are pushed (e.g., `book-v1.0.0`)
- **Purpose**: Production deployment to GitHub Pages
- **Actions**:
  - Builds documentation
  - Deploys to `gh-pages` branch
  - Updates NEWSLabNTU.github.io website repository's submodule
- **Required Secret**: `AUTOSDV_BOOK_DEPLOY_KEY` (SSH deploy key)

### Creating a Release
To deploy documentation:
```bash
# Create and push a tag
git tag -a book-v1.0.0 -m "Documentation v1.0.0"
git push origin book-v1.0.0
```

**Tag Format**: Use `book-v<semver>` (e.g., `book-v1.0.0`, `book-v1.1.0`, `book-v2.0.0`)
- MAJOR: Breaking changes, navigation overhaul
- MINOR: New content sections, significant updates
- PATCH: Bug fixes, typo corrections

## Style Guidelines
- Use consistent Markdown formatting in `.md` files
- Include alt text for all images in the `src/figures` directory
- Add new pages to the `nav:` section in `mkdocs.yml`
- Use relative links for internal references between pages
- Maintain a clear, instructional tone consistent with existing documentation
- For code examples, use proper syntax highlighting with triple backticks and language identifier
- Keep file and directory names lowercase with hyphens (kebab-case)
- Ensure all images are properly sized and optimized for web viewing

## Project Structure

The documentation is organized into logical categories:

```
src/
├── index.md                                 # Home page (includes Getting Started overview)
├── platform-models.md                       # Vehicle build variants
│
├── getting-started/                         # For new users - setup and first run
│   ├── hardware-assembly.md                 # Hardware setup and usage
│   ├── installation/                        # Software installation guides
│   │   ├── overview.md                      # Recommended installation method
│   │   ├── zed-sdk.md                       # ZED SDK installation
│   │   ├── manual-environment.md            # Manual environment setup (optional)
│   │   └── docker.md                        # Docker setup (optional)
│   └── usage.md                             # Operating and launching the system
│
├── guides/                                  # Tutorial-style guides for operators and developers
│   ├── development.md                       # Development guide overview
│   ├── source-code.md                       # Repository structure walkthrough
│   ├── version-control.md                   # Git submodules workflow
│   ├── sensor-integration/                  # Sensor integration tutorials
│   │   ├── using-sensors.md                 # Simple usage guide (sensor suites, launch commands)
│   │   ├── integration-walkthrough.md       # Learn by example (Robin-W LiDAR walkthrough)
│   │   ├── lidar.md                         # LiDAR-specific configuration details
│   │   ├── camera.md                        # Camera-specific configuration details
│   │   ├── imu.md                           # IMU-specific configuration details
│   │   ├── gnss.md                          # GNSS-specific configuration details
│   │   ├── adding-sensor.md                 # Quick checklist for adding new sensors
│   │   └── troubleshooting.md               # Common sensor issues and solutions
│   └── vehicle-control/                     # Vehicle control chapter
│       ├── overview.md                      # Architecture & dataflow
│       ├── hardware.md                      # Hardware specifications & wiring
│       ├── control-details.md               # Control algorithms & implementation
│       └── tuning-and-testing.md            # Testing, tuning, calibration
│
├── reference/                               # Technical specifications and details
│   ├── overview.md                          # Technical reference overview
│   ├── hardware/                            # Hardware specifications
│   │   ├── core-components.md               # Component specifications
│   │   └── wiring-diagrams.md               # Electrical wiring diagrams
│   └── networking/                          # Network and connectivity
│       └── 5g-deployment.md                 # 5G/LTE deployment guide
│
└── figures/                                 # Images and diagrams
```

### File Organization Guidelines
- Use kebab-case (lowercase with hyphens) for file and directory names
- Add new pages to `nav:` section in `mkdocs.yml` (English names)
- Add translations to `nav_translations:` section in `mkdocs.yml` (Chinese names)
- Use relative paths for internal links (adjust based on file location depth)
- MkDocs automatically builds both English and Chinese versions from the same source
- Chinese translations use `.zh-TW.md` suffix (e.g., `index.zh-TW.md`)

## Vehicle Control Documentation Guidelines

**Organization**: overview → hardware → algorithms → practice

**Four files**:
1. `overview.md` - Architecture, components, Autoware integration
2. `hardware.md` - PCA9685, motors, sensors, wiring
3. `control-details.md` - Algorithms (source: `autosdv_vehicle_interface/README.md`)
4. `tuning-and-testing.md` - Testing, tuning, calibration

**Key rules**:
- Safety first: Always use controller GUI, never show manual `ros2 topic pub`
- No duplication: Each concept in exactly one file
- Test everything: Verify all commands work
- Base on source code: Check actual README.md and .yaml files

## Sensor Documentation Guidelines

**Philosophy**: Simple usage → Concrete example (Robin-W) → Sensor-specific details

**Four files**:
1. `using-sensors.md` - Sensor suites table, launch commands, quick verification
2. `integration-walkthrough.md` - Complete Robin-W walkthrough (hardware → driver → config → data flow)
3. Sensor-specific guides (`lidar.md`, `camera.md`, `imu.md`, `gnss.md`) - Only sensor-unique details
4. `adding-sensor.md` - Quick checklist

**Key rules**:
- Concise over comprehensive
- No duplication: Integration concepts in walkthrough, sensor specifics in individual guides
- Be specific: Show exact IPs (e.g., "Robin-W: 172.168.1.10"), not generic instructions
- Test everything: Verify all commands work
- Images in `src/figures/sensor-integration/` with descriptive names

## Translation Workflow

**System**: Suffix-based i18n using `.zh-TW.md` files (e.g., `index.md` + `index.zh-TW.md`)

**Status**: Chinese version enabled with 33 nav translations, partial content translations

### Common Issues

**Images missing on Chinese site** (404 errors):
- **Cause**: Chinese pages served under `/zh-TW/`, need one extra `../` in paths
- **Fix**: Add one extra `../` to image paths (EN: `../figures/x.png` → ZH: `../../figures/x.png`)

**Navigation in English on Chinese site**:
- **Cause**: Missing entries in `mkdocs.yml` under `nav_translations`
- **Fix**: Add `English Title: 中文標題` for each nav item

### Creating Translations

1. Copy: `cp src/file.md src/file.zh-TW.md`
2. Translate text (keep code/commands/filenames untranslated)
3. Fix image paths (add one extra `../`)
4. Add nav translations to `mkdocs.yml` if new page
5. Build: `just build`

### Translation Guidelines

**DO Translate:**
- UI text, navigation elements, descriptions
- Error messages and warnings
- Comments within code blocks (only comments, not the code itself)

**DO NOT Translate:**
- Code snippets, commands, file paths
- Configuration values, technical specifications
- Product names (e.g., Velodyne VLP-32C, NVIDIA Jetson)
- Version numbers (e.g., Ubuntu 22.04, ROS 2 Humble)

**Quality Standards:**
- Use Taiwanese Traditional Chinese (繁體中文)
- Keep technical terms in English unless well-established Chinese term exists
- Maintain consistent terminology throughout

**Common Terms:**
- 感測器 (sensors), 驅動程式 (drivers), 車輛介面 (vehicle interface)
- 套件 (package), 環景光達 (360° LiDAR), 固態光達 (solid-state LiDAR)
- 原始碼說明 (source code walkthrough), 配線 (wiring)

**Image Paths in Chinese Translations:**

Chinese files are served with an extra `/zh-TW/` directory level, requiring one additional `../` in image paths:

```markdown
<!-- English file (src/index.md) - special case: site root -->
![Image](figures/image.png)

<!-- Chinese file (src/index.zh-TW.md) needs extra ../ -->
![Image](../figures/image.png)

<!-- English file (src/platform-models.md) at root level -->
![Image](../figures/image.png)

<!-- Chinese file (src/platform-models.zh-TW.md) needs extra ../ -->
![Image](../../figures/image.png)
```

**Pattern by file depth:**
- **Site index** (`src/index.md`): EN uses `figures/`, ZH uses `../figures/`
- **Root level** (`src/*.md`): EN uses `../figures/`, ZH uses `../../figures/`
- **One level** (`src/guides/*.md`): EN uses `../../figures/`, ZH uses `../../../figures/`
- **Two levels** (`src/guides/sensor/*.md`): EN uses `../../../figures/`, ZH uses `../../../../figures/`

**Rule:** Add one extra `../` to the English path for the Chinese version.

### Maintaining Translation Sync

**Two-tier approach**: Structural checking (daily) + AI semantic analysis (pre-release)


**Tier 1: Structural Checking** (fast, daily)
- Command: `just lint`
- Detects: Missing/outdated files, structural differences, git diff
- Best for: PR reviews, weekly maintenance

**Tier 2: AI Semantic Analysis** (comprehensive, pre-release)
- Command: `just audit-translations` (requires Claude CLI)
- Detects: Missing content, semantic drift, outdated info  
- Features: Smart caching (18-50x faster on re-runs)
- Best for: Pre-release validation

See `scripts/README.md` for detailed usage.

### Nav Translations

Navigation titles are translated in `mkdocs.yml` under the `i18n.languages.zh-TW.nav_translations` section.

**Important:** All navigation items must have translations, otherwise they will appear in English on the Chinese site. When adding new pages:
1. Add the page to the `nav:` section with its English title
2. Add a translation entry to `nav_translations:` with format `English Title: 中文標題`

**Example:**
```yaml
nav:
  - Guides:
    - Using Sensors: guides/sensor-integration/using-sensors.md

# In the i18n.languages.zh-TW section:
nav_translations:
  Using Sensors: 使用感測器
```

### Configuration

- Main config: `mkdocs.yml`
- Language settings: `plugins.i18n.languages` section
- Navigation translations: `plugins.i18n.languages.zh-TW.nav_translations`
- Chinese search: Requires `jieba>=0.42` in `requirements.txt`

### Search Configuration

**Chinese search is now built-in** (Material for MkDocs v8.0+):
- Uses **jieba** for Chinese text segmentation (not lunr.js)
- Automatic detection when jieba is installed
- Zero-width whitespace (\u200b) included in search separator
- Language-specific search: `lang: [en, zh]` in search plugin config

**No configuration needed** - just install jieba:
```bash
pip install -r requirements.txt  # Includes jieba>=0.42
```

### Important Notes & Limitations

**File naming:**
- Chinese files must have `.zh-TW.md` suffix
- Both language versions are built into `site/` directory
- Access via `http://0.0.0.0:3000/AutoSDV-book/` (English) or `http://0.0.0.0:3000/AutoSDV-book/zh-TW/` (Chinese)

**Known limitation - navigation.instant incompatibility:**
- ❌ `navigation.instant` is **fundamentally incompatible** with mkdocs-static-i18n's multi-language switcher
- This is a known architectural limitation, not a bug
- **Must choose**: Multi-language switcher OR instant navigation (cannot have both)
- **Current choice**: Multi-language switcher (better for international users)
- Source: [mkdocs-static-i18n Material setup guide](https://ultrabug.github.io/mkdocs-static-i18n/setup/setting-up-material/)

## Image Path Guidelines

MkDocs creates directory-style URLs with trailing slashes. Image paths must account for the file's depth:

**Site index** (`src/index.md`):
- Served as: `/AutoSDV-book/` (site root)
- Image path: `figures/image.png` (relative from root)

**Root-level files** (`src/platform-models.md`, etc.):
- Served as: `/AutoSDV-book/platform-models/`
- Image path: `../figures/image.png` (one level up)

**One-level subdirectory** (`src/getting-started/hardware-assembly.md`):
- Served as: `/AutoSDV-book/getting-started/hardware-assembly/`
- Image path: `../../figures/image.png` (two levels up)

**Two-level subdirectory** (`src/reference/networking/5g-deployment.md`):
- Served as: `/AutoSDV-book/reference/networking/5g-deployment/`
- Image path: `../../../figures/image.png` (three levels up)

**Example**:
```markdown
<!-- In src/index.md (site root - special case) -->
![Logo](../figures/logo.png)        ❌ Wrong
![Logo](figures/logo.png)            ✓ Correct

<!-- In src/platform-models.md (root level) -->
![Logo](figures/logo.png)            ❌ Wrong
![Logo](../figures/logo.png)         ✓ Correct

<!-- In src/getting-started/hardware-assembly.md -->
![Vehicle](../figures/vehicle.jpg)   ❌ Wrong
![Vehicle](../../figures/vehicle.jpg) ✓ Correct
```

**Chinese Translations:**
Chinese files need one extra `../` in image paths due to the `/zh-TW/` subdirectory. See the "Image Paths in Chinese Translations" section in the Translation Workflow for details.

All images should be stored in `src/figures/` and use `.png`, `.jpg`, `.jpeg`, `.svg`, `.gif`, or `.webp` formats.

## Anchor Links for Chinese Headings

**Problem**: MkDocs doesn't generate reliable anchor IDs from Chinese characters, and the `{#custom-id}` attr_list syntax conflicts with the mkdocs-macros plugin (which interprets `{#...}` as Jinja2 comment syntax).

**Solution**: Use HTML `<span id="...">` tags for explicit anchor IDs in Chinese translations.

**Example**:
```markdown
<!-- ❌ Don't use attr_list syntax (conflicts with macros plugin) -->
## 基礎型號 {#base-model}

<!-- ✓ Use HTML span tags instead -->
<span id="base-model"></span>
## 基礎型號
```

**When to use**:
- When Chinese headings need explicit anchor IDs for cross-references
- When linking from one Chinese page to another Chinese page's section
- To maintain consistency with English anchor IDs (e.g., `#base-model` in both EN and ZH versions)

**Notes**:
- Place the `<span>` tag immediately before the heading
- Use English slugs for anchor IDs (matches English version)
- The macros plugin has been reconfigured to use `[[...]]` delimiters instead of `{...}` to reduce conflicts, but HTML spans are still the recommended approach for anchor IDs