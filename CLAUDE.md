# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- Build: `make build` - Compiles the HTML document using mdbook (both English and Chinese versions)
- Serve: `make serve` - Starts a web server at http://127.0.0.1:3000 with the compiled book
- Clean: `make clean` - Removes the compiled HTML document
- Extract Messages: `make extract-messages` - Extracts translatable messages to PO file

## Development Practices

### Temporary Files
- **ALWAYS** create temporary files in the project's `tmp/` directory
- **NEVER** use system `/tmp/` or shell heredoc (`cat << EOF`)
- Use Write/Edit tools instead of bash commands for file operations
- Example: `/home/aeon/repos/AutoSDV/2025.02/tmp/script_name.sh`

## GitHub Actions
- The repository uses GitHub Actions to automatically deploy the book to GitHub Pages
- Deployments are triggered on pushes to `main` or `2025.02` branches
- The deploy workflow also updates the NEWSLabNTU.github.io website repository's submodule
- The workflow is defined in `.github/workflows/deploy.yml`
- A deploy key named `AUTOSDV_BOOK_DEPLOY_KEY` must be added to repository secrets
- No manual deployment is needed - just push changes to the repository

## Style Guidelines
- Use consistent Markdown formatting in `.md` files
- Include alt text for all images in the `src/figures` directory
- Organize new content in appropriate sections in the `SUMMARY.md` file
- Use relative links for internal references between pages
- Maintain a clear, instructional tone consistent with existing documentation
- For code examples, use proper syntax highlighting with triple backticks and language identifier
- Keep file and directory names lowercase with underscores for spaces
- Ensure all images are properly sized and optimized for web viewing

## Project Structure

The documentation is organized into logical categories:

```
src/
├── introduction.md                          # Main project introduction
├── platform-models.md                       # Platform variants overview
│
├── getting-started/                         # For new users - setup and first run
│   ├── overview.md                          # Getting Started overview
│   ├── hardware-assembly.md                 # Physical vehicle assembly
│   ├── installation/                        # Software installation guides
│   │   ├── overview.md                      # Installation overview
│   │   ├── zed-sdk.md                       # ZED SDK installation
│   │   ├── manual-environment.md            # Manual environment setup
│   │   └── docker.md                        # Docker setup
│   └── usage.md                             # Operating and launching the system
│
├── guides/                                  # Tutorial-style guides for operators and developers
│   ├── development.md                       # Development guide overview
│   ├── source-code.md                       # Repository structure walkthrough
│   ├── version-control.md                   # Git submodules workflow
│   └── sensor-integration/                  # Sensor integration tutorials
│       ├── overview.md                      # Sensor architecture concepts
│       ├── quick-start.md                   # Using sensor suites
│       ├── lidar-sensors.md                 # LiDAR integration tutorial
│       ├── camera-sensors.md                # Camera integration tutorial
│       ├── imu-sensors.md                   # IMU integration tutorial
│       ├── gnss-sensors.md                  # GNSS integration tutorial
│       ├── adding-new-sensor.md             # Step-by-step sensor integration guide
│       └── troubleshooting.md               # Common sensor issues and solutions
│
├── reference/                               # Technical specifications and details
│   ├── overview.md                          # Technical reference overview
│   ├── hardware/                            # Hardware specifications
│   │   ├── core-components.md               # Component specifications
│   │   └── wiring-diagrams.md               # Electrical wiring diagrams
│   ├── software/                            # Software architecture
│   │   └── vehicle-interface.md             # Vehicle control interface
│   └── networking/                          # Network and connectivity
│       └── 5g-deployment.md                 # 5G/LTE deployment guide
│
└── figures/                                 # Images and diagrams
```

### File Organization Guidelines
- Use kebab-case (lowercase with hyphens) for file and directory names
- Update `src/SUMMARY.md` when adding new content files
- Use relative paths for internal links (adjust based on file location depth)
- Translation files are in `po/` directory (zh-TW.po for Traditional Chinese)
- Translation scripts are in `scripts/` directory
- Language-specific config: `book-zh-TW.toml` for Chinese version

## Sensor Documentation Guidelines

The sensor integration guides follow a **tutorial-first philosophy**: teach users how to work with sensors through practical, step-by-step workflows rather than abstract specifications.

### Documentation Philosophy

**What vs. How:**
- ❌ Old approach: "Here's what exists" (specifications, lists of components)
- ✅ New approach: "Here's how to do it" (tutorials, workflows, examples)

**Architecture Understanding:**
All sensor guides should help users understand the **two-layer system**:
1. **Sensor Component Layer** (`src/sensor_component/`) - Individual drivers
2. **Sensor Kit Layer** (`src/sensor_kit/`) - Composition and integration

### Content Structure for Each Sensor Type

Every sensor guide (LiDAR, camera, IMU, GNSS) should follow this structure:

```markdown
# [Sensor Type] Sensors

## Overview
- What this sensor type does
- Supported models (table with specs)

## [Specific Model Name]

### Hardware Setup
- Physical mounting and connections
- Power requirements
- Network/communication configuration

### Driver Package
- Location in repository
- Key features and data format
- Special considerations

### Sensor Kit Integration

#### 1. Add to Description Package
- Calibration YAML example
- URDF/xacro entry
- Coordinate frame explanation

#### 2. Add to Launch Configuration
- Launch file location and example
- Topic remapping
- Parameters

### Standalone Testing
- How to test driver independently
- Verification commands
- RViz visualization

### Advanced Configuration
- Model-specific features
- Performance tuning
- Integration with other systems

### Troubleshooting
- Common issues for this specific sensor
- Model-specific problems

## [Next Model]
[Repeat structure]
```

### Tutorial Workflow Pattern

The **"Adding a New Sensor"** guide is the centerpiece. It demonstrates the complete workflow:

1. **Obtain driver** (vendor package, wrapper, or custom)
2. **Standalone test** (verify driver works in isolation)
3. **Add to description** (sensor_kit_calibration.yaml + sensor_kit.xacro)
4. **Add to launch** (sensor kit launch files)
5. **Configure parameters** (individual_params)
6. **Integration test** (verify in full system)
7. **Document** (update guides)

All sensor-specific guides should reference this workflow pattern.

### Writing Style

**Be Specific:**
- ❌ "Configure the sensor"
- ✅ "Edit `sensor_kit_calibration.yaml` and add coordinates for `robin_lidar_link`"

**Show, Don't Tell:**
- Include complete code snippets, not fragments
- Provide exact file paths: `src/sensor_kit/autosdv_sensor_kit_launch/launch/lidar.launch.xml`
- Use line numbers when referencing existing code: `lidar-sensors.md:342`

**Explain Why:**
- Don't just list steps - explain the purpose
- Example: "The coordinate transformation (roll=180°, pitch=-90°) is needed because Robin-W uses non-standard coordinates (X=Up, Y=Right, Z=Forward) instead of ROS REP-103 (X=Forward, Y=Left, Z=Up)"

**Test Everything:**
- Every code snippet should be tested and working
- Include verification steps after each configuration change
- Provide expected output examples

### Cross-References

Create clear navigation between related topics:

```markdown
<!-- In quick-start.md -->
For detailed ZED camera configuration, see [Camera Sensors](./camera-sensors.md#zed-x-mini-stereo-camera).

<!-- In camera-sensors.md -->
For a complete walkthrough of integrating a new camera, see [Adding a New Sensor](./adding-new-sensor.md).

<!-- In troubleshooting.md -->
If you're experiencing namespace issues with ZED camera, see the [Camera Sensors - Troubleshooting](./camera-sensors.md#troubleshooting) section.
```

### Sensor-Specific Conventions

**Coordinate Frames:**
- Always explain coordinate transformations
- Reference ROS REP-103 for standard conventions
- Include diagrams when possible

**Network Configuration:**
- Specify exact IP addresses and ports
- Show how to verify network connectivity
- Document firewall/udev requirements

**Topic Naming:**
- Use Autoware standard naming: `/sensing/[sensor_type]/[sensor_name]/[data_type]`
- Explain topic remapping when drivers don't follow conventions
- Show topic verification commands

**Testing Commands:**
Always provide complete testing workflows:
```bash
# Build
colcon build --packages-select [package_name]

# Source
source install/setup.bash

# Launch
ros2 launch [package] [launch_file] [args]

# Verify
ros2 topic list | grep [sensor]
ros2 topic echo [topic_name]
rviz2  # Add visualization config
```

### Maintenance Guidelines

When updating sensor documentation:

1. **Keep it DRY**: Don't duplicate content between sensor guides
   - Common concepts → `overview.md`
   - Sensor-specific details → individual sensor guides
   - General troubleshooting → `troubleshooting.md`
   - Sensor-specific issues → sensor guide troubleshooting section

2. **Update all related sections**:
   - Sensor guide itself
   - Quick-start guide (if adding new model to table)
   - Adding-new-sensor guide (if pattern changes)
   - Troubleshooting guide (if new common issue)

3. **Test before documenting**:
   - Verify all commands work on target hardware
   - Confirm file paths are correct
   - Check that code snippets compile/run

4. **Version awareness**:
   - Note driver versions (e.g., "ZED SDK 5.1.2")
   - Document compatibility requirements
   - Update when dependencies change

### Images and Diagrams

**When to Include:**
- Hardware mounting and connections (photos)
- Coordinate frame transformations (diagrams)
- Architecture overview (system diagrams)
- Data flow (flowcharts)
- RViz visualization examples (screenshots)

**Image Guidelines:**
- Store in `src/figures/sensor-integration/`
- Use descriptive names: `robin-lidar-mounting.jpg`, `sensor-kit-architecture.svg`
- Optimize for web: compress photos, use SVG for diagrams
- Always include alt text: `![Robin-W LiDAR mounted on front dock](./figures/sensor-integration/robin-lidar-mounting.jpg)`

### Code Examples

**Complete, Not Snippets:**
Provide enough context to understand where code belongs:

```xml
<!-- File: src/sensor_kit/autosdv_sensor_kit_launch/launch/lidar.launch.xml -->
<launch>
  <!-- Robin-W LiDAR Configuration -->
  <group if="$(eval &quot;'$(var lidar_model)' == 'robin-w'&quot;)">
    <include file="...">
      <!-- Full working example -->
    </include>
  </group>
</launch>
```

**Parameter Documentation:**
Explain what each parameter does:

```yaml
# File: individual_params/config/default/autosdv_sensor_kit/robin_lidar.param.yaml
/**:
  ros__parameters:
    ip_address: "172.168.1.10"  # Robin-W default static IP
    scan_phase: 0.0              # Start angle offset (radians)
    frame_id: "robin_lidar_link" # TF frame name (matches URDF)
```

## Translation Workflow

The book supports Traditional Chinese (zh-TW) translation using a **manual AI-based workflow** with context-aware translation.

### Translation Commands
```bash
# Start new translation session (reset and split)
./scripts/translate_workflow.sh start

# Check translation progress
./scripts/translate_workflow.sh status

# Merge translated chunks back
./scripts/translate_workflow.sh merge

# Build the book with translations
./scripts/translate_workflow.sh build

# Clean up translation workspace
./scripts/translate_workflow.sh clean
```

### Translation Process
1. **Split**: PO file is split into 20-entry chunks in `/tmp/po_chunks/`
2. **Manual Translation**: AI translates each chunk with context awareness
3. **Merge**: Translated chunks are merged back into `po/zh-TW.po`
4. **Build**: Book is built with both English and Chinese versions

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
- Examples of consistent terms:
  - 感測器 (sensors)
  - 驅動程式 (drivers)
  - 車輛介面 (vehicle interface)
  - 映像檔 (image file)
  - 套件 (package)
  - 環景光達 (360° LiDAR)
  - 固態光達 (solid-state LiDAR)
  - 原始碼說明 (source code walkthrough)
  - 配線 (wiring)
  - 最佳化 (optimization)

**Taiwan-Specific Guidelines:**
- Use 固態光達型 instead of just 固態式 for clarity
- Use 環景光達 for 360° LiDAR instead of 360° 光達
- Translate Figure N. as 圖 N.
- Never use simplified Chinese characters (e.g., use 燒錄 not 烧錄)
- Keep parameter names in English (e.g., vehicle_model, not 車輛_模型)
- Don't translate file paths (e.g., keep AutoSDV/src/vehicle/ as is)
- Use Taiwan terminology (e.g., 程式 not 程序, 檔案 not 文件 for files)

**Sensor-Specific Translation Terms:**
- Sensor component/driver: 感測器驅動程式
- Sensor kit: 感測器套件
- Calibration: 校準
- Coordinate frame: 座標系
- Point cloud: 點雲
- Topic remapping: 主題重新映射
- Launch file: 啟動檔案
- ROS package: ROS 套件
- TF (transform): 座標轉換
- URDF (keep acronym): URDF
- IMU: 慣性測量單元 or IMU (both acceptable)
- GNSS: 全球導航衛星系統 or GNSS (both acceptable)
- LiDAR (keep term): LiDAR
- Stereo camera: 立體相機
- RTK: 即時動態定位 or RTK (both acceptable)

### Translation Scripts
- `translate_workflow.sh` - Main workflow orchestrator
- `split_for_manual_translation.py` - Splits PO file into chunks
- `merge_manual_translations.py` - Merges translated chunks
- `fix_po_formatting.py` - Fixes PO file formatting issues
- `fix_newline_mismatches.py` - Fixes newline mismatches between msgid and msgstr
- `comprehensive_retranslate.py` - Fixes common translation issues and typos
- `fix_partial_translations.py` - Fixes search-and-replace style partial translations
- `comprehensive_fix.py` - Applies comprehensive fixes to translations
- `taiwan_localization_fix.py` - Applies Taiwan-specific localization fixes

### Configuration
- Main book config: `book.toml` (English)
- Chinese book config: `book-zh-TW.toml`
  - Sets `language = "zh-TW"`
  - Sets `title = "AutoSDV 使用手冊"`
  - References `po-file = "po/zh-TW.po"`

### Important Notes
- Chunks are temporary and stored in `/tmp/po_chunks/`
- Always validate PO file after merging: `msgfmt --check po/zh-TW.po`
- The workflow uses context-aware translation, NOT search-and-replace
- Each chunk should be translated manually with understanding of context

### Common Translation Issues and Fixes
When reviewing translations, check for:
1. **Partial translations**: Mixed English/Chinese in sentences
2. **Typos**: e.g., 產產 should be 生產
3. **Simplified Chinese**: Should be Traditional Chinese only
4. **Path translations**: File paths should remain in English
5. **Parameter translations**: Launch parameters should stay in English
6. **Newline mismatches**: msgid and msgstr should have matching newlines

To fix issues comprehensively:
```bash
# Fix Taiwan-specific localization issues
python3 scripts/taiwan_localization_fix.py

# Fix newline mismatches if validation fails
python3 scripts/fix_newline_mismatches.py po/zh-TW.po

# Validate the PO file
msgfmt --check po/zh-TW.po

# Rebuild the book
./scripts/translate_workflow.sh build
```