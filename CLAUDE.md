# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- Setup: `make setup` - Installs MkDocs and all dependencies
- Build: `make build` - Compiles the HTML documentation using MkDocs (both English and Chinese versions)
- Serve: `make serve` - Starts development server at http://localhost:8000 with live reload
- Clean: `make clean` - Removes all build outputs (site/ directory)
- Check: `make check` - Verifies all dependencies are installed
- Deploy: `make deploy` - Deploys to GitHub Pages

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
- Update `src/SUMMARY.md` when adding new content files
- Use relative paths for internal links (adjust based on file location depth)
- Translation files are in `po/` directory (zh-TW.po for Traditional Chinese)
- Translation scripts are in `scripts/` directory
- Language-specific config: `book-zh-TW.toml` for Chinese version

## Vehicle Control Documentation Guidelines

The vehicle control guides follow a **logical organization**: overview → hardware → algorithms → practice.

### Documentation Philosophy

**Learning Path:**
1. **Overview** - Understand system architecture and integration
2. **Hardware** - Know the physical components and wiring
3. **Control Details** - Learn how algorithms work
4. **Tuning & Testing** - Apply knowledge in practice

**Key Principles:**
- ✅ Safety first - Emphasize controller GUI over manual commands
- ✅ Practical over theoretical - Show working procedures
- ✅ Accurate implementation details - Based on actual README.md from source code
- ✅ No duplication - Each concept appears in exactly one place
- ❌ No dangerous examples - Never show `ros2 topic pub` for control commands

### Content Structure

The vehicle control documentation consists of four files:

#### 1. overview.md (Architecture & Dataflow)
- System architecture diagram
- Software components (Actuator Node, Velocity Report Node)
- Autoware integration (topics, control modes)
- Safety features (watchdog, emergency stop)
- Hardware summary table
- **NO**: Hardware details, algorithm details, testing procedures

**Purpose**: Understand the big picture and how components interact.

#### 2. hardware.md (Hardware Details)
- PCA9685 PWM driver (I2C, wiring, device detection)
- Motor ESC (PWM mapping, power requirements)
- Steering servo (PWM mapping, power requirements)
- Hall effect sensor (KY-003, GPIO, magnet setup)
- IMU sensor (role in steering feedback)
- Complete wiring diagram and connection tables
- **NO**: Control algorithms, testing procedures

**Purpose**: Physical specifications and electrical connections.

#### 3. control-details.md (Control Algorithms)
- Multi-mode longitudinal controller (4 modes with detailed algorithms)
- Dual-mode lateral controller (2 modes with detailed algorithms)
- Velocity calculation from hall effect sensor
- I2C communication protocol
- Parameter descriptions
- **Source**: Based on `autosdv_vehicle_interface/README.md`
- **NO**: Testing procedures, tuning guidelines

**Purpose**: Technical reference for understanding how control works.

#### 4. tuning-and-testing.md (Practical Testing)
- Quick test procedures with PlotJuggler
- Manual control using controller GUI (safe)
- Automated test scenarios using `control_test` package
- PID tuning guidelines and effects
- Calibration procedures (PWM, steering, hall effect sensor)
- Plot interpretation
- Troubleshooting
- **Safety**: Always use controller GUI, never manual topic publishing
- **NO**: Algorithm details, hardware wiring

**Purpose**: Practical guide for testing, tuning, and calibrating.

### Key Technical Details to Include

**Longitudinal Control - Multi-Mode Controller**:
- Emergency Brake Mode (target ≈ 0, measured > threshold)
- Full Stop Mode (both ≈ 0)
- Deadband Hold Mode (error < deadband)
- Active Control Mode (PID with filtering, anti-windup, direction mapping)

**Lateral Control - Dual-Mode Controller**:
- Fallback Mode (v < 0.3 m/s): Open-loop feedforward
- Normal Mode (v ≥ 0.3 m/s): Yaw rate feedback with Ackermann + PID

**Hardware Details**:
- PCA9685: I2C address 0x40, bus 1, 60 Hz PWM
- Motor PWM: 280-460 (init 370, brake 340)
- Steering PWM: 350-450 (init 400, max angle 0.349 rad)
- Hall effect sensor: KY-003 on GPIO

### Writing Style

**Be Safe**:
- Always warn against manual `ros2 topic pub` commands
- Emphasize controller GUI (`make run-controller`)
- Use `control_test` package for automated tests
- Include safety warnings where appropriate

**Be Accurate**:
- Base technical details on actual README.md files
- Don't guess at algorithm implementation
- Include actual parameter values from config files
- Reference specific file paths

**Test Everything**:
- Every procedure should be tested and working
- Provide verification commands
- Show expected output

### Maintenance Guidelines

When updating vehicle control documentation:

1. **Check source code first**: Always verify implementation details in `autosdv_vehicle_interface/README.md`
2. **Update appropriate file**:
   - Architecture changes → `overview.md`
   - Hardware changes (wiring, specs) → `hardware.md`
   - Algorithm changes → `control-details.md`
   - Testing/tuning changes → `tuning-and-testing.md`
3. **Maintain safety warnings**: Never remove warnings about manual control (especially in tuning-and-testing.md)
4. **Keep parameters current**: Check actual `.yaml` files for parameter values
5. **No duplication**: Each concept should appear in exactly one file
6. **Update cross-references**: If you move content, update links in other files

## Sensor Documentation Guidelines

The sensor integration guides follow a **simple-to-complex philosophy**: start with usage, learn through concrete examples, then explore sensor-specific details.

### Documentation Philosophy

**Learning Path:**
1. **Start Simple** - Show users how to use sensors without technical details
2. **Learn by Example** - Deep dive into one real sensor (Robin-W) to build understanding
3. **Sensor Specifics** - Concise guides focusing on what's unique about each sensor

**Before**: Abstract system → Generic integration → Sensor details
**After**: Simple usage → Concrete example → Sensor specifics

**Key Principles:**
- ✅ Concise over comprehensive - Remove unnecessary detail
- ✅ Practical over theoretical - Show working examples, not abstractions
- ✅ Specific over generic - Focus on what's unique to each sensor
- ❌ No repeated configuration patterns - Reference the walkthrough instead

### Content Structure

The sensor integration documentation consists of:

#### 1. using-sensors.md (Start Simple)
- Sensor suites table with launch commands
- Quick verification steps (check topics, rates)
- Common scenarios (outdoor RTK, indoor no-GPS, Isaac SLAM)
- **NO**: File paths, configuration details, URDF/YAML examples

**Purpose**: Get users running quickly without overwhelming detail.

#### 2. integration-walkthrough.md (Learn by Example)
- Complete walkthrough using Robin-W LiDAR as concrete example
- Covers: hardware → driver → configuration → data flow
- Explains coordinate transformation (why roll=180°, pitch=-90°)
- Shows complete picture of sensor integration
- **NO**: Generic templates or "add your own sensor" instructions

**Purpose**: Build deep understanding through one real example.

#### 3. Sensor-Specific Guides (lidar.md, camera.md, imu.md, gnss.md)
Each guide should be **concise** and focus **only** on sensor-specific details:

```markdown
# [Sensor Type] Sensors

## Supported Models
[Table with key specs and config values]

## [Model 1]
### Network/Hardware Setup
[IP addresses, connections, udev rules - sensor-specific only]

### Coordinate System
[Transformation if non-standard, otherwise just note "standard ROS"]

### Driver Package
[Location, point format, special features]

### Test Standalone
[Minimal verification commands]

### [Special Configuration]
[Only if sensor has unique features - e.g., ZED namespace workaround, RTK/NTRIP]

## [Model 2]
[Repeat concisely]

## Quick Reference
[Network tests, topic verification commands]
```

**What to INCLUDE**:
- Network addresses (e.g., Robin-W: 172.168.1.10)
- Coordinate transformations if non-standard (e.g., Robin-W roll/pitch)
- Driver-specific features (e.g., ZED composable node workaround, ZED IMU relay, u-blox RTK/NTRIP)
- Hardware-specific setup (e.g., I2C for MPU9250, udev rules for u-blox)

**What to REMOVE**:
- Generic URDF/launch/calibration templates (covered in walkthrough)
- Repeated explanations of sensor kit integration
- File path references to every configuration file
- Long explanations of concepts (keep it concise)

#### 4. adding-sensor.md (Quick Checklist)
- Simplified checklist with minimal examples
- References integration-walkthrough.md for detailed understanding
- Common issues section
- **NO**: Full step-by-step tutorial (that's in the walkthrough)

**Purpose**: Quick reference for experienced users.

### Writing Style

**Be Concise:**
- Remove unnecessary detail - users can dive deeper if needed
- ❌ "The sensor_kit_calibration.yaml file, located in the src/sensor_kit/autosdv_sensor_kit_description/config directory, contains..."
- ✅ "Configuration: `sensor_kit_calibration.yaml`"

**Be Specific When It Matters:**
- ❌ "Configure the network"
- ✅ "Robin-W IP: 172.168.1.10, Jetson IP: 172.168.1.100/24"

**Show Working Examples:**
- Include minimal working code snippets
- Focus on sensor-specific values
- Avoid boilerplate that applies to all sensors

**Explain Why (For Unique Cases):**
- Explain unusual configurations (e.g., Robin-W coordinate transform)
- Skip explanations for standard patterns
- Example: "Robin-W requires roll=180°, pitch=-90° because it uses X=up, Y=right, Z=forward instead of ROS standard"

**Test Everything:**
- Every code snippet should be tested and working
- Provide verification commands
- Show expected output

### Cross-References

Create clear navigation between guides:

```markdown
<!-- In using-sensors.md -->
Learn how sensors work: [Integration Walkthrough](./integration-walkthrough.md)

<!-- In integration-walkthrough.md -->
For other sensors: [LiDAR](./lidar.md), [Camera](./camera.md), [IMU](./imu.md), [GNSS](./gnss.md)

<!-- In lidar.md -->
See [Integration Walkthrough](./integration-walkthrough.md) for complete integration example

<!-- In adding-sensor.md -->
Understand the system first: [Integration Walkthrough](./integration-walkthrough.md)
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

1. **Keep it DRY**: Don't duplicate content between guides
   - Integration concepts → `integration-walkthrough.md` (Robin-W example)
   - Sensor-specific details → individual sensor guides (`lidar.md`, `camera.md`, etc.)
   - Usage patterns → `using-sensors.md`
   - General troubleshooting → `troubleshooting.md`
   - **Never repeat**: URDF templates, launch patterns, calibration file structure

2. **Update all related sections when adding a sensor**:
   - Add to supported models table in `using-sensors.md`
   - Create entry in relevant sensor guide (`lidar.md`, `camera.md`, etc.)
   - Update `troubleshooting.md` if new common issues
   - **Consider**: Using new sensor as walkthrough example if it's particularly interesting

3. **Test before documenting**:
   - Verify all commands work on target hardware
   - Test verification commands show expected output
   - Keep examples minimal but complete

4. **Version awareness**:
   - Note critical versions (e.g., "ZED SDK 5.1.2 required")
   - Document compatibility requirements only when relevant
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

The documentation supports Traditional Chinese (zh-TW) translation using MkDocs i18n plugin.

### Translation System

MkDocs uses **suffix-based i18n** where Chinese translations are stored in markdown files with `.zh-TW.md` suffix:
```
src/
├── index.md              # English version
├── index.zh-TW.md        # Chinese version
├── introduction.md       # English
├── introduction.zh-TW.md # Chinese
└── ...
```

### Creating Translations

**Option 1: Manual Translation**
1. Copy English file with `.zh-TW.md` suffix:
   ```bash
   cp src/introduction.md src/introduction.zh-TW.md
   ```
2. Translate content in the `.zh-TW.md` file
3. Rebuild: `make build`

**Option 2: Automated (Recommended)**
Use the existing translation scripts (adapted for MkDocs):
```bash
# Convert existing PO translations to MkDocs format
python3 scripts/po_to_mkdocs.py

# Build with translations
make build
```

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

### Nav Translations

Navigation titles are translated in `mkdocs.yml` under the `i18n.languages.zh-TW.nav_translations` section. Update this section when adding new pages.

### Configuration

- Main config: `mkdocs.yml`
- Language settings: `plugins.i18n.languages` section
- Navigation translations: `plugins.i18n.languages.zh-TW.nav_translations`

### Important Notes

- Chinese files must have `.zh-TW.md` suffix
- Both language versions are built into `site/` directory
- Access via `http://localhost:8000/en/` (English) or `http://localhost:8000/zh-TW/` (Chinese)