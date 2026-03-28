# Software Installation

This guide provides the complete installation workflow for AutoSDV. Follow these steps sequentially to set up a fully functional autonomous vehicle software stack.

## System Requirements

Choose one of the following platforms:

1. **NVIDIA Jetson AGX Orin 64GB** (Recommended for vehicle deployment)
2. **Ubuntu 22.04 PC with NVIDIA GPU** (For development and testing)
3. **Docker Environment** (For simulation and development)

### Storage Requirements

- Minimum 256GB SSD (NVMe recommended for Jetson)
- At least 100GB free space for software installation

## Installation Methods

Choose the installation method that best fits your needs:

| Method              | Best For                              | Difficulty | Customization |
|---------------------|---------------------------------------|------------|---------------|
| **Automatic Setup** | Most users, production deployment     | Easy       | Limited       |
| **Manual Setup**    | Advanced users, custom configurations | Advanced   | Full          |
| **Docker Setup**    | Development, testing, simulation      | Easy       | Limited       |

## Installation Workflow

### Step 1: Prepare Operating System

#### For NVIDIA Jetson AGX Orin

1. Download and install [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
2. Flash the Jetson with the following configuration:
   - **JetPack SDK version: 6.2.1**
   - Install all CUDA and TensorRT packages
   - Flash to external NVMe SSD (not internal eMMC)

#### For Ubuntu 22.04 PC

1. Install Ubuntu 22.04 LTS
2. Install NVIDIA drivers (version 550 or higher):
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-550
   ```
3. Install [CUDA 12.3](https://developer.nvidia.com/cuda-12-3-2-download-archive) using the deb (network) installer
4. Install [TensorRT 8.6 GA](https://developer.nvidia.com/nvidia-tensorrt-8x-download)

#### For Docker

See [Docker Setup](./docker.md) for containerized installation (skips Steps 1-3).

---

### Step 2: Install ZED SDK (if using ZED camera)

**ZED SDK must be installed manually before proceeding.**

The ZED SDK and ZED Link drivers are required if you're using ZED cameras. See [ZED SDK Installation Guide](./zed-sdk.md) for detailed instructions.

> **Note:** This is a manual installation step. The automated setup script does NOT install ZED SDK.

---

### Step 3: Install AutoSDV Software

Choose your installation method:

#### Automatic Setup (Recommended)

The automated setup script installs all dependencies and configures your system.

1. **Clone the repository:**
   ```bash
   cd ~
   git clone https://github.com/NEWSLabNTU/AutoSDV.git
   cd AutoSDV
   ```

2. **Run the automated setup:**
   ```bash
   ./setup.sh
   ```

This script automatically installs:
- ROS 2 Humble
- Autoware 1.5.0 Debian packages
- Isaac ROS Visual Localization (if NVIDIA GPU detected)
- Blickfeld Scanner Library (for Cube1 LiDAR, with license acceptance)
- Velodyne drivers (via rosdep)
- NMEA/serial drivers (via rosdep)
- All other ROS dependencies

The script first asks whether to install all optional components. If you decline, it prompts individually for:

- **Autoware Debian packages**: Install pre-built packages (~2-3 GB)
- **Isaac ROS Visual Localization**: Camera-only localization (requires NVIDIA GPU)
- **Blickfeld Scanner Library**: Accept license terms for Cube1 LiDAR support

After setup completes, it recommends optional post-setup steps:

- **CycloneDDS kernel buffers**: `./setup.sh cyclonedds-sysctl`
- **TurboVNC + VirtualGL**: `./setup.sh turbovnc-virtualgl`

**Note:** ZED SDK is NOT installed by this script and must be installed manually beforehand.

3. **Install and configure direnv:**

   AutoSDV uses direnv for automatic environment activation:

   ```bash
   # Install direnv
   sudo apt install direnv

   # Add to your shell (bash)
   echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
   source ~/.bashrc

   # Allow .envrc in AutoSDV directory
   cd ~/AutoSDV
   direnv allow
   ```

   After this, the ROS 2 and Autoware environments will automatically activate when you enter the AutoSDV directory.

#### Manual Environment Setup (Advanced)

Manually install ROS 2, Autoware, and dependencies for full control:

**See [Manual Environment Setup](./manual-environment.md) for detailed instructions**

#### Docker Setup (Alternative)

Run AutoSDV in a containerized environment:

**See [Docker Setup](./docker.md) for detailed instructions**

---

### Step 4: Build and Verify

After installing AutoSDV software, build the project:

```bash
cd ~/AutoSDV
just build    # Build the project
```

Verify the installation by launching the system:

```bash
just launch   # Launch AutoSDV system
```

If successful, you should see the system starting without critical errors.

## Troubleshooting

### Build Errors

If you encounter build errors:

```bash
# Clean and rebuild
just clean
just build
```

### Missing Dependencies

```bash
# Update rosdep database
rosdep update

# Install missing dependencies
cd ~/AutoSDV
rosdep install --from-paths src --ignore-src -r -y
```

### CUDA/TensorRT Issues

Verify CUDA installation:
```bash
nvcc --version  # Should show 12.3 or compatible
nvidia-smi      # Should show driver 550+
```

### ROS 2 Environment Issues

Always source ROS 2 before building:
```bash
source /opt/ros/humble/setup.bash
```

## Next Steps

After successful installation:

- [Operating the Vehicle](../usage.md) - Learn how to run AutoSDV
- [Development Guide](../../guides/development.md) - Start developing with AutoSDV

## Getting Help

If you encounter issues not covered here:
- Check the [AutoSDV GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
- Review the troubleshooting sections in each installation method page
