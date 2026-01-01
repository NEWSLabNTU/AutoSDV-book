# Software Installation

This guide provides the recommended installation process for AutoSDV. For manual installation or customization options, see the [Manual Setup Guide](./manual-setup.md).

## System Requirements

### Hardware Options

Choose one of the following platforms:

1. **NVIDIA Jetson AGX Orin 64GB** (Recommended for vehicle deployment)
2. **Ubuntu 22.04 PC with NVIDIA GPU** (For development and testing)
3. **Docker Environment** (For simulation and development) - See [Docker Setup](./docker-setup.md)

### Storage Requirements

- Minimum 256GB SSD (NVMe recommended for Jetson)
- At least 100GB free space for software installation

## Installation Steps

### Step 1: Prepare the Operating System

#### For NVIDIA Jetson AGX Orin

1. Download and install [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
2. Flash the Jetson with the following configuration:
   - **JetPack SDK version: 6.0** (exactly - not 6.1 or 6.2)
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

### Step 2: Clone AutoSDV Repository

```bash
cd ~
git clone https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

### Step 3: Run Automated Setup

The project includes an Ansible playbook that automatically configures your environment:

```bash
make setup
```

This script will:
- Install ROS 2 Humble
- Install Autoware 2025.02 binary release
- Install Blickfeld Scanner Library for Cube 1 LiDAR
- Download required Autoware artifacts
- Configure Cyclone DDS as default RMW
- Optimize system settings for real-time performance

### Step 4: Install Additional Sensor Drivers

Some sensor drivers require manual installation:

#### ZED Camera SDK

1. Visit the [ZED SDK 4.2 Release Page](https://www.stereolabs.com/en-tw/developers/release/4.2)
2. Download and install:
   - **For Jetson AGX Orin**: [ZED SDK for JetPack 6.0 GA](https://download.stereolabs.com/zedsdk/4.2/l4t36.3/jetsons)
   - **For Ubuntu 22.04 PC**: [ZED SDK for Ubuntu 22](https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22)

```bash
# After downloading the installer
chmod +x ZED_SDK_*.run
sudo ./ZED_SDK_*.run
```

#### Seyond Robin-W LiDAR Driver (Optional)

If using the Robin-W LiDAR, install the Innovusion ROS driver:
- Contact Seyond/Innovusion to obtain the driver package
- Install the provided `.deb` package

### Step 5: Build AutoSDV

```bash
cd ~/AutoSDV
make prepare  # Install ROS dependencies
make build    # Build the project
```

### Step 6: Verify Installation

Test the installation by launching the system:

```bash
source install/setup.bash
ros2 launch autosdv_launch autosdv.launch.yaml
```

## Next Steps

- [Operating the Vehicle](./usage.md) - Learn how to run AutoSDV
- [Development Guide](../guides/development.md) - Start developing with AutoSDV
- [Docker Setup](./docker-setup.md) - Alternative Docker-based installation
- [Manual Setup Guide](./manual-setup.md) - Manual installation and customization options

## Troubleshooting

### Build Errors with Autoware Dependencies

If you encounter missing Autoware package errors during build:

1. Ensure Autoware is properly installed:
   ```bash
   source /opt/autoware/autoware-env
   ```

2. Use the clean build script:
   ```bash
   ./clean_build.sh clean
   ```

### CUDA/TensorRT Issues

Verify CUDA installation:
```bash
nvcc --version  # Should show 12.3
nvidia-smi      # Should show driver 550+
```

### ROS 2 Environment Issues

Always source ROS 2 before building:
```bash
source /opt/ros/humble/setup.bash
```