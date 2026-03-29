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

| Method                                                      | Best For                              | Difficulty | Customization |
|-------------------------------------------------------------|---------------------------------------|------------|---------------|
| **[Recommended Installation](./recommended.md)**            | Most users, production deployment     | Easy       | Limited       |
| **[Manual Environment Setup](./manual-environment.md)**     | Advanced users, custom configurations | Advanced   | Full          |
| **[Docker Setup](./docker.md)**                             | Development, testing, simulation      | Easy       | Limited       |

## Prepare Operating System

Before installing AutoSDV, prepare your target platform.

### For NVIDIA Jetson AGX Orin

1. Download and install [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
2. Flash the Jetson with the following configuration:
   - **JetPack SDK version: 6.2.1**
   - Install all CUDA and TensorRT packages
   - Flash to external NVMe SSD (not internal eMMC)

### For Ubuntu 22.04 PC

1. Install Ubuntu 22.04 LTS
2. Install NVIDIA drivers (version 550 or higher):
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-550
   ```
3. Install [CUDA 12.3](https://developer.nvidia.com/cuda-12-3-2-download-archive) using the deb (network) installer
4. Install [TensorRT 8.6 GA](https://developer.nvidia.com/nvidia-tensorrt-8x-download)

### For Docker

See [Docker Setup](./docker.md) for containerized installation. This skips the OS preparation steps above.

## Install ZED SDK (if using ZED camera)

**ZED SDK must be installed manually before proceeding.**

The ZED SDK and ZED Link drivers are required if you're using ZED cameras. See [ZED SDK Installation Guide](./zed-sdk.md) for detailed instructions.

> **Note:** This is a manual installation step. The automated setup script does NOT install ZED SDK.

## Next Step

Once your operating system is prepared, proceed to the [Recommended Installation](./recommended.md) guide.
