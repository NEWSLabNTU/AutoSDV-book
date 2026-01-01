# ZED SDK Installation

The ZED SDK provides drivers and APIs for ZED stereo cameras and ZED Link capture cards. This is a **manual installation step** required before running the AutoSDV setup script.

## When Do You Need This?

Install ZED SDK if you're using:
- ZED X Mini camera (standard AutoSDV configuration)
- ZED 2/2i cameras
- ZED Link capture cards (Mono/Dual/Quad)

Skip this step if you're using only LiDAR sensors (Velodyne, Blickfeld, Robin-W).

## Prerequisites

- Ubuntu 22.04 with NVIDIA drivers (550+) installed, OR
- Jetson AGX Orin with JetPack 6.0

## Installation Steps

### Step 1: Download ZED SDK 5.1

Visit the [ZED SDK Downloads](https://www.stereolabs.com/developers/release) page.

Download the appropriate installer:

**For Jetson AGX Orin (JetPack 6.0):**
- [ZED SDK 5.1 for JetPack 6.0](https://download.stereolabs.com/zedsdk/5.1/l4t36.3/jetsons)

**For Ubuntu 22.04 PC:**
- [ZED SDK 5.1 for Ubuntu 22 + CUDA 12](https://download.stereolabs.com/zedsdk/5.1/cu124/ubuntu22)

### Step 2: Install ZED SDK

```bash
# Make the installer executable
chmod +x ZED_SDK_*.run

# Run the installer
sudo ./ZED_SDK_*.run
```

Follow the on-screen prompts:
- Accept the license agreement
- Install all components (SDK, tools, Python API, samples)
- Allow AI model downloads (required for object detection)

Installation takes 10-20 minutes depending on internet speed.

### Step 3: Verify Installation

```bash
# Run ZED diagnostic tool
/usr/local/zed/tools/ZED_Diagnostic

# Expected output should show:
# - ZED SDK Version: 5.1.x
# - CUDA version detected
# - Camera detection status (if connected)
```

## ZED Link Driver Installation (Optional)

If you're using ZED Link capture cards for multi-camera setups, install the appropriate driver:

### Identify Your ZED Link Model

- **ZED Link Mono**: Single camera input
- **ZED Link Dual**: Two camera inputs
- **ZED Link Quad**: Four camera inputs

### Download and Install

1. Visit [Stereolabs Download Center](https://www.stereolabs.com/developers/release)
2. Download the ZED Link driver for your model and Ubuntu version
3. Install the debian package:

```bash
# For ZED Link Mono
sudo dpkg -i zed-link-mono_*.deb

# For ZED Link Dual
sudo dpkg -i zed-link-dual_*.deb

# For ZED Link Quad
sudo dpkg -i zed-link-quad_*.deb
```

4. Verify installation:

```bash
# Check if ZED Link is detected
lspci | grep -i stereolabs

# Should show PCIe device for your ZED Link model
```

## Troubleshooting

### CUDA Not Found

If the installer cannot find CUDA:

```bash
# Verify CUDA installation
nvcc --version
nvidia-smi

# CUDA should be 12.3 or compatible
# If not installed, return to Step 1 of the main installation guide
```

### Python Dependencies

The installer may install Python packages. To avoid conflicts:

```bash
# After installation, verify numpy location
python3 -c "import numpy; print(numpy.__file__)"

# Should be in /home/user/.local or /usr/lib
# NOT in /usr/local (which can cause conflicts)
```

### Camera Not Detected

If ZED camera is not detected after installation:

```bash
# Check USB connection
lsusb | grep -i stereo

# Add user to video group
sudo usermod -aG video $USER

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes to take effect
```

### Permission Errors

```bash
# Fix ZED SDK directory permissions
sudo chown -R root:root /usr/local/zed
sudo chmod -R 755 /usr/local/zed

# Fix calibration directory permissions
sudo chmod 777 /usr/local/zed/settings
```

## Next Steps

After successfully installing ZED SDK:

- Return to [Software Installation Overview](./overview.md#step-3-install-autosdv-software)
- Continue with automatic or manual setup
- Proceed to build and verify AutoSDV
