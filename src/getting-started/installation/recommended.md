# Recommended Installation

This guide walks through installing AutoSDV using the automated setup script. Make sure you have [prepared your operating system](./overview.md#prepare-operating-system) first.

## Clone the Repository

```bash
cd ~
git clone -b develop --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

## Run the Setup Script

```bash
./setup.sh
```

The script first asks whether to install all optional components. If you decline, it prompts individually for:

- **Autoware Debian packages**: Install pre-built packages (~2-3 GB)
- **Isaac ROS Visual Localization**: Camera-only localization (requires NVIDIA GPU)
- **Blickfeld Scanner Library**: Accept license terms for Cube1 LiDAR support

The following are always installed:

- ROS 2 Humble
- ROS 2 development tools (colcon, rosdep, vcstool)
- GeographicLib datasets
- Development tools (git-lfs, golang, pre-commit, plotjuggler)
- Python dependencies (Adafruit-PCA9685, simple-pid, play_launch)
- u-blox GPS udev rules
- All ROS dependencies via rosdep

After setup completes, it recommends optional post-setup steps:

- **CycloneDDS kernel buffers**: `./setup.sh cyclonedds-sysctl`
- **TurboVNC + VirtualGL**: `./setup.sh turbovnc-virtualgl`

> **Note:** ZED SDK is NOT installed by this script. Install it manually beforehand if you use ZED cameras. See [ZED SDK Installation](./zed-sdk.md).

## Install and Configure direnv

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

## Build and Verify

Build the project:

```bash
cd ~/AutoSDV
just build
```

Verify the installation by launching the system:

```bash
just launch
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
