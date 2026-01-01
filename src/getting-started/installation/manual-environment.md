# Manual Setup and Customization

This guide covers advanced configuration options and alternatives to the automated installation. **Most users should use the [automated setup](./overview.md)** instead.

## When to Use Manual Setup

Use manual setup if you need:
- **Custom Autoware**: Build Autoware from source to modify core components
- **Performance tuning**: Real-time optimization for production deployment
- **Custom DDS configuration**: Specific networking requirements
- **Development tools**: Additional debugging and profiling utilities
- **Non-standard installation paths**: Custom directory structures

## What the Automated Setup Does

Before proceeding, understand what `./setup.sh` already installs:

- ROS 2 Humble (ros-humble-desktop)
- ROS 2 development tools (colcon, rosdep, vcstool)
- Autoware 2025.02 Debian packages (optional, recommended)
- Blickfeld Scanner Library (for Cube1 LiDAR)
- Velodyne, NMEA, and serial drivers (via rosdep)
- Development tools: git-lfs, golang, pre-commit, clang-format, plotjuggler
- Python dependencies: Adafruit-PCA9685, simple-pid, ros2systemd
- u-blox GPS udev rules and user group permissions

The manual steps below provide alternatives or additions to this automated setup.

## Prerequisites

1. **Operating system prepared** (see [Step 1 in Overview](./overview.md#step-1-prepare-operating-system))
2. **ZED SDK 5.1 installed** if using ZED camera (see [ZED SDK Installation](./zed-sdk.md))

## Building Autoware from Source

The automated setup installs Autoware Debian packages from `/opt/autoware`. To modify Autoware core components, build from source instead:

### Step 1: Clone Autoware Repository

```bash
mkdir -p ~/autoware_ws/src
cd ~/autoware_ws
git clone https://github.com/autowarefoundation/autoware.git -b release/2025.02
```

### Step 2: Install Dependencies

```bash
cd autoware
./setup-dev-env.sh
```

This script installs:
- CUDA, cuDNN, TensorRT (if NVIDIA GPU detected)
- ROS 2 Humble and development tools
- GeographicLib and other Autoware dependencies

### Step 3: Import Repositories

```bash
mkdir -p src
vcs import src < autoware.repos
```

### Step 4: Install ROS Dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble
```

### Step 5: Build Autoware

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build time: 30-60 minutes depending on CPU cores and whether you're building perception packages.

### Step 6: Source the Workspace

```bash
echo "source ~/autoware_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Using Custom Autoware Path

If you build Autoware in a custom location, AutoSDV will automatically detect it if you source the Autoware workspace before building AutoSDV.

## System Optimization

### Real-time Performance Tuning

For production deployment, configure system for real-time performance:

**1. Configure CPU governor:**
```bash
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

**2. Increase system limits:**

Add to `/etc/security/limits.conf`:
```
* soft memlock unlimited
* hard memlock unlimited
* soft rtprio 99
* hard rtprio 99
```

**3. Jetson-specific optimizations:**
```bash
# Lock clocks to maximum performance
sudo jetson_clocks

# Set performance mode (0 = MAXN)
sudo nvpmodel -m 0
```

## ROS 2 Middleware Configuration

### Cyclone DDS Configuration (Recommended)

Cyclone DDS is the recommended middleware for Autoware. Follow these steps for optimal configuration:

**1. Configure kernel network buffers:**

```bash
# Apply immediately
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

# Make persistent across reboots
sudo tee /etc/sysctl.d/10-cyclone-max.conf > /dev/null << EOF
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
```

**2. Create CycloneDDS configuration file:**

Save as `~/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config
  https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" name="lo"
          priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

**Note:** The `name="lo"` configuration uses the loopback interface for single-machine operation. For multi-machine setups, change to your network interface (e.g., `eth0`, `wlan0`, or `enp0s31f6`). Use `ip link show` to list available interfaces.

**3. Set environment variables:**

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
```

**4. Optional - Set ROS Domain ID:**

For multi-vehicle setups, use different domain IDs (1-255) for each vehicle:

```bash
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
```

**Important:** Do NOT set `ROS_LOCALHOST_ONLY=1` - it has known compatibility issues with Autoware.

### FastDDS Configuration (Alternative)

FastDDS is an alternative middleware. Note: Autoware documentation primarily supports CycloneDDS.

```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
```

## Custom Build Options

### Debug Build

Build AutoSDV with debug symbols for development:

```bash
cd ~/AutoSDV
colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Selective Package Building

Build only specific packages to save time:

```bash
# Build only vehicle interface packages
colcon build --packages-select \
  autosdv_vehicle_interface \
  autosdv_vehicle_launch

# Build package and all its dependencies
colcon build --packages-up-to autosdv_launch
```

### Cross-compilation

For cross-compiling to Jetson on x86_64 host:

```bash
colcon build \
  --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/jetson-toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release
```

## Additional Development Tools

The automated setup installs basic development tools. Add more for advanced development:

```bash
# Code quality tools
sudo apt install python3-autopep8 cppcheck

# Debugging and profiling
sudo apt install gdb valgrind heaptrack

# Build acceleration (can reduce build time by 50%)
sudo apt install ccache
echo 'export CC="ccache gcc"' >> ~/.bashrc
echo 'export CXX="ccache g++"' >> ~/.bashrc

# Advanced visualization
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
```

## Environment Variables

Useful environment variables for development:

```bash
# CUDA paths (if not using Debian Autoware)
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
```

Add these to `~/.bashrc` for persistence.

**Note:** `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` are configured in the ROS 2 Middleware Configuration section above.

## Troubleshooting Manual Installations

### Missing ROS Dependencies

If you encounter missing package errors after manual installation:

```bash
# Update rosdep database
rosdep update --rosdistro=humble

# Install missing dependencies
cd ~/AutoSDV
rosdep install --from-paths src --ignore-src -r -y
```

### Library Path Issues

For libraries installed to non-standard paths:

```bash
# Add custom library path
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/local.conf
sudo ldconfig

# Verify library is found
ldconfig -p | grep <library_name>
```

### Permission Issues

For device access (USB sensors, CAN bus, serial ports):

```bash
# Add user to required groups
sudo usermod -aG dialout $USER    # Serial ports, GPS
sudo usermod -aG plugdev $USER    # USB devices
sudo usermod -aG video $USER      # Cameras

# Log out and back in for changes to take effect
```

### Build Failures

If colcon build fails:

```bash
# Clean build artifacts
rm -rf build install log

# Try building with verbose output
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Build with single thread to see errors clearly
colcon build --executor sequential
```

## Verification

After manual setup, verify your installation:

```bash
# Check ROS 2 installation
ros2 --version

# Check Autoware installation (Debian)
dpkg -l | grep autoware

# Check Autoware installation (source build)
source ~/autoware_ws/install/setup.bash
ros2 pkg list | grep autoware

# Check sensor drivers
ros2 pkg list | grep velodyne
ros2 pkg list | grep nmea

# Test AutoSDV build
cd ~/AutoSDV
colcon test --packages-select autosdv_launch
```

## Next Steps

- Return to [Software Installation Overview](./overview.md) to complete the standard workflow
- See [Usage Guide](../usage.md) for launching and operating the system
- Check [Development Guide](../../guides/development.md) for development workflows
