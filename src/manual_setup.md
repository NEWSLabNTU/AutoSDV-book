# Manual Setup and Customization

This guide provides detailed manual installation steps and customization options for advanced users. For the recommended automated installation, see the [Software Installation](./software_installation.md) guide.

## Manual Environment Setup

### ROS 2 Humble Installation

If you prefer to install ROS 2 manually instead of using the Ansible script:

1. **Add ROS 2 apt repository:**
   ```bash
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install ROS 2 Humble:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop ros-dev-tools
   ```

3. **Configure environment:**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Autoware Installation Options

#### Option 1: Build Autoware from Source

For developers who want to modify Autoware components:

1. **Clone Autoware repository:**
   ```bash
   mkdir -p ~/autoware_ws
   cd ~/autoware_ws
   git clone https://github.com/autowarefoundation/autoware.git -b release/2025.02
   ```

2. **Install dependencies:**
   ```bash
   cd autoware
   ./setup-dev-env.sh
   sudo apt install python3-vcstool
   ```

3. **Import repositories and build:**
   ```bash
   mkdir src
   vcs import src < autoware.repos
   rosdep install -y --from-paths src --ignore-src --rosdistro humble
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **Source the workspace:**
   ```bash
   echo "source ~/autoware_ws/autoware/install/setup.bash" >> ~/.bashrc
   ```

#### Option 2: Custom Autoware Configuration

To use a custom Autoware installation path:

1. **Modify the build script:**
   ```bash
   export AUTOWARE_PATH=/your/custom/autoware/path
   ```

2. **Update AutoSDV's build configuration:**
   Edit `~/AutoSDV/clean_build.sh` and change the Autoware path:
   ```bash
   AUTOWARE_WS="/your/custom/autoware/path"
   ```

### Manual Sensor Driver Installation

#### Blickfeld Scanner Library

Manual installation for different architectures:

1. **Download the appropriate package:**
   - ARM64 (Jetson): [blickfeld-scanner-lib_2.20.6-1_arm64.deb](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_arm64.deb)
   - AMD64 (PC): [blickfeld-scanner-lib_2.20.6-1_amd64.deb](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_amd64.deb)

2. **Install the package:**
   ```bash
   sudo dpkg -i blickfeld-scanner-lib_*.deb
   sudo apt-get install -f  # Fix any dependency issues
   ```

#### Velodyne LiDAR Driver

For Velodyne VLP-32C support:

```bash
sudo apt install ros-humble-velodyne ros-humble-velodyne-pointcloud
```

#### GNSS/IMU Drivers

Install additional GNSS and IMU support:

```bash
# NMEA GPS support
sudo apt install ros-humble-nmea-navsat-driver

# Serial communication for IMU
sudo apt install ros-humble-serial-driver
pip3 install pyserial
```

## System Optimization

### Real-time Performance Tuning

1. **Configure CPU governor:**
   ```bash
   sudo apt install cpufrequtils
   echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
   sudo systemctl restart cpufrequtils
   ```

2. **Increase system limits:**
   Add to `/etc/security/limits.conf`:
   ```
   * soft memlock unlimited
   * hard memlock unlimited
   * soft rtprio 99
   * hard rtprio 99
   ```

3. **Disable CPU frequency scaling (Jetson):**
   ```bash
   sudo jetson_clocks
   sudo nvpmodel -m 0  # Maximum performance mode
   ```

### Network Configuration

#### Configure Cyclone DDS

1. **Create configuration file:**
   ```bash
   mkdir -p ~/.ros
   cat > ~/.ros/cyclone_dds.xml << EOF
   <?xml version="1.0" encoding="UTF-8" ?>
   <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
     <Domain>
       <General>
         <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
         <AllowMulticast>true</AllowMulticast>
       </General>
       <Discovery>
         <ParticipantIndex>auto</ParticipantIndex>
         <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
       </Discovery>
     </Domain>
   </CycloneDDS>
   EOF
   ```

2. **Set as default RMW:**
   ```bash
   echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
   echo "export CYCLONEDDS_URI=file://$HOME/.ros/cyclone_dds.xml" >> ~/.bashrc
   ```

#### Configure FastDDS (Alternative)

If you prefer FastDDS over Cyclone DDS:

```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Custom Build Options

### Debug Build

For development with debug symbols:

```bash
cd ~/AutoSDV
colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Selective Package Building

Build only specific packages:

```bash
# Build only vehicle interface packages
colcon build --packages-select \
  autosdv_vehicle_interface \
  autosdv_vehicle_launch

# Build with dependencies
colcon build --packages-up-to autosdv_launch
```

### Cross-compilation

For cross-compiling to different architectures:

```bash
# Example for ARM64 on x86_64 host
colcon build \
  --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release
```

## Environment Variables

### Essential Variables

Add to `~/.bashrc` for persistent configuration:

```bash
# ROS 2 Configuration
export ROS_DOMAIN_ID=0  # Change for multi-robot setups
export ROS_LOCALHOST_ONLY=0  # Set to 1 for local-only communication

# GPU Configuration (if applicable)
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# Autoware Configuration
export AUTOWARE_PATH=/opt/autoware  # Or your custom path
```

### Development Tools

Install additional development utilities:

```bash
# Code formatting and linting
sudo apt install clang-format python3-autopep8

# Debugging tools
sudo apt install gdb valgrind ros-humble-plotjuggler

# Build acceleration
sudo apt install ccache
export CC="ccache gcc"
export CXX="ccache g++"
```

## Troubleshooting Manual Installations

### Missing Dependencies

If encountering missing package errors:

```bash
# Update rosdep database
rosdep update

# Install missing dependencies
cd ~/AutoSDV
rosdep install --from-paths src --ignore-src -r -y
```

### Library Path Issues

For custom library installations:

```bash
# Add custom library paths
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/local.conf
sudo ldconfig
```

### Permission Issues

For device access (sensors, CAN bus):

```bash
# Add user to required groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
# Logout and login for changes to take effect
```

## Next Steps

- Return to [Software Installation](./software_installation.md) for the standard setup
- See [Docker Setup](./docker_setup.md) for containerized deployment
- Continue to [Development Guide](./development_guide.md) for development workflows