# Driving Software Installation

## Prepare the Onboard Computer

### Recommended: NVIDIA Jetson AGX Orin

NVIDIA Jetson AGX Orin 64G is the major platform for the onboard
computer. Flash the Jetson box using [SDK
manager](https://developer.nvidia.com/sdk-manager) with the following
configuration.

- **JetPack SDK** with exact version **6.0**.

    > Note that newer releases such as 6.1 and 6.2 are not compatible.

- Install all CUDA and TensorRT packages in the SDK manager window.
- Flash the system on the external NVMe SSD disk with size at least
  256GB .

    > It's not recommended to boot on the builtin EMMC due to limited
    > capacity.

### Alternative: Ubuntu 22.04

The fresh Ubuntu 22.04 operating system with the following
dependencies is preferable.

-  Visit the [CUDA
  Archive](https://developer.nvidia.com/cuda-12-3-2-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network)
  and install **CUDA 12.3** with "deb (network)" installer type.

- NVIDIA driver 550 or above is recommended.

- Proceed the [download
  page](https://developer.nvidia.com/nvidia-tensorrt-8x-download) and
  install TensorRT 8.6 GA.


## Prepare the Environment (Recommended Method)

### Step 1: Run Ansible Automated Setup Script

The project ships an Ansible playbook that configures the environment
automatically. The following tasks are done during the process.

- Install ROS Humble.
- Install Autoware 2025.02 binary release and its dependencies.
- Install Blickfeld Scanner Lib required by the Blickfeld Cube 1 LiDAR.
- Download artifacts for Autoware.
- Set default RMW library to Cyclone DDS and optimize system-wide
  settings.

```sh
make setup
```

### Step 2: Install Sensor Drivers

Installation for following packages still need manual manipulation.

- Visit the [release
  archive](https://www.stereolabs.com/en-tw/developers/release/4.2)
  and install **ZED SDK 4.2**
  - NVIDIA AGX Orin users install the [ZED SDK for JetPack 6.0
    GA](https://download.stereolabs.com/zedsdk/4.2/l4t36.3/jetsons)
    release.
  - Ubuntu 22.04 PC/laptop users install the [ZED SDK for Ubuntu 22
    ](https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22)
    release.

- Install the `innovusion` ROS driver for Seyond Robin-W LiDAR. You
  may contact the LiDAR vendor to obtain the Debian package.

## Prepare the Environment Manually (If Not Using the Recommended Method)

### Step 1: Install ROS Humble

Visit this
[guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
and install **ROS Humble**. Please install both `ros-humble-desktop`
and `ros-dev-tools`.

### Step 2: Install Sensor Drivers and SDKs

- **Blickfeld Scanner Library 2.20.6**

  - NVIDIA AGX Orin users Install the
    [amd64](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_arm64.deb)
    release.
  - PC/laptop users install the
    [amd64](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_amd64.deb)
    release.

- Visit the [release
  archive](https://www.stereolabs.com/en-tw/developers/release/4.2)
  and install **ZED SDK 4.2**
  - NVIDIA AGX Orin users install the [ZED SDK for JetPack 6.0
    GA](https://download.stereolabs.com/zedsdk/4.2/l4t36.3/jetsons)
    release.
  - Ubuntu 22.04 PC/laptop users install the [ZED SDK for Ubuntu 22
    ](https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22)
    release.

- Install the `innovusion` ROS driver for Seyond Robin-W LiDAR. You
  may contact the LiDAR vendor to obtain the Debian package.

### Step 3: Install Autoware

**Method 1: Debian Binary Release**

Check Autoware 2025.02 [binary
release](https://github.com/NEWSLabNTU/autoware/releases/tag/rosdebian%2F2025.02-1).
It performs automated system configuration.

After the installation is complete, activate the development
environment.

```sh
source /opt/autoware/autoware-env
```


**Method 2: Build from Source**

Visit the official
[tutorial](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
and build the Autoware step by step.

After the installation is complete, activate the development
environment.

```bash
source ~/autoware/install/setup.bash
```

## Build the AutoSDV Project

Download the AutoSDV source repository.

```sh
git clone -b 2025.02 --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

Assume that Autoware development environment is activated. Build the
project in the following steps.

```bash
make prepare
make build
```

After the project is successfully built, activate the development
environment.

```sh
source install/setup.sh
```
