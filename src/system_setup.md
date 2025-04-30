# Driving Software Installation

## Prepare the Development Environment

**Jetson Linux 36.3.0 for Jetson AGX Orin 64G** is the preferred
choice. Flash using [NVIDIA SDK
Manager](https://developer.nvidia.com/sdk-manager). Ensure CUDA and
TensorRT are enabled during flashing.

For installations on a PC or laptop, use **Ubuntu 22.04**. Manually
install **CUDA 12.3** and **TensorRT 8** in advance.

## Prerequisites

Install the following dependent packages on your Orin box.

- ROS 2 Humble

  Please follow the official installation
  [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

- Blickfeld Scanner Library 2.20.6
  
  Install the prebuilt Debian package:
  [amd64](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_amd64.deb),
  [arm64](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_arm64.deb).

- ZED SDK 4.0.8

  This is required for the ZED X mini camera. Please visit the
  official [download
  page](https://www.stereolabs.com/developers/release) and download
  the _ZED SDK for Ubuntu 22 4.0.8_ version.

## Install Autoware

It is recommended to download and install the binary package from
NEWSLab Releases
[here](https://github.com/NEWSLabNTU/autoware/releases/tag/rosdebian%2F2025.02-1).
For Orin users, download
[`autoware-localrepo_2025.2-1_jetpack6.0.deb`](https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_jetpack6.0.deb)
and follow the installation instructions on that page.

## Build the AutoSDV Project

Download the project repository. The option `-b 2025.02` ensures the
correct version.

```sh
git clone -b 2025.02 --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

Configure the system.

```sh
make setup
```

This is a meta step for all following commands. Always enable ROS
environment whenever you start a new shell.

```bash
source /opt/ros/humble/setup.bash
make prepare
make build
```

When the project is successfully built, activate the development
environment.

```sh
source install/setup.sh
```
