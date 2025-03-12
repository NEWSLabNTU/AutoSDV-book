# System Installation

## Download This Project

Download the project repository.

```sh
git clone -b 2024.11 --recursive https://github.com/NEWSLabNTU/F1EIGHTH.git
cd F1EIGHTH
```

## Prepare the Operating System

**Jetson Linux 36.3.0** on Jeton AGX Orin 64G is preferred. You can
flash the Jetson box using [NVIDIA SDK
manager](https://developer.nvidia.com/sdk-manager). Please make sure
CUDA and TensorRT installation are enabled before flashing the box.

If you're install on a PC or a laptop, Ubuntu 22.04 is preferred. You
have to install CUDA 12.3 and TensorRT 8 manually in anticipation.

## Environment Setup

### The Recommended Way

The project ships an Ansible playbook that configures the environment
automatically. Run this command and it does the job for you.

```sh
make setup
```

### The Manual Way

If you prefer to configure environment manually, please install the
following packages.

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


## Build Autoware

This is a meta step for all following commands. Always enable ROS
environment whenever you start a new shell.

```bash
source /opt/ros/humble/setup.bash
```

Install required dependencies.

```bash
make prepare
```

Build the whole project.

```bash
make build
```

When the project is successfully built, activate the development
environment.

```sh
source install/setup.sh
```
