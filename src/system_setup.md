# System Setup

## Download the Project

Download the F1EIGHTH project repository on GitHub.

```sh
git clone --recursive https://github.com/NEWSLabNTU/F1EIGHTH.git
cd F1EIGHTH
```

## Environment

### Operating System

**Jetson Linux 36.3** on Jeton AGX Orin 64G is the preferred
environment. You can flash the Jetson box using [NVIDIA SDK
manager](https://developer.nvidia.com/sdk-manager). Please make sure
CUDA and TensorRT are selected during flashing.

If you're not using a Jetson box, the Ubuntu 22.04 is preferred. You
must install CUDA 12.3 and TensorRT manually.

### Setup the Development Environment

The project ships an Ansible playbook that can configure the
environment automatically. Run this command to launch the playbook.

```sh
make setup
```

If you prefer to setup the environment manually,

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


## Build This Project

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
