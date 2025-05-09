# Docker Environment Setup

If you don't have access to an NVIDIA Jetson AGX Orin or want to try out AutoSDV in a containerized environment first, you can use our Docker setup. This approach provides a consistent development and testing environment regardless of your host system.

## Prerequisites

The Docker environment requires the following on your host system:

- Docker with NVIDIA container toolkit installed
  - [Docker Engine installation guide](https://docs.docker.com/engine/install/ubuntu/)
  - [NVIDIA Container Toolkit installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- QEMU for ARM64 emulation (if building on x86_64)
- Git for cloning the repository

## Getting Started with Docker

### Step 1: Clone the Repository

```bash
git clone -b 2025.02 --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV/docker
```

### Step 2: Initial Setup

Before building containers for the first time, run the bootstrap command to set up cross-architecture support:

```bash
make bootstrap
```

This installs required dependencies like QEMU and configures Docker to handle ARM64 images.

### Step 3: Build the Docker Image

Build the AutoSDV Docker image with:

```bash
make build
```

This creates a Docker image configured for ARM64 architecture, suitable for Jetson devices. The image will use the current commit of your local repository, clone the repository, and check out that same commit inside the container.

**Note**: Before building, the system checks if your current commit has been pushed to the remote repository. If not, you'll receive an error message asking you to push your changes first.

### Step 4: Run the Container

Launch an interactive shell in the container with:

```bash
make run
```

When you enter the container, you'll have a ready-to-use AutoSDV environment with all dependencies and artifacts installed.

## Working Inside the Docker Container

Once inside the container, you'll find the AutoSDV repository at `/home/developer/AutoSDV` with all dependencies already installed.

You can run commands just as you would on a regular system:

```bash
# Inside the container
cd /home/developer/AutoSDV

# Build if needed (already done during image creation)
# make build

# Launch AutoSDV
ros2 launch autosdv_launch autosdv.launch.yaml
```

## Docker Commands Reference

Here are some useful Docker commands for working with the AutoSDV environment:

| Command            | Description                                          |
|--------------------|------------------------------------------------------|
| `make build`       | Build the Docker image using the current commit hash |
| `make run`         | Enter the container shell                            |
| `make save`        | Save the Docker image as a compressed file           |
| `make clean`       | Remove the Docker image                              |
