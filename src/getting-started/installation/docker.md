# Docker Setup

Docker provides a containerized environment for running AutoSDV without modifying your host system. This is ideal for development, testing, and simulation scenarios.

## Use Cases

Docker is recommended for:
- **Development and Testing**: Consistent environment across different machines
- **Simulation**: Running AutoSDV without physical hardware
- **CI/CD**: Automated testing and deployment
- **Quick Evaluation**: Try AutoSDV without full installation

## Prerequisites

### Host System Requirements

- Ubuntu 20.04 or 22.04 (other Linux distributions may work)
- NVIDIA GPU with driver 470+ (for GPU acceleration)
- At least 50GB free disk space
- 16GB+ RAM recommended

### Software Requirements

1. **Docker Engine** (20.10 or newer):
   ```bash
   # Install Docker
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   
   # Add user to docker group
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. **NVIDIA Container Toolkit** (for GPU support):
   ```bash
   # Add NVIDIA repository
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
     sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   
   # Install nvidia-container-toolkit
   sudo apt update
   sudo apt install nvidia-container-toolkit
   sudo systemctl restart docker
   ```

3. **Docker Compose** (optional, for multi-container setups):
   ```bash
   sudo apt install docker-compose
   ```

## Quick Start

### Step 1: Clone AutoSDV Repository

```bash
git clone -b 2025.02 --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV/docker
```

### Step 2: Bootstrap Docker Environment

Set up cross-architecture support (required for ARM64 emulation on x86_64):

```bash
make bootstrap
```

### Step 3: Build Docker Image

Build the AutoSDV Docker image:

```bash
make build
```

This creates an image with:
- Ubuntu 22.04 base with ROS 2 Humble
- Autoware 2025.02 pre-installed
- All AutoSDV dependencies
- CUDA and TensorRT support
- Sensor driver libraries (except proprietary ones)

### Step 4: Run Container

Start an interactive container session:

```bash
make run
```

You'll enter a shell with AutoSDV ready to use at `/home/developer/AutoSDV`.

## Docker Image Details

### Image Architecture

The AutoSDV Docker image is built for **ARM64 architecture** to match the Jetson platform. On x86_64 hosts, QEMU provides transparent emulation.

### Pre-installed Software

- **ROS 2 Humble** with desktop tools
- **Autoware 2025.02** binary release
- **CUDA 12.3** and **TensorRT 8.6**
- **Cyclone DDS** configured as default
- **Development tools**: git, vim, tmux, htop

### Volume Mounts

The `make run` command automatically mounts:
- `/tmp/.X11-unix` for GUI applications
- NVIDIA GPU devices for CUDA access

## Advanced Usage

### Custom Run Options

Run with additional volumes or ports:

```bash
docker run -it --rm \
  --gpus all \
  --network host \
  -v /path/to/data:/data \
  -v /dev:/dev \
  --privileged \
  autosdv:latest
```

### Development Workflow

For active development, mount your local code:

```bash
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace/AutoSDV \
  -w /workspace/AutoSDV \
  autosdv:latest
```

### GUI Applications

Enable X11 forwarding for visualization tools:

```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  autosdv:latest
```

Then run GUI applications like RViz:
```bash
# Inside container
rviz2
```

### Multi-Container Setup

Create a `docker-compose.yml` for complex deployments:

```yaml
version: '3.8'

services:
  autosdv:
    image: autosdv:latest
    runtime: nvidia
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev
      - ./data:/data
    environment:
      - ROS_DOMAIN_ID=0
      - DISPLAY=${DISPLAY}
    command: ros2 launch autosdv_launch autosdv.launch.yaml

  monitoring:
    image: autosdv:latest
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    command: python3 /home/developer/AutoSDV/src/launcher/autosdv_launch/autosdv_launch/autosdv_monitor.py
```

Run with:
```bash
docker-compose up
```

## Container Management

### Save and Load Images

Export image for deployment:

```bash
make save  # Creates autosdv_docker.tar.gz
```

Load on another machine:

```bash
docker load < autosdv_docker.tar.gz
```

### Clean Up

Remove container and image:

```bash
make clean
```

## Limitations

### Hardware Access

Docker containers have limited hardware access:
- **No direct LiDAR access** (USB/Ethernet sensors need special configuration)
- **No CAN bus** without `--privileged` flag
- **Camera access** requires device mounting

### Performance

- ARM64 emulation on x86_64 reduces performance
- GPU passthrough adds overhead
- Network performance may vary with Docker networking modes

### Sensor Drivers

The Docker image includes most drivers via rosdep, but:

- **ZED SDK**: Cannot be fully used in Docker due to hardware requirements. Physical deployment requires native installation.
- **Blickfeld**: Installed via rosdep in container
- **Velodyne**: Installed via rosdep in container

For full ZED camera support, use native installation with [ZED SDK Installation Guide](./zed-sdk.md).

## Troubleshooting

### GPU Not Accessible

Verify NVIDIA runtime:
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### Network Issues

Use host networking for ROS 2 communication:
```bash
docker run --network host ...
```

### Permission Denied

For device access, run with privileges:
```bash
docker run --privileged -v /dev:/dev ...
```

### Build Failures

Clear Docker cache and rebuild:
```bash
docker system prune -a
make bootstrap
make build
```

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: AutoSDV Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      
      - name: Set up Docker
        uses: docker/setup-buildx-action@v1
      
      - name: Build Docker image
        run: |
          cd docker
          make build
      
      - name: Run tests
        run: |
          docker run --rm autosdv:latest \
            bash -c "cd /home/developer/AutoSDV && colcon test"
```

### Jenkins Pipeline Example

```groovy
pipeline {
    agent any
    
    stages {
        stage('Build') {
            steps {
                sh 'cd docker && make build'
            }
        }
        
        stage('Test') {
            steps {
                sh 'docker run --rm autosdv:latest make test'
            }
        }
    }
}
```

## Next Steps

- [Software Installation](./overview.md) - Native installation guide
- [Usage Guide](./usage.md) - Operating AutoSDV
- [Development Guide](../guides/development.md) - Development workflows
- [Manual Setup](./manual-environment.md) - Customization options