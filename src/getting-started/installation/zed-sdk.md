# ZED SDK Installation

The ZED SDK provides the drivers and APIs for ZED stereo cameras and ZED Link
capture cards. **You install it by hand.** Stereolabs publishes no apt
repository; the only official artifact is a self-extracting installer that asks
you to accept a proprietary licence, so `setup.sh` checks for it and tells you
what to download rather than answering that question on your behalf.

## When Do You Need This?

Install the ZED SDK if you are using:

- a ZED X Mini camera (the standard AutoSDV configuration)
- a ZED 2/2i camera
- a ZED Link capture card (Mono/Dual/Quad)

Skip it if you run LiDAR only (Velodyne, Blickfeld, Robin-W). Nothing else
breaks without it: `zed_components` reports that it is skipping itself, and the
rest of the workspace builds.

## The version is not a preference

| Component | Version |
|-----------|---------|
| ZED SDK | 5.4.1 |
| ZED ROS 2 wrapper | 5.4.1 (our `ntust-workshop` branch, rebased onto `v5.4.1`) |

`zed_components` compiles against the SDK's own headers, so an SDK and a wrapper
at different versions is a build or a runtime failure, not a degraded mode. The
pair is recorded in `versions.yaml` under `zed:`; bump both or neither.

## Prerequisites

- Ubuntu 22.04 with an NVIDIA driver and CUDA 12 (the version Autoware pins), or
- a Jetson running JetPack 6.x (L4T 36.4 or 36.5)

## Step 1 — Ask setup.sh what this machine needs

```bash
./setup.sh --run --only zed-sdk --yes
```

It prints the installed version if there is one, and otherwise the exact
download for this machine. The same advice is repeated, highlighted, at the end
of any setup run that leaves it unsatisfied.

## Step 2 — Download

| Machine | Installer |
|---------|-----------|
| amd64, Ubuntu 22.04, CUDA 12 | <https://download.stereolabs.com/zedsdk/5.4/cu12/ubuntu22> |
| Jetson, L4T 36.4 (JetPack 6.0/6.1) | <https://download.stereolabs.com/zedsdk/5.4/l4t36.4/jetsons> |
| Jetson, L4T 36.5 | <https://download.stereolabs.com/zedsdk/5.4/l4t36.5/jetsons> |

These are redirects Stereolabs keeps stable. Each resolves to a CDN file whose
name carries the patch version (for amd64, today,
`ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.4.1.zstd.run`), so bookmark the
redirect and not the file it lands on.

Check which L4T a Jetson is running before you choose:

```bash
head -1 /etc/nv_tegra_release     # "# R36 (release), REVISION: 4.4" -> L4T 36.4
```

## Step 3 — Install

```bash
curl -fsSL -o zed_sdk.run 'https://download.stereolabs.com/zedsdk/5.4/cu12/ubuntu22'
chmod +x zed_sdk.run
./zed_sdk.run
```

Answer the prompts: accept the licence, install the tools and the Python API,
and allow the AI model download if you intend to use object detection. It takes
10–20 minutes, mostly download.

## Step 4 — Confirm

```bash
./setup.sh --rerun zed-sdk     # "ZED SDK 5.4.1 is installed at /usr/local/zed."
just build                     # now builds zed_components
```

`./setup.sh --status` reads the SDK's own cmake version file — the same file
`find_package(ZED)` reads during the build — so it reports a version mismatch as
not-installed, which is the honest answer.

## On amd64, the installer also brings TensorRT 10.9

The amd64 package ships TensorRT 10.9, while Autoware's perception engines
require 10.8 **exactly**: a cached engine records the TensorRT that built it, and
Autoware discards any engine whose version differs, rebuilding all five models on
every launch.

Both can coexist, and AutoSDV arranges that for you. The `tensorrt-runtime`
setup step installs Autoware's TensorRT into a private prefix, and
`scripts/env.sh` puts it ahead of the system one for AutoSDV processes only —
which is exactly why that step does not downgrade the system libraries: doing so
would break the ZED SDK.

```bash
./setup.sh --run --only tensorrt-runtime --yes
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
```

CUDA 12 is what Autoware pins (`nvidia_amd64.cuda` in `versions.yaml`), so match
that rather than a specific patch. On a Jetson it comes from JetPack; on a
workstation it comes from the host image or NVIDIA's apt repository —
`setup.sh` deliberately does not install a CUDA toolkit, because doing so
repoints `/usr/local/cuda` for every other user of the machine.

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

- Continue to [Recommended Installation](./recommended.md)
- Continue with automatic or manual setup
- Proceed to build and verify AutoSDV
