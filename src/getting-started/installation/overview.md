# Software Installation

This guide provides the complete installation workflow for AutoSDV. Follow these steps sequentially to set up a fully functional autonomous vehicle software stack.

## How this differs from installing Autoware

If you have read the Autoware documentation, you arrived expecting to clone a
workspace, run `vcs import` over dozens of repositories, and spend an hour or
more in `colcon build`.

**AutoSDV does not do that.** Autoware arrives as **Debian packages**, installed
into `/opt/autoware/1.5.0`. There is no Autoware source tree on your disk and
nothing of Autoware is compiled on your machine — a source build on an Orin
costs hours that a student or a vehicle integrator should not pay. In exchange
you take the version that was built for you, pinned in `versions.yaml`.

**AutoSDV itself is still compiled.** Everything under `src/` is a colcon
workspace and `just build` builds it.

So you end up in a hybrid: a binary Autoware underneath, a source workspace on
top. That shape is what the two `source` lines in
[The Environment](../../concepts/environment.md) are about, and it is where
package dependencies are resolved — not by the build. Read that page if
anything later says "package not found".

## What you install it on

1. **An Ubuntu 22.04 laptop or desktop** — for the simulations and for
   development. A GPU is useful and **not required**; both simulations run
   without one. Keep **20 GB** free. Measured figures — cores, memory, the build
   peak, what a GPU does and does not buy — are in
   [What Machine You Need](../requirements.md), the next page.
2. **NVIDIA Jetson AGX Orin 64GB** — the vehicle itself, with the sensors
   attached. See [Hardware Setup](../hardware-assembly.md).
3. **Docker Environment** (unmaintained)

Ubuntu 22.04 is the one requirement with no alternative: AutoSDV is built on
ROS 2 Humble, which is not packaged for 24.04.

## Installation Methods

Choose the installation method that best fits your needs:

| Method                                                      | Best For                              | Difficulty | Customization |
|-------------------------------------------------------------|---------------------------------------|------------|---------------|
| **[Recommended Installation](./recommended.md)**            | Most users, production deployment     | Easy       | Limited       |
| **[Manual Environment Setup](./manual-environment.md)**     | Advanced users, custom configurations | Advanced   | Full          |
| **[Docker Setup](./docker.md)**                             | Development, testing, simulation      | Easy       | Limited       |

## Prepare Operating System

Before installing AutoSDV, prepare your target platform.

### For NVIDIA Jetson AGX Orin

1. Download and install [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
2. Flash the Jetson with:
   - **JetPack SDK 6.2.2 or newer** — a floor, not an exact version
   - all CUDA and TensorRT packages selected
   - an external NVMe SSD as the target, not the internal eMMC

**Check the L4T version, not the JetPack version,** once the board is up:

```bash
head -1 /etc/nv_tegra_release     # "# R36 (release), REVISION: 5.0" -> L4T 36.5.0
```

JetPack's patch numbering crosses an L4T *minor* inside the 6.2 series, which is
why the floor is where it is and why several downloads later on ask for L4T
rather than JetPack ([NVIDIA's
mapping](https://developer.nvidia.com/embedded/jetpack-archive)):

| JetPack | L4T | |
|---------|-----|---|
| 6.2 | 36.4.3 | below the floor |
| 6.2.1 | 36.4.4 | below the floor |
| **6.2.2** | **36.5.0** | the floor |
| 6.2.3 | 36.5.2 | fine |

Three things read that number rather than the JetPack one: the
[ZED SDK installer](./zed-sdk.md) is published per L4T minor, NVIDIA's Jetson apt
pocket is `r36.5`, and a prebuilt TensorRT engine set is keyed by L4T — so a board
at the floor will not match the engine set published from JetPack 6.2.1 and will
build its own the first time, about an hour. That is slow, not broken; see
[TensorRT engines, afterwards](./recommended.md#tensorrt-engines-afterwards).

The Autoware package for arm64 keeps the filename suffix `jetpack62` at every
patch: it names the 6.2 series it was built for, and 6.2.2 is in that series.

### For Ubuntu 22.04 PC

1. Install Ubuntu 22.04 LTS
2. Install the NVIDIA driver (version 550 or higher):
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-550
   ```
3. Install **CUDA 12.x** using the deb (network) installer
4. Install **TensorRT 10.x**

!!! warning "TensorRT must be 10.x, not 8.x"

    Autoware 1.5.0's own libraries are linked against `libnvinfer.so.10`.
    TensorRT 8.6 provides `libnvinfer.so.8`, so perception will fail to load
    with an unresolved-symbol or missing-library error that names a shared
    object rather than the real cause. Earlier versions of this page said
    "TensorRT 8.6 GA"; that applied to an older Autoware base.

    You can verify what your installation actually requires:

    ```bash
    ldd /opt/autoware/1.5.0/lib/libtensorrt_ops.so | grep nvinfer
    # libnvinfer.so.10 => ...
    ```

    The same check gives the CUDA major version:

    ```bash
    ldd /opt/autoware/1.5.0/lib/libautoware_lidar_centerpoint_cuda_lib.so \
      | grep -E 'cublas|cudart'
    # libcublas.so.12 => ...
    ```

    These sonames are the real requirement. Any CUDA 12.x and TensorRT 10.x
    that satisfy them will work; the exact patch versions do not matter.

### For Docker

See [Docker Setup](./docker.md) for containerized installation. This skips the OS preparation steps above.

## Install ZED SDK (if using ZED camera)

**ZED SDK must be installed manually before proceeding.**

The ZED SDK and ZED Link drivers are required if you're using ZED cameras. See [ZED SDK Installation Guide](./zed-sdk.md) for detailed instructions.

> **Note:** This is a manual installation step. The automated setup script does NOT install ZED SDK.

## Next Step

Once your operating system is prepared, proceed to the [Recommended Installation](./recommended.md) guide.
