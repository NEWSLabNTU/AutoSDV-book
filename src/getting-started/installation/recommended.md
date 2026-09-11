# Recommended Installation

This guide installs AutoSDV with the automated setup program. Make sure you have
[prepared your operating system](./overview.md#prepare-operating-system) first.

## Clone the Repository

The workspace is mostly submodules, so clone recursively:

```bash
cd ~
git clone --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

If you already cloned without `--recursive`:

```bash
just checkout    # git submodule update --init --recursive --checkout
```

## Run the Setup Program

```bash
./setup.sh
```

This opens a menu: choose a **profile**, adjust individual steps if you want to,
then confirm. Nothing is installed until you do.

### Profiles

A profile is a default selection of steps, not a restriction — you can toggle
any step in the menu.

| Profile | For |
|---------|-----|
| `dev` | a development workstation. Everything needed to build, simulate and run. |
| `vehicle` | a vehicle computer. `dev`, plus the u-blox GNSS udev rules. |
| `ci` | build-only automation. Toolchain and workspace dependencies, **no Autoware** — it cannot build the workspace, and is not what you want on a laptop. |
| `all` / `none` | computed: every step, or none. |

For an unattended install:

```bash
./setup.sh --run --profile vehicle --yes
```

### Command line

The menu is optional; every operation has a flag:

```bash
./setup.sh --status                      # what is installed, checked against the machine
./setup.sh --list                        # every step, and whether it applies here
./setup.sh --run --profile dev --yes     # unattended
./setup.sh --run --all --skip tensorrt-engines
./setup.sh --rerun opencv                # one step again
./setup.sh --run --dry-run --profile ci  # print what would run, install nothing
./setup.sh --plain                       # numbered menu, when curses cannot drive the terminal
```

### What `--status` actually reports

`--status` is not a log of what you ran. Where a step carries a verification
command, **the machine's answer wins over the record** — so a step can read as
not installed although you installed it, and be right:

- a reboot drops the loopback `MULTICAST` flag
- a JetPack OTA replaces the OpenCV headers
- an Autoware upgrade leaves the mirrored data tree pointing at a version that
  is gone

It also fingerprints the *content* of the scripts it ran, so it can tell you a
step succeeded but its script has changed since.

### The steps

Grouped as the menu groups them. "Default in" is the profile that selects a step
without you asking.

#### Toolchain

| Step | Default in | Why it is there |
|------|-----------|-----------------|
| `just` | all | every workflow in this repo is a `just` recipe |
| `ros2` | all | ROS 2 Humble, the base everything builds against |
| `ros2-dev-tools` | all | colcon, rosdep, vcstool — without these nothing builds |
| `rust` | all | `cuda_ndt_matcher` is Rust; without a toolchain colcon skips it and `pose_source:=cuda_ndt` has nothing to launch |
| `colcon-cargo-ros2` | all | teaches colcon to build the Rust packages; without it they are skipped *silently* and the build fails later on a missing package |
| `play-launch` | all | the launch orchestrator this book uses throughout |
| `python-deps` | all | `Adafruit-PCA9685`, `simple-pid` — imported by the vehicle interface at runtime |
| `geographiclib` | all | Autoware's map projection needs the egm2008-1 geoid to convert GNSS altitude |
| `gdown` | dev, vehicle | used by the sample-data download scripts |
| `dev-tools` | dev, vehicle | git-lfs, pre-commit, clang-format, PlotJuggler |

#### Autoware

| Step | Default in | Why it is there |
|------|-----------|-----------------|
| `autoware-debian` | dev, vehicle | the Autoware 1.5.0 localrepo, 2–3 GB. Everything in `src/` builds against it |
| `autoware-data` | dev, vehicle | **see below** — without it, perception fails on every launch, forever |
| `ros-deps` | all | rosdep resolves every key the packages under `src/` declare, which is why there are no per-driver apt steps |
| `tensorrt-engines` | *opt-in* | pre-compiles the engines; minutes per model. Skipping it moves the cost to your first launch |

#### Libraries

| Step | Default in | Why it is there |
|------|-----------|-----------------|
| `opencv` | dev, vehicle | JetPack leaves 4.8.0 headers over a 4.5.4 runtime, which compiles and then misbehaves. Also provides aruco/contrib |
| `zed-sdk` | *opt-in* | the ZED X Mini, which every default sensor suite includes. Large download |
| `blickfeld` | *opt-in* | the Cube1 LiDAR driver. Selecting it accepts the library's licence terms |
| `isaac-ros` | *opt-in* | `pose_source:=visual` and `pose_source:=isaac`. Needs an NVIDIA GPU |

#### System configuration

| Step | Default in | Why it is there |
|------|-----------|-----------------|
| `cyclonedds-sysctl` | dev, vehicle | `net.core.rmem_max` and the IP fragment settings. **Below about 10 MB no ROS 2 node can start at all** |
| `multicast-lo` | dev, vehicle | `cyclonedds.xml` pins `lo`, and `lo` loses its `MULTICAST` flag on every reboot. Installs a unit so it survives one |
| `ublox-udev` | vehicle | a stable `/dev/ublox-gps` name, and adds you to `dialout`. Log out and back in for the group to take effect |
| `turbovnc-virtualgl` | *opt-in* | GPU-accelerated rendering over VNC, which the ZED tools need in a VNC session |

### The opt-in steps are a choice you have to make

Four steps are in **no** profile — nothing selects them for you:

- `zed-sdk` — needed if you have a ZED camera. Every default sensor suite
  includes one, so on a vehicle you almost certainly want it.
- `blickfeld` — only for the Cube1 LiDAR.
- `isaac-ros` — only for visual localization.
- `tensorrt-engines` — never strictly needed, always worth it on a vehicle.

> **Note:** the ZED SDK is not installed unless you select it. See
> [ZED SDK Installation](./zed-sdk.md).

## Install and Configure direnv

AutoSDV ships an `.envrc` that activates the environment when you enter the
directory:

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc   # or your shell's equivalent
source ~/.bashrc

cd ~/AutoSDV
direnv allow
```

Without direnv, source the two lines by hand in every new shell:

```bash
source /opt/autoware/1.5.0/setup.bash   # includes ROS 2
source install/setup.bash               # after the first build
```

**Those two lines are where package dependencies are resolved**, which is worth
understanding before something says "package not found" — see
[The Environment](../../concepts/environment.md).

## Build

```bash
just build
```

This runs colcon with the flags the project needs:

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --cargo-args --release
```

`--cargo-args --release` is not decoration. `CMAKE_BUILD_TYPE=Release` covers
only the C++ packages; without the cargo flag the Rust ones build unoptimised,
and `pose_source:=cuda_ndt` runs at roughly 80 ms per scan instead of 5.

## Make the Autoware model tree writable

**Do this even if you skipped every optional step.**

Autoware compiles each `.onnx` model into a `.engine` and writes it *next to the
onnx file*. The Debian package's tree at `/opt/autoware/1.5.0/data` is
root-owned, so that write fails, the engine is discarded, and the same models
rebuild — and fail again — on every single launch.

```bash
just setup-autoware-data
```

This mirrors the tree into `data/autoware_data` with symlinks (171 files, under
a megabyte), which the launch files already default to. Re-run it after an
Autoware upgrade.

## Pre-compile the TensorRT engines (optional, recommended)

```bash
just build-engines
```

Without this, the first launch compiles engines inside each node's constructor —
10 to 30 minutes on an Orin, with perception unavailable throughout.

Engines are tied to **both** the TensorRT version and the GPU, so this must run
on the machine that will use them. It cannot be baked into an image built
elsewhere, and must be re-run after an Autoware or JetPack upgrade.

## Verify

See [Verifying the Installation](./verify.md) — four checks, in order of how
much each one proves.

## Troubleshooting

### The build fails on a missing package

Usually rosdep has not resolved something:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### The build succeeds but `cuda_ndt_matcher` is absent

The Rust toolchain or `colcon-cargo-ros2` is missing, and colcon skipped the
package without saying so:

```bash
./setup.sh --rerun rust
./setup.sh --rerun colcon-cargo-ros2
just build
```

### No ROS 2 node starts at all

Kernel socket buffers. This is not subtle — nothing starts:

```bash
./setup.sh --rerun cyclonedds-sysctl
```

### Nodes cannot see each other on one machine after a reboot

The loopback interface lost its `MULTICAST` flag:

```bash
./setup.sh --status          # multicast-lo will report as not satisfied
./setup.sh --rerun multicast-lo
```

### Perception recompiles its models on every launch

The writable data tree step was skipped, or an Autoware upgrade invalidated the
mirror:

```bash
just setup-autoware-data
```

### CUDA or TensorRT problems

Check what your Autoware installation actually requires, rather than what you
think you installed:

```bash
ldd /opt/autoware/1.5.0/lib/libtensorrt_ops.so | grep nvinfer
# libnvinfer.so.10  -> you need TensorRT 10.x, not 8.x
nvidia-smi
```

### A step reports as not installed although you installed it

That is `--status` reading the machine rather than its own records, and it is
usually right.

## Next Steps

- [Verifying the Installation](./verify.md)
- [The Tutorial](../../tutorial/00-what-you-will-build.md) — drive in simulation
- [Operating the Vehicle](../usage.md) — the full launch argument reference

## Getting Help

- [AutoSDV GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
