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

**You do not need to learn them.** Pick a profile, and read the one-line
explanation the menu shows for whatever is highlighted — every step carries its
own reasoning, including the cost of skipping it, and that text is written
beside the step rather than here where it would drift. `./setup.sh --list`
prints the same thing.

The menu groups them the way the work divides:

| Group | What it covers |
|-------|----------------|
| Toolchain | `just`, ROS 2 Humble, colcon/rosdep, the Rust toolchain and its colcon plugin, `play_launch`, and the Python packages the vehicle interface imports at runtime |
| Autoware | the Autoware 1.5.0 Debians (2–3 GB), the TensorRT runtime they were built against, the writable model tree, rosdep over `src/`, and the perception engines |
| Libraries | OpenCV consistency, the ZED SDK check, the Blickfeld Cube1 driver |
| System configuration | kernel socket buffers for CycloneDDS, loopback multicast, the u-blox udev rule, TurboVNC/VirtualGL |

Four of those used to be documented here as things to do by hand, and the two
that most often broke a fresh install — the CycloneDDS socket buffers and the
loopback `MULTICAST` flag — are the reason to let the profile choose. **Below
about 10 MB of `net.core.rmem_max` no ROS 2 node starts at all**, and `lo` drops
its multicast flag on every reboot, so `dev` and `vehicle` both select the steps
that fix them. Nothing here needs to be typed out of a manual any more.

Two choices are worth understanding before you confirm, because they are the
only ones where the default is a judgement call rather than a requirement.

#### TensorRT engines: download or build

Autoware compiles five perception models into TensorRT engines. Doing that on
the machine takes about an hour on an Orin and nine minutes on a desktop GPU —
and if you skip it, that same work happens inside the first launch's node
constructors, where it looks like a hang and leaves perception unavailable until
it finishes.

The menu offers it as one decision with two answers:

```
    TensorRT engines (pick one, or neither)
   15   (o) Download the published set (build only if none matches)
   16   ( ) Build here, ignoring the published set
```

The default is the download, and it takes about 30 seconds: engines built for
this exact hardware are published on the AutoSDV releases, and the download is
verified by loading every engine before it is trusted. A published set exists
for the boards we run; for anything else the same step builds locally, which is
what you would have paid anyway. Pick the second answer when you are changing a
model yourself, or producing a set to publish.

An engine is tied to the GPU *and* to the exact TensorRT version, so neither
answer can be baked into an image built somewhere else, and both must be redone
after an Autoware or JetPack upgrade.

#### The ZED SDK is installed by you, not by the setup program

Stereolabs publishes no apt repository — the only official artifact is an
interactive installer that asks you to accept a proprietary licence. So the
`zed-sdk` step **checks** rather than installs: it reports the version you have,
and if it is missing or wrong it prints the exact download for your machine, and
prints it again in bright text at the end of the run so it is the last thing on
screen.

Leaving it selected costs nothing on a machine with no ZED camera: the driver
package skips itself and the rest of the workspace builds. Follow
[ZED SDK Installation](./zed-sdk.md) when the run tells you to.

The remaining opt-in step is `blickfeld`, for the Cube1 LiDAR; selecting it
accepts that library's licence terms.

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

## TensorRT engines, if you skipped them in the menu

The setup program does this for you — see
[TensorRT engines: download or build](#tensorrt-engines-download-or-build)
above. To do it afterwards:

```bash
just engines          # the published set for this machine, else build (setup.sh default)
just build-engines    # build here, ignoring what is published
```

`just engines` takes about 30 seconds when a published set matches this
hardware, and falls back to the build when none does. It is safe to re-run: a
second run notices the cache is already in place and does nothing, and an
interrupted download resumes rather than starting over.

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
