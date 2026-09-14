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

Pick a profile, confirm, and let it run. Nothing is installed until you confirm.

| Profile | For |
|---------|-----|
| `dev` | a development workstation: everything needed to build, simulate and run |
| `vehicle` | a vehicle computer: `dev`, plus the u-blox GNSS udev rules |
| `ci` | build-only automation. **No Autoware** — not what you want on a laptop |

Unattended, same thing:

```bash
./setup.sh --run --profile dev --yes
```

The menu lists every step with a one-line reason, so read it there rather than
here. Two of them ask you to choose, and both have a sensible default:

- **TensorRT engines** — download the set published for this machine (about 30
  seconds) or build them here (about an hour on an Orin). The download is the
  default; it falls back to building when nothing matches your hardware.
- **ZED SDK** — checked, not installed, because Stereolabs ships no apt package.
  If you have a ZED camera the run ends by telling you what to download; see
  [ZED SDK Installation](./zed-sdk.md). Without a camera, ignore it.

## Build

The workspace is built with colcon, and this is the full command:

```bash
source /opt/autoware/1.5.0/setup.bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --cargo-args --release
```

Four flags, each load-bearing:

| flag | what it does |
|------|--------------|
| `--base-paths src` | build the packages under `src/`, not whatever else is in the directory |
| `--symlink-install` | install by symlink, so edits to YAML, XML and Python take effect without rebuilding |
| `--cmake-args -DCMAKE_BUILD_TYPE=Release` | optimised C++ |
| `--cargo-args --release` | optimised Rust — `CMAKE_BUILD_TYPE` does not reach cargo, and an unoptimised `cuda_ndt_matcher` runs roughly 15x slower |

**Shortcut:** `just build` runs exactly that, sourcing included.

```bash
just build
```

Expect a few minutes and a `Summary:` line with no failed packages; warnings on
stderr are normal.

## Check it worked

```bash
./setup.sh --status
```

Then [Verifying the Installation](./verify.md) — a handful of checks that take a
minute.

## Next

[The tutorial](../../tutorial/00-what-you-will-build.md): drive in simulation,
twice, and see the system actually run. That is the real proof the install
worked.

---

## Going further

Nothing below is needed for a working install.

### Enter the environment automatically with direnv

AutoSDV ships an `.envrc` that activates the environment when you enter the
directory:

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc   # or your shell's equivalent
source ~/.bashrc

cd ~/AutoSDV
direnv allow
```

Without direnv, source two lines by hand in each new shell:

```bash
source /opt/autoware/1.5.0/setup.bash   # includes ROS 2
source install/setup.bash               # after the first build
```

Those two lines are where package dependencies are resolved — see
[The Environment](../../concepts/environment.md).

### Driving setup.sh from the command line

```bash
./setup.sh --status                      # what is installed, checked against the machine
./setup.sh --list                        # every step, and whether it applies here
./setup.sh --run --profile dev --yes     # unattended
./setup.sh --rerun opencv                # one step again
./setup.sh --run --dry-run --profile ci  # print what would run, install nothing
./setup.sh --plain                       # numbered menu, when curses cannot drive the terminal
```

`--status` is not a log of what you ran. Where a step can be checked on the
machine, **the machine's answer wins** — so a step can read as not installed
although you installed it, and be right: a reboot drops the loopback
`MULTICAST` flag, a JetPack OTA replaces the OpenCV headers, an Autoware upgrade
invalidates the mirrored model tree.

### TensorRT engines, afterwards

If you skipped the engine step, or want to change your mind:

```bash
just engines          # the published set for this machine, else build
just build-engines    # build here, ignoring what is published
```

Safe to re-run: a second run notices the cache is in place and does nothing, and
an interrupted download resumes.

Engines are tied to the GPU *and* the TensorRT version, so they cannot be baked
into an image built elsewhere, and must be redone after an Autoware or JetPack
upgrade.

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

## Getting Help

- [AutoSDV GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
