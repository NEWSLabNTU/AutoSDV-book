# What Machine You Need

For running AutoSDV on a laptop or desktop: the two simulations, and developing
against them. **This is not the vehicle's computer.** The car runs a Jetson AGX
Orin with LiDAR, camera, GNSS and the vehicle interface attached, and its
numbers are dominated by sensor traffic that a simulation never generates — see
[Hardware Setup](./hardware-assembly.md) for that machine.

Everything below was measured, on one desktop, with the method named under each
table. Treat it as a floor to design a purchase around, not a guarantee.

## The short answer

| | Minimum | Comfortable |
|---|---|---|
| OS | **Ubuntu 22.04 LTS** — not negotiable, see below | Ubuntu 22.04 LTS |
| CPU | 4 cores | 8 cores or more |
| RAM | 8 GB | 16 GB |
| Free disk | 20 GB | 40 GB |
| GPU | **none** | NVIDIA, CUDA 12.x |

A machine at the minimum runs both simulations and builds the workspace. A
machine at "comfortable" does it without you noticing, and leaves room for the
GPU path and for perception.

### Ubuntu 22.04 is a hard requirement

AutoSDV is built on ROS 2 Humble, which targets Ubuntu 22.04 and is not
packaged for 24.04. This is the one row with no flexibility: not a newer Ubuntu,
not Debian, not WSL as a first choice. A virtual machine works for the
simulations if it gets enough cores and RAM, but it will not get the GPU.

## What each number came from

Measured on: Ubuntu 22.04, Intel Core Ultra 7 270K Plus (24 cores), 125 GB RAM,
RTX 5090, with `scripts/profiling/host_resource_sampler.py`. Memory is reported
as the machine's own usage over an idle baseline, which is what a smaller
machine has to find. CPU is reported in whole cores, because that is the figure
that transfers to a machine with a different core count.

| Workload | Peak RAM | CPU at startup | CPU in steady state |
|---|---|---|---|
| `just build`, clean workspace | **11.3 GiB** | every core available | — |
| Planning simulation | **2.5 GiB** | 19 cores, briefly | ~2 cores |
| Logging simulation, CPU path | **3.4 GiB** | 21 cores, briefly | ~3 cores |
| Logging simulation, GPU path | 3.4 GiB + ~1 GiB VRAM | same | ~3 cores |
| RViz, added to any of them | ~1.4 GiB | — | ~1 core |

Three of those numbers deserve a sentence each.

**The build peak is the largest, and it is adjustable.** 11.3 GiB is what colcon
reached compiling 31 packages with 24 jobs in parallel; the trace spends about
twelve seconds there and then falls away. The peak scales with how many
compilers run at once, so a 4-core laptop reaches roughly a quarter of it
without being asked. If a machine is short of memory, cap it explicitly rather
than discovering the OOM killer:

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

**"CPU at startup" is a burst, not a requirement.** Launching a stack starts
thirty-odd nodes at once, and they will use every core present — 21 of 24 here.
On four cores the same work simply takes longer; nothing fails. What a machine
has to sustain is the steady-state column, and that is two to three cores.

**The build is quick because most of Autoware is already built.** 31 packages in
**89 seconds** on this desktop. AutoSDV is a workspace *over* a binary Autoware
installation, so `just build` compiles the vehicle's own packages and not the
30 GB of Autoware underneath. Budget minutes, not hours — and on four cores,
still minutes.

## Disk, itemised

| | |
|---|---|
| Autoware Debian, downloaded | 1.9 GB |
| Autoware, installed at `/opt/autoware/1.5.0` | 4.8 GB |
| This repository, cloned (includes the COSS map) | ~1 GB |
| Workspace after `just build` (`build/` + `install/`) | 0.9 GB |
| COSS rosbag, downloaded | 1.6 GB |
| COSS rosbag, unpacked | 2.8 GB |
| **Total, everything, after cleanup** | **~11 GB** |

20 GB free is the minimum because the download and the unpacked copy of the
rosbag coexist for a while, and because ROS logs under `play_log/` grow with
every run. 40 GB if you intend to record your own bags, which is the one thing
here that has no natural size.

The map needs no download: `data/COSS-map-planning` is committed to the
repository. The rosbag is the only large fetch, and only the logging simulation
needs it — see [Datasets and Rosbags](../simulation/datasets.md).

## The GPU, in detail

**Neither simulation requires one.** This is the part most often assumed wrong.

- The **planning simulation** has no sensors, no perception and no
  localization. It never touches a GPU.
- The **logging simulation** replays real LiDAR and localizes against a map.
  With `pose_source:=ndt launch_perception:=false` it is pure CPU, and it holds
  the sensor's full 10 Hz — measured at 10.06 Hz against a 10 Hz recording.

A GPU buys two things:

| | Needs | Costs |
|---|---|---|
| `pose_source:=cuda_ndt` | CUDA, and a toolkit that knows your GPU | ~1 GiB VRAM |
| Perception (TensorRT models) | NVIDIA GPU | VRAM, plus a **10–30 minute** engine build on first launch |

The engine build is a one-off per machine and per Autoware version, and
`just build-engines` does it deliberately instead of during your first launch.
Until then, `launch_perception:=false` is the honest way to run on a laptop.

### If you have a very new GPU

`cuda_ndt` compiles its kernels at run time, so the CUDA toolkit has to know
your card's architecture. A card newer than the toolkit fails exactly like
having no GPU, except louder and later — the launcher reports every node ready
and then nothing publishes. Blackwell (RTX 50-series, sm_120) needs **CUDA
12.8** or newer.

```bash
scripts/check-cuda-arch.sh     # also part of `just demo check`
```

It compares your GPU against the selected toolkit and, if they disagree, names
the installed toolkits that would work. Selecting one is a single variable —
see [The Environment](../concepts/environment.md).

### RViz needs working OpenGL

RViz is the one part that wants a real graphics stack. Over a plain VNC server
with software rendering it rendered at **1 fps** on this machine — usable to
confirm something is on screen, useless to watch a vehicle drive. A local
display, or VNC with GPU acceleration, is what makes the visual parts of the
tutorial worth doing.

## Checking a machine before you trust it

```bash
just demo check
```

Reports the rosbag, the map, the build, `play_launch`, the CUDA toolkit against
your GPU, and whether a display is available. Everything it names as missing has
a fix on the page it points to.
