# What Machine You Need

**Read this first if you are installing on your own laptop or desktop** — it is
the prerequisite for [Recommended Installation](./installation/recommended.md),
and the answer is probably "the machine you already have": both simulations run
on four cores, 8 GB and no GPU at all.

**This is not the vehicle's computer.** The car runs a Jetson AGX Orin with
LiDAR, camera, GNSS and the vehicle interface attached, and its numbers are
dominated by sensor traffic that a simulation never generates. That machine is
described under [The Vehicle](../platform-models.md) →
[Hardware Setup](./hardware-assembly.md); nothing on this page constrains it.

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

## Disk

About **11 GB** all told: Autoware 4.8 GB, the repository ~1 GB, the built
workspace ~1 GB, and a 2.8 GB rosbag that only the logging simulation needs.

Keep **20 GB** free — the rosbag's download and its unpacked copy coexist for a
while, and run logs accumulate under `play_log/`. Make it 40 GB if you intend to
record your own bags.

The map needs no download: `data/COSS-map-planning` is in the repository. The
rosbag is the only large fetch — see
[Datasets and Rosbags](../running/datasets.md).

## The GPU

**Neither simulation requires one.** This is the part most often assumed wrong.
The planning simulation has no sensors and never touches a GPU; the logging
simulation runs on the CPU with `pose_source:=ndt launch_perception:=false` and
still keeps up with the recording.

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

RViz is the one part that wants a real graphics stack. Over a VNC server with
software rendering it draws about one frame a second — enough to confirm
something is on screen, useless for watching a vehicle drive. Use a local
display, or VNC with GPU acceleration.

## Checking a machine before you trust it

```bash
just demo check
```

Reports the rosbag, the map, the build, `play_launch`, the CUDA toolkit against
your GPU, and whether a display is available. Everything it names as missing has
a fix on the page it points to.

---

The measured figures behind this page — per-workload memory and CPU, the build
peak and how to cap it, the disk itemisation — are kept in the repository, in
`docs/reports/host-resource-measurements.md`.
