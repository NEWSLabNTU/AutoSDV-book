# Run It in a Container

The simulations, on the laptop you already own, with no ROS 2, no Autoware and
no AutoSDV build on it. You open a browser and get a desktop with RViz and a
terminal.

This is the path for a workshop, a classroom, or anyone who wants to try
AutoSDV before committing a machine to it. Windows, macOS — Intel **and** Apple
Silicon — and Linux all run the same command.

!!! info "This is not the unmaintained `docker/` image"

    That one is [Docker Setup (Unmaintained)](./docker.md), a different thing
    that no longer builds. This page is the desktop image, which is built,
    published and tested on both architectures.

## What you need

| | |
|---|---|
| **Docker Desktop** (or Docker Engine on Linux) | [docs.docker.com/desktop](https://docs.docker.com/desktop/) |
| **Disk** | 30 GB free |
| **Memory given to Docker** | 8 GB minimum, **12 GB recommended** — see [Give Docker enough memory](#give-docker-enough-memory) |
| **Download** | 5.4 GB on Apple Silicon, 14.3 GB elsewhere |

No GPU is needed, and on macOS none is possible: Hypervisor.framework exposes
no GPU to a container, and no flag changes that. Both simulations were designed
around that floor and run without one.

## Get it

You still clone the repository, because the map and the rosbags are **not** in
the image — they are large, they change independently, and a student who
already has them should not download them twice.

```bash
git clone https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

Then start the container. There is a launcher per platform, and it does the
pull, the run and the shell in one step:

=== "Linux / macOS"

    ```bash
    ./docker/desktop/autosdv.sh
    ```

=== "Windows (PowerShell)"

    ```powershell
    .\docker\desktop\autosdv.ps1
    ```

    If PowerShell refuses to run it — *"running scripts is disabled"* — allow
    local scripts for your own account, once:

    ```powershell
    Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
    ```

The first run pulls several gigabytes and only happens once. When it finishes
you are at a shell **inside** the container, with ROS 2, Autoware and the
workspace already sourced.

Open the desktop in a browser:

```
http://localhost:6080/vnc.html?autoconnect=1&resize=remote
```

!!! tip "One tag, every machine"

    `jerry73204/autosdv:desktop` is a multi-architecture manifest. Docker asks
    for the platform you are on and gets the matching image — the amd64 one on
    Windows, Linux and Intel Macs, the arm64 one on Apple Silicon. Nobody picks
    a variant.

### Doing it by hand

The launcher is a convenience, not a requirement. The same thing directly:

```bash
docker pull jerry73204/autosdv:desktop

docker run -dit --name autosdv \
  -p 6080:6080 \
  -v "$PWD/data:/opt/AutoSDV/data" \
  --shm-size=2gb --cap-add=NET_ADMIN \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  jerry73204/autosdv:desktop

docker exec -it autosdv bash
```

!!! warning "The `data` mount is not optional"

    Without it the map directory is empty, and `lanelet2_map_loader`
    **segfaults** on a missing map rather than reporting one — so the first
    line you read is a crash, not the cause:

    ```
    PCD load failed: /opt/AutoSDV/data/COSS-map-planning/pointcloud_map.pcd
    Composable node '/map/lanelet2_map_loader' crashed: killed by signal 11
    ```

## Run a simulation

Inside the container shell:

```bash
just sim planning
```

Then watch it in the browser tab. For the tutorial's own walkthrough of what
you are looking at, continue to [1. First Run](../../tutorial/01-first-run.md);
everything there works the same inside the container.

!!! note "RViz draws nothing for about 90 seconds"

    The viewport is black while the map loads. This is load time, not a
    failure — do not start debugging it until the map appears.

### Running Autoware's launch files directly

If you follow a recipe that calls `play_launch` yourself, add
`--container-mode observable`:

```bash
play_launch launch --container-mode observable \
  autoware_launch planning_simulator.launch.xml \
  map_path:=$PWD/data/COSS-map-planning \
  vehicle_model:=autosdv_vehicle \
  sensor_model:=autosdv_sensor_kit
```

The default (`isolated`) forks one process per composable node. On a laptop
that is the difference between working and not — measured in a container
limited to 8 cores and 8 GB:

| | isolated (default) | observable |
|---|---|---|
| all nodes ready | ~90 s | **7 s** |
| memory | 5.71 GiB | **3.09 GiB** |
| processes | 119 | **49** |

What you give up is per-node fault isolation and OOM accounting, which a
tutorial does not need.

### Two terminals

The logging simulation needs one shell for the stack and one for the rosbag
replay. Run the launcher again — from anywhere — and it opens a **second shell
in the same container** rather than starting a new one:

=== "Linux / macOS"

    ```bash
    ./docker/desktop/autosdv.sh
    ```

=== "Windows (PowerShell)"

    ```powershell
    .\docker\desktop\autosdv.ps1
    ```

## Give Docker enough memory

This is the single setting most likely to spoil a first attempt, and its
symptom does not mention memory at all.

**Docker Desktop → Settings → Resources → Memory.** Give it **12 GB** if the
machine can spare it; 8 GB is the floor. Linux with Docker Engine has no such
limit and needs no change.

When the limit is too low, nodes are killed as they start, and the log reads as
scattered crashes across unrelated parts of the stack:

```
Composable node '/map/lanelet2_map_visualization' crashed: killed by signal 6 (Aborted)
Composable node '/adapi/node/vehicle_metrics' crashed: killed by signal 6 (Aborted)
```

Nothing there says "out of memory". If you see aborts in several unrelated
nodes, raise the memory and add `--container-mode observable` before looking
for a bug.

## Stopping it

=== "Linux / macOS"

    ```bash
    ./docker/desktop/autosdv.sh --stop
    ```

=== "Windows (PowerShell)"

    ```powershell
    .\docker\desktop\autosdv.ps1 -Stop
    ```

The container keeps running between shells on purpose, so closing a terminal
does not tear down a simulation.

## Options

| | Linux / macOS | Windows |
|---|---|---|
| newer image first | `--pull` | `-Pull` |
| pass an NVIDIA GPU through | `--gpu` | `-Gpu` |
| stop and remove | `--stop` | `-Stop` |

`--gpu` works on Linux with the NVIDIA Container Toolkit, and on Windows
through Docker Desktop's WSL2 GPU support. It **cannot** work on macOS. It only
takes effect when the container is created, so stop the container first if one
is already running.

Three environment variables change the defaults: `AUTOSDV_PORT` (when 6080 is
taken), `AUTOSDV_IMAGE`, and `AUTOSDV_CONTAINER`.

## Handing it out offline

Fifty laptops pulling 14 GB through one access point is a lost morning. Export
the images to files and serve them locally instead:

```bash
./docker/desktop/export-images.sh          # both architectures
cd /srv/autosdv && ./docker/desktop/serve-images.sh
```

That prints an address to put on the board. Each student downloads the file for
their laptop — the names say *"Apple Silicon Mac"* rather than *"arm64"* — and
loads it:

```bash
docker load < autosdv-desktop-arm64.tar.gz
```

The launcher then finds the image locally and pulls nothing.

## What the container is not for

- **The vehicle.** It has no sensor drivers to talk to, and the Jetson runs the
  real install. See [Software Installation](./overview.md).
- **Perception.** The TensorRT engines are not prebuilt and there is no GPU to
  run them on; the workshop simulations do not launch perception.
- **`pose_source:=cuda_ndt`**, the default on a real machine, which needs an
  NVIDIA GPU at run time. Use `pose_source:=ndt` in the container.

## Next

- [1. First Run](../../tutorial/01-first-run.md) — the tutorial, which works
  unchanged in here
- [What Machine You Need](../requirements.md) — if you would rather install on
  the host
- [The Environment](../../concepts/environment.md) — why those two `source`
  lines exist, already done for you in every shell the launcher opens
