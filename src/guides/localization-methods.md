# Localization Methods

`pose_source` selects how AutoSDV estimates where it is. Three values, with
genuinely different requirements — different sensors, different map artefacts,
different hardware.

```bash
play_launch launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

## The options at a glance

| `pose_source` | Estimates from | Map artefact needed | Needs a GPU |
|---------------|----------------|---------------------|-------------|
| `cuda_ndt` *(default)* | 3-D LiDAR scan matched to a point cloud map | PCD | yes |
| `ndt` | the same, on CPU | PCD | no |
| `mcl` | one 2-D `LaserScan` matched to an occupancy grid | occupancy grid | no |

## `cuda_ndt` — the default

CUDA-accelerated NDT scan matching. 1.3–1.6× faster than the CPU matcher and
57 % less CPU on Jetson platforms, which is what makes it the default on a
vehicle whose CPU is also running perception and control.

The package is `cuda_ndt_matcher`, maintained by AutoSDV rather than upstream
Autoware, and it presents the same input and output interfaces as the standard
NDT.

It is written in Rust. If the Rust toolchain or `colcon-cargo-ros2` was missing
at build time, colcon skips the package **silently** and this pose source has
nothing to launch — see
[the installation troubleshooting](../getting-started/installation/recommended.md#the-build-succeeds-but-cuda_ndt_matcher-is-absent).

It must also be built in release mode. `just build` passes `--cargo-args
--release` for exactly this reason; without it the matcher runs at roughly 80 ms
per scan instead of 5.

## `ndt` — the CPU fallback

Autoware's built-in OpenMP NDT. Same algorithm, same map, no GPU. Use it on a
machine without CUDA, or to isolate whether a problem belongs to the CUDA path.

!!! warning "`pose_source_package` and the empty value"

    `pose_source_package` defaults to `auto`, which resolves to
    `cuda_ndt_matcher_launch` for `cuda_ndt` and to the built-in Autoware NDT
    otherwise. Set it explicitly only to plug in a third-party estimator. To
    get built-in NDT, pass `pose_source:=ndt` — not an empty
    `pose_source_package`, which `ros2 launch` rejects.

## `mcl` — 2-D Monte Carlo localization

Localizes a single-plane `LaserScan` against a 2-D occupancy grid, instead of a
3-D cloud against a PCD. Measured on the Autoware sample site against NDT ground
truth over five seeds: **mean 0.789 m, p95 2.075 m, mean |yaw| 0.0159 rad**
with a 3-ring scan source.

### The scan contract

MCL consumes **one** `sensor_msgs/LaserScan`, in any frame TF connects to
`base_link`. It holds no scan geometry of its own. The sensor kit owns scan
production, because which physical plane to use depends on the sensor and how it
is mounted.

```bash
# the kit publishes the scan (production)
play_launch launch autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl map_path:=data/my_site

# MCL synthesises one from a 3-D cloud (test scaffolding only)
just coss logging-sim   # then, for MCL: pass pose_source:=mcl scan_source:=test_pointcloud
```

`mcl_scan_normalizer` resolves the mounting offset itself. This is not a detail:
the particle filter treats the scan as originating *at the particle pose*, so a
laser frame mounted 0.5 m forward of `base_link` would otherwise bias every
range by 0.5 m.

### Producing a scan from a 3-D LiDAR

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  publish_scan:=true   # plus scan_ring, or ring_min / ring_max
```

**Use a small ring group, not a single ring.** Measured:

| scan source | mean | seed spread | gate | mean \|yaw\| |
|---|---|---|---|---|
| slab, 0.30 m band | 0.821 m | 0.072 | 5/5 | 0.0339 rad |
| 1 ring | 0.992 m | 0.317 | 3/5 | 0.0321 rad |
| **3 rings (70–72)** | **0.789 m** | **0.037** | **5/5** | **0.0159 rad** |

A single ring is geometrically a perfect plane but too sparse on a 128-ring
spinner. Three adjacent VLS128 channels span 0.22° — 0.23 m at 60 m — which is
*tighter* than the slab they beat, so this is not a trade of fidelity for
density.

### Which sensors this applies to

**Spinning LiDARs only.** Ring extraction assumes constant-elevation rings, so
it works for `vlp32c` and not for the kit's solid-state sensors. Robin-W and
Cube1 have restricted fields of view and a channel index that is not a fixed
elevation, so no ring of theirs is a horizontal plane — and a narrow field of
view constrains MCL poorly against a 360° grid anyway. Use those sensors on the
3-D NDT path, or with a native 2-D LiDAR alongside.

### The ring is sensor-specific and must be measured

0.11° channel spacing is a property of one particular VLS128, not a constant.
Do not copy a ring number from this page:

```bash
python3 scripts/sensor/inspect_rings.py <bag> --topic <cloud> --height <mounting_h>
```

It reports per-channel elevation, names the horizontal ring, and warns when a
ring points too far up to meet the ground — which is how a low vehicle ends up
configured with a ring aimed at the sky.

### Map and diagnostics

MCL needs `occupancy_grid.yaml` plus its `.pgm`, not a PCD. Build one with
`just map grid-from-pcd` or `just map grid-from-bag` and validate with `just map
check`; see [Maps](./maps.md).

The scan normaliser reports a missing scan, a missing TF, an all-non-finite
scan, and an out-of-plane mount. Every scan-side failure in this project's
history was previously silent.

## Mapless mode

Not a `pose_source` — an escape hatch. For indoor operation where no map exists
and no localization is possible:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  use_mapless_mode:=true use_gnss:=false
```

Mapless mode implies no point cloud map.

## Choosing between them

- **On the vehicle, outdoors, with a 3-D LiDAR**: `cuda_ndt`.
- **On a laptop without CUDA**: `ndt`.
- **With a 2-D LiDAR, or a spinning 3-D one and only a floor plan**: `mcl`.

The [logging simulation](../tutorial/03-logging-simulation.md) is the right place
to compare them, because the recorded input is identical every run.

## Related

- [Maps](./maps.md) — what each method needs, and how to build it
- [The CUDA point cloud pipeline](./cuda-pipeline.md)
- [Operating the Vehicle](../getting-started/usage.md)
