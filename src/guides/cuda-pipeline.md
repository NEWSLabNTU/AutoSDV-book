# The CUDA Point Cloud Pipeline

Point cloud preprocessing can run on the GPU instead of the CPU. Two
whole-stage switches, both defaulting to `cpu`, plus the scan matcher:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  pointcloud_backend:=cuda \
  localization_pointcloud_backend:=cuda \
  pose_source:=cuda_ndt
```

| Switch | Selects the backend for |
|--------|-------------------------|
| `pointcloud_backend` | sensing: crop-self, deskew, ring outlier filter |
| `localization_pointcloud_backend` | the NDT input chain: crop box, voxel grid, random downsample |
| `pose_source` | the scan matcher itself |

They are independent. You can move sensing to the GPU and leave localization on
the CPU, or the reverse.

## Which LiDARs qualify

**`pointcloud_backend:=cuda` works with `vlp32c` and `robin-w`, not `cube1`.**

The reason is deskewing. Removing motion distortion from a scan needs a
**per-point time offset**, and not every driver provides one:

| LiDAR | Driver | Per-point time | CUDA sensing |
|-------|--------|----------------|--------------|
| `vlp32c` | Nebula | yes — `PointXYZIRCAEDT` | supported |
| `robin-w` | `seyond_ros_driver`, from `autosdv-1.5.0` | yes | supported |
| `cube1` | Blickfeld | **no** | refused, with an error naming the reason |

For the Robin-W there is a build-time condition worth knowing: the driver must
be built with the default `POINT_TYPE`. A driver built as `PointXYZIRC`
publishes a cloud that the CUDA preprocessor rejects, and the failure is at run
time rather than at build time.

`cube1` is refused explicitly rather than silently producing a wrong result.

## Everything must load into one container

This is the constraint that surprises people.

`cuda_blackboard` is **not a transport**. It is a process-local map from an id
to a device pointer. A stage passes the *id* downstream, and the next stage
looks that id up in its own process. If that next stage is in a different
process, it receives the id and finds nothing behind it.

So every CUDA stage must load into the same component container. That is why the
switches are whole-stage: a half-applied backend is not a slower configuration,
it is a broken one.

## What AutoSDV had to supply

Autoware ships CUDA implementations of most of the chain, but not all of it. Two
filters live in this repository:

- a standalone CUDA crop box
- a CUDA random downsample

They are in `src/sensing/cuda_pointcloud_filters`. The package skips itself when
no CUDA toolkit is found, so a machine without CUDA still builds the workspace.

## Is it faster?

- `pointcloud_backend:=cuda` — the measurements are in
  `docs/design/cuda-pipeline-data-flow.md` in the repository.
- `localization_pointcloud_backend:=cuda` — correctness is verified; **speed has
  not been measured**. It is honest to treat this one as unproven rather than as
  an optimisation.
- `pose_source:=cuda_ndt` — 1.3–1.6× faster than CPU NDT, and 57 % less CPU on
  Jetson. This is the one with the clearest benefit, and it is already the
  default.

On a vehicle the CPU saving often matters more than the latency: the same CPU is
running perception, planning and control.

## Related

- [Localization Methods](./localization-methods.md)
- [Operating the Vehicle](../getting-started/usage.md)
- `docs/design/cuda-pipeline-data-flow.md` in the repository
