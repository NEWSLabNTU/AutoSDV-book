# Maps

Different localization methods consume different map artefacts. Handing a method
the wrong one usually fails *quietly* — it localizes badly rather than refusing
to start — which is why this page is mostly about checking.

## What each method needs

| `pose_source` | Needs | In the map directory |
|---------------|-------|----------------------|
| `cuda_ndt`, `ndt` | a 3-D point cloud map | `pointcloud_map.pcd` |
| `mcl` | a 2-D occupancy grid | `occupancy_grid.yaml` + `occupancy_grid.pgm` |
| `visual` | a visual map | `cuvgl_map/`, `cuvslam_map/` |
| any of them | the lanelet2 map, for planning | `lanelet2_map.osm` + the projector info |

A map directory is passed with `map_path`:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml map_path:=data/my_site
```

## Check a map before you trust it

```bash
just map check <map_dir> [pose_source]
just map check data/COSS-map-planning cuda_ndt
just map check data/COSS-map-planning mcl
```

This verifies that the artefacts the given method needs are present, and — the
reason the tool exists — **that the occupancy grid is in the lanelet2 map's
frame**. A grid built in the wrong frame produces a system that starts cleanly,
localizes confidently, and is wrong by a constant offset. That failure class has
cost days; the check takes a second.

Extra flags pass through, for example `--grid-yaml NAME` to check a grid variant
other than `occupancy_grid.yaml`.

## Building an occupancy grid

MCL needs a grid, and most sites only have a PCD. Two ways to get one.

### From an existing PCD map

```bash
just map grid-from-pcd data/COSS-map-planning
```

Run it **without flags first**. It prints the height distribution of the point
cloud, an estimated ground level and a suggested band — and then refuses to
guess, because the z band is the one judgement that matters and it is not
forgiving. A wrong band yields a valid-looking grid that localizes badly rather
than an error.

Then re-run with the band:

```bash
just map grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
```

This writes `occupancy_grid.pgm` and `occupancy_grid.yaml`, records how the grid
was built in `autosdv_map.yaml`, and validates the result for
`pose_source:=mcl`.

Other flags: `--resolution` (default 0.05 m/px) and `--min-points`, the number
of points that must fall in the band before a cell counts as occupied.

### From a recorded drive, for a site with no PCD

```bash
just map grid-from-bag <bag> <map_dir>
```

Accumulates 2-D scans at their ground-truth poses. Here the band is relative to
the *scan plane* rather than to site ground level, so the defaults
(−0.15 … 0.15 m) are meaningful and often correct.

Provenance is recorded in `autosdv_map.yaml` either way, so a grid can always be
traced back to how it was made.

## Choosing the z band

The band is a horizontal slice through the point cloud. You want it:

- **above the ground**, or every cell is occupied
- **below the overhangs** — tree canopy, awnings, ceilings — or you map things
  the LiDAR will not see from the vehicle
- **at roughly the height the scan plane will be**, since that is what the
  particle filter will compare against

A 0.3 m band starting a little above the estimated ground level is a reasonable
first attempt. Then look at the `.pgm`: walls and building faces should be
continuous lines, and open ground should be empty.

## Visual maps

For `pose_source:=visual`:

```bash
# 1. record a rosbag with ZED stereo + IMU
./scripts/visual-map/record.sh ./data/visual_maps/my_location

# 2. build the map
./scripts/visual-map/create-map.sh ./data/visual_maps/my_location_recording
```

This produces `cuvgl_map/`, `cuvslam_map/` and `occupancy_map/`, and is passed
with `visual_map_dir:=` rather than `map_path:=`.

## The default map

`data/COSS-map-planning`, the COSS Park map, is the default for `map_path` and
is what the [simulation guides](../tutorial/02-planning-simulation.md) use.

## Related

- [Localization Methods](./localization-methods.md)
- `docs/design/map-handling-per-localization-method.md` in the repository
