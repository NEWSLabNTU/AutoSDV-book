# Maps

Different localization methods consume different map artefacts. Handing a method
the wrong one usually fails *quietly* — it localizes badly rather than refusing
to start — which is why this page is mostly about checking.

## What each method needs

| `pose_source` | Needs | In the map directory |
|---------------|-------|----------------------|
| `cuda_ndt`, `ndt` | a 3-D point cloud map | `pointcloud_map.pcd` |
| `mcl` | a 2-D occupancy grid | `occupancy_grid.yaml` + `occupancy_grid.pgm` |
| any of them | the lanelet2 map, for planning | `lanelet2_map.osm` + the projector info |

A map directory is passed with `map_path`:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml map_path:=data/my_site
```

### How the three artefacts relate

They are not three independent things you obtain three independent ways. One of
them is surveyed and the other two are built on top of it:

```text
a recorded drive
      |
      | SLAM (GLIM)
      v
pointcloud_map.pcd  ------ slice a height band ----->  occupancy_grid.pgm/.yaml
      |                                                  (what mcl matches)
      | (what ndt / cuda_ndt match)
      |
      +-- drawn on top of, by hand ----> lanelet2_map.osm  (what planning follows)
```

The point cloud is the survey. The occupancy grid is a horizontal slice through
it, which is why [building a grid](#building-an-occupancy-grid) below starts from
a PCD. The lanelet2 map is drawn over the cloud by a human deciding where lanes
and stop lines go, and `map_projector_info.yaml` is what ties all of them to a
common frame — georeferenced (`TransverseMercator`, `MGRS`, …) or local
(`Local`), but the same one for every file in the directory.

Which is also why a directory holds more than its name suggests.
`data/COSS-map-planning` sounds like a planning map, and holds all four:

```text
data/COSS-map-planning/
├── pointcloud_map.pcd            74.7 MB   the survey
├── pointcloud_map_metadata.yaml            which PCD covers which tile
├── lanelet2_map.osm                        lanes, for planning
├── map_projector_info.yaml                 TransverseMercator @ 25.0201, 121.5423
├── occupancy_grid.pgm / .yaml              derived from the PCD, for mcl
└── autosdv_map.yaml                        provenance, optional
```

So a site is mapped once, and the artefacts for each `pose_source` are produced
from that one survey.

## Where the point cloud map comes from

`pointcloud_map.pcd` is the input everything else depends on, and nothing in this
repository produces it from nothing: it comes from **driving the site with the
LiDAR and IMU running, recording that, and putting the recording through a SLAM
mapper**. The mapper does the part that matters — it estimates the trajectory,
closes loops where the route revisits itself, and only then stacks the scans into
one consistent cloud. Concatenating scans at odometry poses does not work; the
drift shows up as doubled walls.

!!! warning "This section was not run for this page"

    Producing a map needs a vehicle, a site and a drive. The commands below are
    adapted from the GLIM tutorials in a sibling
    project's tutorials — they are **not** verified against a mapping session
    here. Everything from [Check a map before you trust it](#check-a-map-before-you-trust-it)
    onward was run against `data/COSS-map-planning`.

### Record the drive

Record the concatenated cloud, the IMU, and — outdoors — GNSS, plus the
transforms:

```bash
ros2 bag record -o mapping_run \
  /sensing/lidar/concatenated/pointcloud \
  /sensing/imu/imu_data \
  /sensing/gnss/ublox/nav_sat_fix \
  /tf /tf_static
```

`just bag record` is **not** the shortcut for this one. It records the fixed
list in `scripts/rosbag/outdoor_topics.txt`, which is aimed at replaying a drive
through the stack: raw `velodyne_points` and the ZED's IMU, not the concatenated
cloud and not `/sensing/imu/imu_data`. Record mapping data explicitly.

How you drive decides how good the map can be, and no amount of later processing
recovers it:

- **Slowly** — 5–10 km/h. Fast motion exposes every timing and calibration error.
- **Return to where you started**, from a similar viewpoint, so loop closure has
  overlap to work with. A route that never revisits itself cannot be corrected.
- **Cover each area from more than one direction**, with overlap between passes.
- **Stay still for a few seconds at the start** so the IMU settles.
- **Avoid crowds, traffic and foliage** where you can. Moving objects become
  smeared ghosts in the cloud, and they have to be deleted later by hand.
- **Check the recording before leaving the site** with `ros2 bag info
  mapping_run` — message counts and duration should be plausible. A bag that
  dropped point clouds cannot be repaired.

### Build the cloud with GLIM

GLIM does LiDAR-inertial graph SLAM, and what makes it the mapper to use here is
that its result stays **editable**. It keeps a dump directory — the factor graph,
the submaps, the trajectories, the configuration it used — and ships an offline
viewer that lets you add a loop closure by hand, re-optimize, delete moving
objects, and only then export. A flattened point cloud can only be re-run from
the bag; a dump can be repaired.

The topics it reads are AutoSDV's:

| GLIM configuration key | AutoSDV topic |
|---|---|
| `points_topic` | `/sensing/lidar/concatenated/pointcloud` |
| `imu_topic` | `/sensing/imu/imu_data` |

!!! note "Not run on an AutoSDV vehicle yet"

    GLIM is the mapper the sibling golf-cart project uses, and this workflow
    comes from its tutorials. GLIM is not installed by `setup.sh` and is not
    referenced anywhere in this tree, so you install it yourself and the
    commands below are GLIM's own — only the topic names are AutoSDV's.

Run it offline against the bag, with a copy of GLIM's `config` directory whose
`config_ros.json` names `/sensing/lidar/concatenated/pointcloud` and
`/sensing/imu/imu_data`, and whose `config_sensors.json` carries the real
LiDAR-to-IMU transform:

```bash
ros2 run glim_ros glim_rosbag ./mapping_run \
  --ros-args -p config_path:=$(realpath ./config)
```

On a clean exit it writes its dump to `/tmp/dump`. Move it somewhere durable
immediately — `/tmp` gets cleaned:

```bash
mkdir -p ~/glim-results
mv /tmp/dump ~/glim-results/my_site_raw
```

Then inspect and correct it before exporting anything:

```bash
ros2 run glim_ros offline_viewer   # open the dump, check alignment, add loop closures
ros2 run glim_ros map_editor       # delete parked cars and other ghosts, after the poses are right
```

The order matters: correct the trajectory first, delete points second. The map
editor freezes submap poses, so anything removed before the graph is right has
to be removed again.

GLIM exports binary PLY, not PCD. Convert only if you must, and keep the PLY:

```bash
pcl_ply2pcd ~/glim-results/my_site_final.ply data/my_site/pointcloud_map.pcd
```

Check the point count, the units (metres) and the fields after conversion.
Converters silently drop scalar fields, intensity in particular.

### Keep the recording and the mapper's own output

Keep three things, not one:

1. **The bag.** It is the only artefact that can be re-processed with a different
   mapper or different parameters.
2. **The mapper's own output** — GLIM's dump directory, or another mapper's saved
   keyframes and graph. This is what can still be loop-closed, merged with a
   later session, or re-optimized.
3. **The exported cloud**, the `.ply` or `.pcd`.

Only the third one is in the map directory, and it is the one that cannot be
repaired. A PCD is a flattened bag of points with no graph behind it: a seam
found in it six months later is not fixable in place, and without the first two
artefacts the answer is another trip to the site. Record the configuration, the
calibration and the capture date alongside them.

### Assemble the map directory

Autoware's map loader wants specific filenames, so the cloud is copied in under
the name it expects, with a metadata file that says which file covers which tile:

```bash
mkdir -p data/my_site
cp ~/glim-results/my_site_final.pcd data/my_site/pointcloud_map.pcd
```

For a single undivided cloud, `pointcloud_map_metadata.yaml` is one cell wide
enough to contain it. The COSS map's is 300 m square with its lower corner at
(−150, −150):

```yaml
x_resolution: 300.0
y_resolution: 300.0
pointcloud_map.pcd: [-150, -150]
```

`map_projector_info.yaml` declares the frame. Georeferenced, as COSS is:

```yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 25.0201
  longitude: 121.5423
  altitude: 25.0
```

or, for an indoor site with no geodetic anchor:

```yaml
projector_type: Local
vertical_datum: WGS84
```

`Local` is the one case where the lanelet2 map's own nodes must carry
`local_x` / `local_y` tags rather than latitude and longitude — `just map check`
says so explicitly if they are missing, and GNSS-based pose initialization is
unavailable with it.

The lanelet2 map is the remaining piece, and it is authored rather than
computed: someone draws lanes and stop lines over the point cloud in a vector
map editor. That work happens outside this repository, in a tool this page
cannot speak for.

Then check what you have built, which is the next section.

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

## The default map

`data/COSS-map-planning`, the COSS Park map, is the default for `map_path` and
is what the [simulation guides](../tutorial/02-planning-simulation.md) use.

## Related

- [Localization Methods](./localization-methods.md)
- [NDT Tuning](./ndt-tuning.md) — once the map exists and NDT is running on it
- `docs/design/map-handling-per-localization-method.md` in the repository

<!--
RECONCILE (phase 2 requests from W5 -- Maps):

- nav: no new page; `guides/maps.md` keeps its existing nav entry. No
  mkdocs.yml change is needed from this unit.

- cross-links INTO this page:
  - guides/localization-methods.md, in the `cuda_ndt` / `ndt` section: link to
    guides/maps.md#where-the-point-cloud-map-comes-from for "where the PCD
    comes from".
  - guides/localization-methods.md, in the `mcl` section (it already mentions
    building a grid with `just map grid-from-pcd`): link to
    guides/maps.md#building-an-occupancy-grid rather than restating it, and to
    guides/maps.md#how-the-three-artefacts-relate for why the grid is derived
    from the PCD.
  - guides/ndt-tuning.md (W3): a map that is wrong cannot be tuned out; link
    back to guides/maps.md#check-a-map-before-you-trust-it early on that page.
  - reference/commands.md (W6): `just map check`, `just map grid-from-pcd`,
    `just map grid-from-bag` -- three recipes, verified 2026-09-21 against
    `just map`; link the reference rows here.

- cross-link OUT of this page, once the target exists:
  - the [NDT Tuning](./ndt-tuning.md) link in Related is written on the
    assumption W3 creates `src/guides/ndt-tuning.md`. If that page lands under
    another name, fix this link.

- glossary (concepts/glossary.md): PCD, occupancy grid, lanelet2, SLAM, loop
  closure, GLIM, factor graph, deskewing, map projector / projector
  type (TransverseMercator vs Local).

- de-duplication note for 2.5: the map-artefact table at the top of this page
  and the "Map artefact needed" column in localization-methods.md overlap
  deliberately. Keep the table here as the detailed one and leave the other as
  a one-line pointer.

- honesty flags carried into this page, for the phase 3 audit:
  - the whole "Where the point cloud map comes from" section is unverified --
    it needs a vehicle and a drive. It says so in an admonition.
  - GLIM is adapted from `2026-golf-cart/docs/guides/glim/` and has never been
    run on AutoSDV. Marked in place.
  - verified by running, 2026-09-21: `just map` lists exactly the three
    recipes named here; `just map check data/COSS-map-planning cuda_ndt`
    reports READY. The map directory listing and all YAML contents quoted
    above were read from `data/COSS-map-planning/`.
-->
