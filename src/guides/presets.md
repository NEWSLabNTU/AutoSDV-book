# Presets

A preset is a named bundle of launch argument defaults. AutoSDV uses them the
way Autoware does: to group the settings that always change together, so you
select a scenario rather than remembering six arguments.

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion
```

## How they work

A preset file is a launch file that declares nothing but arguments and their
defaults:

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
  - arg:
      name: use_detection_by_tracker
      default: "false"
  - arg:
      name: use_image_segmentation_based_filter
      default: "false"
  - arg:
      name: use_pointcloud_map
      default: "true"
```

The main launch file includes one by name:

```yaml
- arg:
    name: perception_preset
    default: "lidar_only"

- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

The mechanism is that the preset supplies **defaults**, and a default is only
used when nothing else supplies a value. So an argument you pass on the command
line beats the preset:

```bash
# the fusion preset, but without traffic lights
play_launch launch autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion \
  use_traffic_light_recognition:=false
```

That is the whole design: presets for convenience, individual arguments for
experiments.

## Perception presets

`config/perception/preset/`

| Preset | `perception_mode` | Traffic lights | Detection by tracker | Image segmentation filter |
|--------|-------------------|----------------|----------------------|---------------------------|
| `lidar_only` *(default)* | `lidar` | off | off | off |
| `camera_lidar_fusion` | `camera_lidar_fusion` | on | on | on |
| `minimal` | — | off | off | off |

- **`lidar_only`** — the default. No camera features, so no camera means no
  missing dependency.
- **`camera_lidar_fusion`** — needs a working camera. It also adds three
  traffic-light models to the set TensorRT must compile, which is worth knowing
  before the first launch on a fresh machine.
- **`minimal`** — for development and debugging, when you want the stack up and
  perception out of the way.

To disable perception entirely, do not reach for a preset — use the argument:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml launch_perception:=false
```

That publishes empty object lists and loads no model at all.

## Localization presets

`config/localization/preset/`

| Preset | `twist_source` | Needs |
|--------|----------------|-------|
| `default` | `gyro_odom` | nothing extra |
| `eagleye` | `eagleye` | GNSS |

These select the **twist** (velocity) estimator, which is a separate question
from `pose_source`. `default` fuses gyro and wheel odometry; `eagleye` derives
odometry from GNSS and needs a receiver with a usable fix.

```bash
play_launch launch autosdv_launch autosdv.launch.yaml localization_preset:=eagleye
```

## Writing your own

1. Copy an existing preset:

   ```bash
   cd src/launcher/autosdv_launch/config/perception/preset
   cp lidar_only_preset.yaml custom_preset.yaml
   ```

2. Change the defaults in the new file.

3. Use it:

   ```bash
   play_launch launch autosdv_launch autosdv.launch.yaml perception_preset:=custom
   ```

Two requirements:

- **The filename must be `<name>_preset.yaml`.** The include interpolates
  `$(var perception_preset)_preset.yaml`, so the suffix is not a convention, it
  is the lookup.
- **A new file needs `just build`** to create its symlink, even though the
  workspace is built with `--symlink-install`. Edits to an *existing* preset
  take effect immediately; a new one does not exist until it is installed.

## Checking what a preset resolved to

Presets make the effective configuration less obvious, so verify rather than
assume:

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion -o ./tmp/resolved.yaml
```

This is also how to re-derive the model set that `just build-engines` compiles,
when a preset changes which models the stack actually loads.

## Related

- [Operating the Vehicle](../running/on-the-vehicle.md) — the full argument set
- `config/{perception,localization}/preset/README.md` in the repository
