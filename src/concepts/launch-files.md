# Launch Files

Nearly everything in AutoSDV is a launch file. The whole driving system is one;
so is each sensor driver, each component group, and every preset. If you are
going to change how AutoSDV runs, you will do it by passing a launch argument or
by editing a launch file — so it is worth twenty minutes to understand how they
work.

**Read this with the files open.** You already ran two of them in the tutorial;
they are in your checkout, and every example below names a real path:

```bash
cd ~/AutoSDV
$EDITOR src/launcher/autosdv_launch/launch/autosdv.launch.yaml
ls src/launcher/autosdv_launch/config/perception/preset/
```

This page covers what AutoSDV actually uses. For the full launch system, see the
[ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html).

## What a launch file is

A program that starts other programs. It says which nodes to run, what
parameters to give them, which topics to remap, and which other launch files to
include. Running one is:

```bash
ros2 launch <package> <launch_file> [name:=value ...]
```

The package is an installed ROS 2 package; the launch file is a file inside its
`launch/` directory. AutoSDV's top-level one is:

```bash
ros2 launch autosdv_launch autosdv.launch.yaml
```

!!! note "In this project you will usually type `play_launch launch` instead"

    Same three positions, same arguments. It is our own launch runner and it is
    worth knowing why, and when not to — see
    [play_launch](../tutorial/06-play-launch.md). Everything on this page is
    true of both.

## Three file formats, one meaning

ROS 2 accepts launch files written in Python, XML or YAML, and AutoSDV uses all
three. They express the same things.

=== "YAML"

    ```yaml
    launch:
      - arg:
          name: pose_source
          default: "cuda_ndt"
      - node:
          pkg: my_package
          exec: my_node
    ```

=== "XML"

    ```xml
    <launch>
      <arg name="pose_source" default="cuda_ndt"/>
      <node pkg="my_package" exec="my_node"/>
    </launch>
    ```

Python files are used where logic is needed — a loop, a condition that is
awkward to express declaratively. YAML and XML are preferred everywhere else
because they are easier to read and to diff.

## Arguments versus parameters

These are different things and the difference causes real confusion.

An **argument** is an input to the *launch file*. It exists while the launch file
is being evaluated and then it is gone.

```yaml
- arg:
    name: pose_source
    default: "cuda_ndt"
    description: "Pose estimation source"
```

You set one on the command line:

```bash
ros2 launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

Note `:=`, not `=`. And note that **every value is a string** — there are no
typed arguments, so a boolean is the literal text `true` or `false`, lowercase.

A **parameter** belongs to a *node*, exists as long as the node runs, and can be
read and sometimes written at runtime:

```bash
ros2 param list /some_node
ros2 param get /some_node some_parameter
```

The connection between them is that a launch file takes its arguments and hands
them to nodes as parameters:

```yaml
- node:
    pkg: my_package
    exec: my_node
    param:
      - name: source
        value: "$(var pose_source)"
```

So `pose_source:=ndt` on your command line becomes the node parameter `source`
with the value `ndt`. That chain — command line → launch argument →
substitution → node parameter — is the one to hold on to.

## Substitutions

A substitution is a value computed while the launch file is evaluated. Four
appear constantly in this project.

| Substitution | Meaning |
|---|---|
| `$(var name)` | the value of a launch argument |
| `$(find-pkg-share pkg)` | the installed `share/` directory of a package |
| `$(env NAME default)` | an environment variable, with a fallback |
| `$(eval "...")` | a small Python expression |

`find-pkg-share` is how launch files refer to files inside packages without
hard-coding a path:

```yaml
rviz_config: "$(find-pkg-share autosdv_launch)/rviz/autosdv.rviz"
```

This resolves through the same `AMENT_PREFIX_PATH` that
[the environment page](./environment.md) describes — which is another way of
saying that a missing `source` line breaks launch files too, and does it with an
error about a package rather than about your environment.

`$(env ...)` is how the model directory picks up an override:

```yaml
default: "$(env AUTOSDV_DATA_PATH ./data/autoware_data)"
```

## Includes, and how arguments travel

A launch file can run another one, passing values down:

```yaml
- include:
    file: "$(find-pkg-share autosdv_launch)/launch/components/tier4_localization_component.launch.xml"
    arg:
      - name: pose_source
        value: "$(var pose_source)"
```

This is how AutoSDV is structured: `autosdv.launch.yaml` declares the arguments
and includes component launch files, which include the launch files of individual
packages.

**An argument does not travel automatically.** If an include does not pass
`pose_source` down, the included file uses its own default and your command-line
value is silently ignored. That class of bug — a value accepted on the command
line that reaches nothing — is the reason the next section exists.

## Seeing what a launch file actually resolved to

Do not guess. Resolve the launch without running it:

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl -o ./tmp/resolved.yaml
```

The output lists every node that would start and every parameter it would get.
This answers "did my argument take effect", and it costs nothing — no node is
launched.

To see just the arguments a launch file accepts:

```bash
ros2 launch autosdv_launch autosdv.launch.yaml --show-args
```

## Presets are launch files

Once arguments and includes make sense, AutoSDV's preset system stops being a
separate feature.

A preset is a launch file that declares nothing but argument defaults:

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
```

and the main launch file includes one by name:

```yaml
- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

Because the preset supplies *defaults*, and a default only applies when nothing
else provides a value, an argument you pass on the command line beats the preset:

```bash
ros2 launch autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion \
  use_traffic_light_recognition:=false
```

That is the entire mechanism. See [Presets](../guides/presets.md).

## Editing launch files

The workspace is built with `--symlink-install`, which means `install/` contains
links to your source files rather than copies. So:

- **Editing an existing** `.yaml`, `.xml` or `.py` takes effect on the next
  launch, with no rebuild.
- **Adding a new file** requires `just build`, to create its symlink. Until then
  the file exists in `src/` and does not exist as far as ROS is concerned.

That asymmetry catches people who add a preset and find it is not found.

## Composable nodes, briefly

Some ROS 2 nodes can be loaded into a shared process — a **component container**
— instead of each running as its own process. They then pass messages within one
process rather than over the network, which for point clouds is a large saving.

Autoware uses this heavily, and it has two consequences you will meet:

- Killing a launch command by PID leaves containers running as orphans, still
  holding memory and the GPU. Kill the process group instead.
- The CUDA point cloud pipeline *requires* its stages to be in one container,
  because they hand each other GPU pointers rather than data. See
  [the CUDA pipeline](../guides/cuda-pipeline.md).

## Next

- [Inspecting a Running System](./inspecting.md) — now that it is running
- [Autoware Conventions](./autoware-conventions.md) — what the topic names mean
- [Operating the Vehicle](../getting-started/usage.md) — the full argument list
