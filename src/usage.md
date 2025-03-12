# Drive the Vehicle

Before start reading this article, please make sure you followed the
[installation guide](installation.md) and built the project. The
project repository has a launch file `f1eighth.launch.yaml` that
defines the set of nodes to be executed and assigned parameters to
start the whole driving system.

## The Simple Way

### Startup

The Makefile has a receipt to start the whole system.

```sh
make launch
```

### Termination

The recommended way to terminate the launch is to press the _Ctrl-Z +
kill_ combo. The key sequence goes like this.

| Step | Shell Output                                | Action                             |
|------|---------------------------------------------|------------------------------------|
| T    | `$ make launch`                             | Launch the system.                 |
| T+1  |                                             | Press Ctrl-Z to freeze the launch. |
| T+1  | `[1]+  Stopped                 make launch` | Read job number `[1]`.             |
| T+3  | `$ kill %1`                                 | Kill the `%1` job number.          |

The trick is preferred rather than Ctrl-C because the launch system
sometimes cannot handle the Ctrl-C properly and hang in the middle of
termination process, leaving orphan processes in the background.

## The Advanced Way: Launch Customization

You can either modify the launch file directly located here:

```
F1EIGHTH/src/autoware/launcher/f1eighth_launch/launch/f1eighth.launch.yaml
```

or assign argument values to the launch command. For example, to set
 `launch_sensing_driver` to false.


```sh
source install/setup.sh
ros2 launch f1eighth_launch f1eighth.launch.yaml launch_sensing_driver:=false
```

### Arguments

| Argument                | Value                                         | Default                    |
|-------------------------|-----------------------------------------------|----------------------------|
| `vehicle_model`         | The name of the vehicle model.                | `f1eighth_vehicle`         |
| `sensor_model`          | The name of the sensor model.                 | `f1eighth_sensor_kit`      |
| `map_path`              | The path to the map data directory.           | `./data/COSS-map-planning` |
| `launch_vehicle`        | Whether to launch the vehicle interface.      | `true`                     |
| `launch_system`         | Whether to launch the system compoment.       | `false`                    |
| `launch_map`            | Whether to launch the map compoment.          | `false`                    |
| `launch_sensing`        | Whether to launch the sensing compoment.      | `true`                     |
| `launch_sensing_driver` | Whether to launch sensor drivers.             | `true`                     |
| `launch_localization`   | Whether to launch the localization compoment. | `false`                    |
| `launch_perception`     | Whether to launch the perception compoment.   | `false`                    |
| `launch_planning`       | Whether to launch the planning compoment.     | `false`                    |
| `launch_control`        | Whether to launch the control compoment.      | `true`                     |
| `pose_source`           | The localization method.                      | `eagleye`                  |
