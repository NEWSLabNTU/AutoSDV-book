# Drive the Vehicle

Before start reading this article, please make sure you followed the [installation guide](installation.md) and built the project. The project repository has a launch file `autosdv.launch.yaml` that defines the set of nodes to be executed and assigned parameters to start the whole driving system.

## The Simple Way

The Makefile has a receipt to start the whole system.

```sh
make launch
```

## Customize the Launch

You can either modify the launch file directly located here:

```
AutoSDV/src/autoware/launcher/autosdv_launch/launch/autosdv.launch.yaml
```

or assign argument values to the launch command. For example, to set `launch_sensing_driver` to false.


```sh
source install/setup.sh
ros2 launch autosdv_launch autosdv.launch.yaml launch_sensing_driver:=false
```

### Arguments

| Argument                | Value                                         | Default                    |
|-------------------------|-----------------------------------------------|----------------------------|
| `vehicle_model`         | The name of the vehicle model.                | `autosdv_vehicle`          |
| `sensor_model`          | The name of the sensor model.                 | `autosdv_sensor_kit`       |
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
