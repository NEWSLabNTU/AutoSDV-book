# Sensor Kit Configuration

The sensor kit consists of the _description_ package and the _launch_ package. The _description_ package, [`autosdv_sensor_kit_description`](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch/tree/main/autosdv_sensor_kit_description), stores the relative coordinates for each sensor on the vehicle. The _launch_ package, [`autosdv_sensor_kit_launch`](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch/tree/5daa6018ef4814dafaad262135ca1d50154cdb2e/autosdv_sensor_kit_launch), contains a set of launch files for all kinds of sensors along with their runtime parameters.

> **Notice**
>
> The sensor kit defines the composition and the data paths of sensors. If you're looking per-sensor driver configuration, please refer to [Sensor Component](../software/sensor-components.md) Chapter.

## The Description Package

The description package stores the coordinates of each sensor installed on the vehicle. It's done by working on these two configuration files.

- `../autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml`
- `../autosdv_sensor_kit_description/urdf/sensor_kit.xacro`

For example, the coordinate for the ZED camera is named as `zedxm_camera_link`, which pose parameters are defined in [`sensor_kit_calibration.yaml`](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch/blob/main/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml).

```yaml
sensor_kit_base_link:
  zedxm_camera_link: # Zed Camera
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

The [`sensor_kit.xacro`](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch/blob/main/autosdv_sensor_kit_description/urdf/sensor_kit.xacro) file has corresponding entries for the coordinate. In the xacro snipplet, it defines a `<xacro:zed_camera>` component and a joint from `sensor_kit_base_link` to `zedxm_camera_link`.

```xml
    <xacro:zed_camera name="zedxm" model="zedxm" custom_baseline="0" enable_gnss="false">
      <origin
        xyz="${calibration['sensor_kit_base_link']['zedxm_camera_link']['x']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['y']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['z']}"
        rpy="${calibration['sensor_kit_base_link']['zedxm_camera_link']['roll']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['pitch']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['yaw']}"
      />
    </xacro:zed_camera>

    <joint name="zedxm_camera_joint" type="fixed">
      <origin
        xyz="${calibration['sensor_kit_base_link']['zedxm_camera_link']['x']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['y']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['z']}"
        rpy="${calibration['sensor_kit_base_link']['zedxm_camera_link']['roll']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['pitch']}
             ${calibration['sensor_kit_base_link']['zedxm_camera_link']['yaw']}"
      />
      <parent link="sensor_kit_base_link"/>
      <child link="zedxm_camera_link"/>
    </joint>
```

## The Launch Package

The [`autosdv_sensor_kit_launch`](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch/tree/main/autosdv_sensor_kit_launch) package contains a collection of launch files in the [`launch`](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch/tree/main/autosdv_sensor_kit_launch/launch) directory. The notable one is `sensing.launch.xml`. It is the mother launch file used to start the whole sensing module and all the other launch files are included.

The `camera.launch.xml`, `gnss.launch.xml`, `imu.launch.xml` and `lidar.launch.xml` launch files correspond to respective sensing functions. Each of them contains sensing drive execution methods and their parameters.

The `pointcloud_preprocessor.launch.py` is the special one that provides the multi-LiDAR fusion feature. It includes a point cloud processor node that subscribes to one or multiple input topics from LiDARs drivers.

```python
        parameters=[
            {
                "input_topics": [
                    "/sensing/lidar/bf_lidar/points_raw",
                ],
                "output_frame": LaunchConfiguration("base_frame"),
                "input_twist_topic_type": "twist",
                "publish_synchronized_pointcloud": True,
            }
        ],
```
