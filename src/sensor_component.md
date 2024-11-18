# Sensor Component

The
[`sensor_component`](https://github.com/NEWSLabNTU/F1EIGHTH/tree/main/src/autoware/sensor_component)
directory contains a collection of drivers data processros for sensors
on the F1EIGHTH vehicle. They are mostly provided by vendors and
existing open source projects.

> **Notice**
>
> The sensor component defines the collection of sensor drivers in
> Autoware. If you're looking for the composition of the sensor
> drivers, please refer to [Sensor Kit](sensor_kit.md) Chapter.

The F1EIGHTH Autoware adds the following ROS packages along with
official packages.

- ZED X Mini camera
- Blickfeld Cube1 LiDAR
- MPU9250 Nine-Axis Motion Sensor

## ZED X Mini Camera

The ROS2 package requires ZED SDK 4.2 to be installed on the
system. ZED SDK is installed by the setup script described in
[Installation Guide](installation.md). The driver package is located
at:

```
src/autoware/sensor_component/external/zed-ros2-wrapper
```

To run the standalone ZED camera driver,

```sh
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
```

## Blickfeld Cube1 LiDAR

The driver package is located at

```
src/autoware/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5
```

To run the standalone driver,

```sh
ros2 launch blickfeld_driver live_scanner_node.launch.py
```

## MPU9250 Nine-Axis Accelerometer Gyroscope Sensor

MPU9250 measures the motion state of the vehicle, including the linear
acceleration, angular acceleration and angular speed. The source
package is located at

```
src/autoware/sensor_component/external/ros2_mpu9250_driver/include/mpu9250driver
```

To run the standalone driver,

```sh
ros2 run mpu9250driver mpu9250driver
```
## Garmin GPS 18x 5Hz
[Full specification of the sensor](https://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf)

1. Localise the device at first. Execute:
```
sudo dmesg
```
- you should find this line: *FTDI USB Serial Device converter now attached to **ttyUSB0***

2. Once you find the device, get raw data from it by executing:
```
sudo cat /dev/ttyUSB0
```

And you should see data in NMEA format like this:
```
$GPGSA,A,1,,,,,,,,,,,,,,,*1E
$GPGSV,3,1,10,10,76,279,22,12,34,053,23,23,51,164,46,24,10,053,27*75
$GPGSV,3,2,10,25,65,108,26,28,42,271,25,29,04,141,25,31,15,250,18*7B
$GPGSV,3,3,10,32,44,335,21,26,05,208,00*74
```

Note: If you see something else, for example binary data, make sure, you use the correct baund rate - 4800.

2.1 Check the current bound rate by this command:
```
stty -F /dev/ttyUSB0
```
Set the 4800 baund rate by this command:
```
sudo stty -F /dev/ttyUSB0 4800
```

3. Execute the GPS Deamon (gpsd) for the right device:
```
sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0
```

To verify the signal, you can open the CLI app:
```
cgps
```
or GUI app:
```
xgps
```

- Wait till you get the proper longitute and latitude coordination and the status of the GPS must be 3D Fix. If there is 'No Fix', no good signal is receiving.

4. Since you get the signal, you can launch Autoware and subscribe the topic:
```
 ros2 topic echo /sensing/gnss/garmin/fix
```

### Configuration files in Autoware
If you do not get data on the topic, make sure, the configuration is correct by checking these files:

1. Enable the GNSS Driver at:
```
F1EIGHTH/src/autoware/sensor_kit/f1eighth_sensor_kit_launch/f1eighth_sensor_kit_launch/launch/sensing.launch.xml
```

2. Enable the Garmin Driver at:
```
F1EIGHTH/src/autoware/sensor_kit/f1eighth_sensor_kit_launch/f1eighth_sensor_kit_launch/launch/gnss.launch.xml
```

3. No mistake in the Python script at (the script composes configuration to launch the GNSS Driver):
```
F1EIGHTH/src/autoware/sensor_component/external/gps_umd/gpsd_client/launch/gpsd_client-launch.py
```
-  Note: the topic *fix* is remaped as *garmin/fix*

3.1 Parameters for the Python script is at:
```
 F1EIGHTH/src/autoware/sensor_component/external/gps_umd/gpsd_client/config/gpsd_client.yaml
```

4. GNSS Client connecting to the gpsd (GPS Deamon) is located at:
```
 F1EIGHTH/src/autoware/sensor_component/external/gps_umd/gpsd_client/src/client.cpp
```



