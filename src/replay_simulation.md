# Replay Simulation

This is the guide to run a replay simulation on AutoSDV replaying a pre-recorded ros2 bag.

Please refer to the node diagram [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/overall-node-diagram-autoware-universe.drawio.svg) for the whole localization module workflow and datapath.

## Steps
*Steps 1 and 2 are done on the AutoSDV vehicle, so you don't have to do them again.*
1. Clone the launch tool repo developed by Jerry [here](https://github.com/NEWSLabNTU/ros-launch-perf) into `~/AutoSDV`.
2. Build the above tool based on the repo's README guide.
3. Play the ros bag: `ros2 bag play "./path/to/rosbag --clock`. The ros bag we use to test the simulation is in `~/AutoSDV/scripts`.
4. Launch the replay simulation:

```bash
dump_launch ./launch/logging_simulator.launch.yaml
play_launch --load-node-timeout-millis 60000 --max-concurrent-load-node-spawn 20 --load-orphan-composable-nodes
```

5. Wait for all nodes to be loaded completely.

## Observation
1. Check the log path specified on the terminal if there is any error.
2. Trace the topic flow in the node diagram. For example, if you want to check if the topic `/sensing/lidar/concatenated/pointcloud`,<br>
Use `ros2 topic info -v /sensing/lidar/concatenated/pointcloud` to check the subscriber and publisher, and `ros2 topic echo /sensing/lidar/concatenated/pointcloud` to check the actual data.
3. You may observe the connection status of frames by this command: `ros2 run tf2_tools view_frames`
