# COSS Park Scenario

The COSS Park scenario runs a full-stack simulation using recorded sensor data from NTU COSS Park. It launches the Autoware stack, plays the rosbag, and records the localization output — all in a single command.

**Prerequisites**:

- Complete the [Recommended Installation](../getting-started/installation/recommended.md)
- Download the test rosbag: `just download-data`
- Install GNU parallel: `sudo apt install parallel`

## Launch

```bash
just sim-coss-park
```

This runs three processes in parallel:

1. **Logging simulation** — launches the full Autoware stack (`just launch-sim-logging`)
2. **Rosbag playback** — plays the COSS Park recording in a loop at 1x speed (starts after 40 seconds)
3. **Localization recording** — records `/localization/pose_estimator/*` topics for 60 seconds (starts after 45 seconds)

The delays ensure the Autoware stack is fully started before data begins flowing.

## What This Tests

Unlike the [logging simulation](./logging-simulation.md) where you launch and play manually, this scenario automates the entire pipeline and captures localization output for analysis. It is useful for:

- Verifying the localization stack works end-to-end
- Comparing localization performance across code changes
- Regression testing after modifying launch files or parameters

## Inspecting Results

The localization recording is saved to `rosbags/localization_test_<timestamp>/`. Inspect it with:

```bash
ros2 bag info rosbags/localization_test_*/
```

Visualize the recorded pose traces:

```bash
just tool-plotjuggler
```

In PlotJuggler, load the recorded bag and plot `/localization/pose_estimator/pose_with_covariance` to see the estimated trajectory.

## Scope Inspection

Use `play_launch dump` to inspect the launch graph without running any nodes:

```bash
play_launch dump -o tmp/scope.json \
    launch autosdv_launch logging_simulation.launch.yaml \
    pose_source:=ndt

play_launch context tmp/scope.json --tree
```

This shows the full include tree and node hierarchy, useful for verifying that launch file changes produce the expected structure.

## Troubleshooting

**`parallel: command not found`**

Install GNU parallel:

```bash
sudo apt install parallel
```

**Rosbag not found**

Download the test data first:

```bash
just download-data
```

**Localization recording is empty (0 messages)**

This can happen if NDT does not initialize during the recording window. Check the NDT logs:

```bash
cat play_log/latest/node/ndt_scan_matcher/err
```

If NDT reports "No InputSource", the pointcloud preprocessing nodes may have failed to load. Try restarting the scenario.

## Next Steps

- [Datasets and Rosbags](./datasets.md) — available recordings and how to create your own
- [Operating the Vehicle](../getting-started/usage.md) — launch parameters for live driving
