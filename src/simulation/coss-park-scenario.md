# COSS Park Scenario

The full localization stack, replayed against a real recording, from one
command — including fetching the data. This is the closest thing AutoSDV has to
a regression test you can watch.

## The recording

`data/rosbags/outdoor_20251226_153115`, about 2.8 GB, 157 seconds at COSS Park:
parked for the first 115.7 s, then a 41 s drive at up to 1.58 m/s. Nothing about
the vehicle is required — the whole scenario runs from the recording.

## Run it

```bash
just demo check        # are the prerequisites in place?
just demo run          # everything
```

`just demo run` does the following, in order:

1. fetches the rosbag if it is missing (~2.8 GB) and checks the map is present
2. stops any stack left over from a previous run — two would fight for the same
   topics
3. launches `logging_simulation.launch.yaml` with `pose_source:=cuda_ndt`,
   `use_gnss:=false`, and RViz when `$DISPLAY` is set
4. waits for `ndt_scan_matcher`, then gives the 4.9 M-point map 25 s to load
5. starts the wheel-speed scaler and remaps the bag's raw velocity topic through
   it
6. records every localization diagnostic to `tmp/demo-runs/<label>_<stamp>/bag`
7. replays the bag, seeding the initial pose 8 s in
8. prints the metrics and **leaves the stack running** so you can inspect the
   result

Stop it when you are done:

```bash
just demo stop
```

That kills the whole process group. This matters more than it sounds: killing
the launcher by PID orphans the component containers, and `play_launch`'s own
wrapper regularly survives a group signal, which is why the recipe exists rather
than a `kill` in the documentation.

## Variants

```bash
just demo run-headless      # no RViz, and does not leave the stack up
just demo run-manual-init   # no pose seed — set it yourself in RViz
just demo run-raw-speed     # raw (uncorrected) wheel speed, to see the lurching it causes
```

`run-manual-init` is the honest version: the seeded pose in `demo run` is what
makes the run reproducible, and removing it shows you how much of the result
depended on a good initial guess.

`run-raw-speed` exists to demonstrate a real defect rather than to hide it —
the recorded wheel speed needs scaling, and without it the pose lurches.

## Reading the result

```bash
just demo report              # metrics for the most recent run
just demo report run_dir=...  # a specific run
just demo list-runs
just demo compare a=<dir> b=<dir>
```

`compare` is the regression workflow: record a run before a change, another
after, and put them side by side. Because the input is a fixed recording, a
difference in the output is a difference in the code.

Two more specialised reports:

```bash
just demo map-quality    # does the map cover the scan, and where it does, does it agree?
just demo yaw-bias       # heading-vs-course yaw bias for a run
```

Runs are several GB each. `just demo clean` deletes them.

## Benchmarking the matchers

```bash
just demo bench                 # cuda_ndt on GPU, the same code on CPU, and Autoware's
just demo bench-offline         # offline GPU-vs-CPU on identical recorded input
just demo bench-nvtl-probe      # NVTL scoring parity between the arms, at identical poses
just demo bench-report <tsv>    # rebuild a report from runs already recorded
```

`bench-offline` is described in the repository as "the fair one" — it feeds both
arms identical recorded input rather than letting them each run live, which is
the only way the comparison means anything.

## The lower-level version

`just sim coss-park` runs the same scenario more crudely — the logging
simulation, the bag, and a localization recorder, started together under GNU
`parallel` with fixed sleeps:

```bash
parallel --line-buffer ::: \
    "just sim logging" \
    "sleep 40 && ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
    "sleep 45 && ./scripts/rosbag/record_localization.sh"
```

It requires GNU `parallel`, hardcodes that bag path, and waits by clock rather
than by readiness — so on a slower machine the bag can start before the map has
loaded. `just demo run` waits for the scan matcher instead, which is why it is
the one to use.

## Next steps

- [Datasets & Rosbags](./datasets.md)
- [Localization Methods](../guides/localization-methods.md)
