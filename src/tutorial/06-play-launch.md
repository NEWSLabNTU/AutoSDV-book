# 6. play_launch

Every launch command in this book has been `play_launch launch`. This page says
what it is, why it is used here, and — the part that matters when something goes
wrong — **that it is our own software, and how to step around it.**

## What it is

A launch runner that replaces `ros2 launch`:

```bash
play_launch launch <package> <launch_file> [name:=value ...]
```

Same three positions, same arguments. It is installed by the setup program, and
from PyPI otherwise:

```bash
pip install play_launch
play_launch setcap      # optional: per-process I/O monitoring, non-root RT scheduling
```

## Why this project uses it

**It shuts the system down properly.** This is the main reason. AutoSDV runs
most nodes as composable nodes inside container processes. Killing `ros2 launch`
kills the launcher and leaves those containers running — still publishing, still
holding memory and the GPU, surviving the terminal that started them.
`play_launch` escalates SIGINT → SIGTERM → SIGKILL across the whole process
group.

**A web UI**, by default at `http://127.0.0.1:8080`, listing every node with its
state and streaming its logs. Faster than reading a merged console when 34 nodes
start at once.

**Monitoring** — per-process CPU, memory, I/O and GPU, plus `/diagnostics`.

**A resolve/replay workflow:**

```bash
play_launch resolve <pkg> <file> args…        # what would start, without starting it
play_launch dump launch <pkg> <file> args…    # write a resolved system model
play_launch up system_model.yaml              # spawn from that model, repeatedly
```

`resolve` is the one you will use most. It answers "did my argument take effect"
without launching anything.

## It is our software, and `ros2 launch` is the reference

`ros2 launch` is part of ROS 2. Every launch file in the ecosystem is written
and tested against it. `play_launch` is maintained by this project, so when a
launch file misbehaves under it, the first question is not *what is wrong with
the launch file* — it is **is this us?**

The tool's own options tell you where it can differ.

### The parser

`play_launch` reimplements the launch-file language in Rust for speed, and ships
the original Python implementation as a fallback:

```bash
play_launch launch <pkg> <file> --parser python
```

The flag is documented as being "for maximum compatibility", which is a plain
admission that the fast path is a reimplementation and can disagree with the
reference on unusual files. If a launch file fails to parse, or parses into
something you did not expect, try this before anything else.

### The container mode

By default `play_launch` overrides how composable nodes are containerised, to
get process isolation:

```bash
play_launch launch <pkg> <file> --container-mode isolated     # default: fork+exec per node
play_launch launch <pkg> <file> --container-mode observable   # shared process, ComponentEvents
play_launch launch <pkg> <file> --container-mode stock        # no override at all
```

`stock` uses the container the launch file asked for and changes nothing. If
composable nodes behave oddly — a node that will not load, or one that works
under `ros2 launch` and not here — this is the setting to try.

## The fallback ladder

In order. Each step gives up one of `play_launch`'s behaviours:

```bash
play_launch launch <pkg> <file> …                        # 1. default
play_launch launch <pkg> <file> --parser python          # 2. if it parses wrong
play_launch launch <pkg> <file> --container-mode stock    # 3. if composable nodes misbehave
ros2 launch <pkg> <file> …                                # 4. the reference; always available
```

If you reach step 4, remember you have also given up the shutdown handling, so
kill by process group:

```bash
kill -- -$(ps -o pgid= -p <pid> | tr -d ' ')
ros2 node list    # confirm it is empty
```

## Which one is at fault

A simple rule:

> **If it works under `ros2 launch` and not under `play_launch`, that is a
> `play_launch` bug.**

Report it to [play_launch](https://github.com/NEWSLabNTU/play_launch), not as an
AutoSDV issue. Include the launch file, the arguments, and which rung of the
ladder made it work — that last detail usually identifies the component
responsible.

Conversely, if it fails under both, the launch file or the configuration is at
fault and `play_launch` is an innocent bystander.

This test costs one command and saves a great deal of misdirected debugging,
which is the only reason this page exists.

## The port confusion

Worth knowing because it looks like a failure:

| Started with | Web UI |
|---|---|
| `play_launch launch …` | `http://127.0.0.1:8080` |
| `just launch`, `just sim …`, `just demo run` | `http://localhost:8081` |

The `just` recipes pass `--web-addr 0.0.0.0:8081`. Opening the wrong port shows
nothing, and looks exactly like a system that failed to start.

`0.0.0.0` also means the UI is reachable from other machines on your network,
which is convenient on a vehicle and worth knowing about on a shared one.

## Upgrading

`play_launch` has had breaking changes — `replay` became `up` in 0.9.0, among
others, and its changelog notes that some of them change behaviour rather than
erroring. The setup program installs a version known to work with this
repository. If you upgrade it independently and launches start behaving oddly,
that is a likely cause.

## You have finished the tutorial

You can now:

- run the whole stack with one command, and stop it without leaving orphans
- drive the planning simulator and read the state panel
- replay real sensor data and tell whether localization is actually working
- pass arguments, and check whether they took effect
- drop a layer when something breaks

Where to go next:

- **[Localization Methods](../guides/localization-methods.md)** — what else
  `pose_source` can be
- **[Presets](../guides/presets.md)** — configuring perception and localization
- **[Operating the Vehicle](../getting-started/usage.md)** — the full argument
  reference
- **[Maps](../guides/maps.md)** — building a map for your own site
