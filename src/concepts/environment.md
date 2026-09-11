# The Environment, and Where Dependencies Come From

Read this before the first time something says "package not found".

Almost every early failure in AutoSDV is one mistake: a terminal that has not
been told where ROS 2, Autoware and the AutoSDV workspace live. The mistake is
easy to make because the project also ships a tool that hides it, and because
the error messages point at the wrong thing.

## AutoSDV does not install Autoware the way Autoware does

If you have read the Autoware documentation, you arrived expecting to clone a
workspace, run `vcs import` over dozens of repositories, and spend an hour or
more in `colcon build`.

**AutoSDV does not do that.** Autoware arrives as **Debian packages**, installed
into `/opt/autoware/1.5.0`. There is no Autoware source tree on your disk and
nothing of Autoware is compiled on your machine.

The reason is cost. A source build of Autoware on a Jetson AGX Orin takes hours,
and it takes them again after every clean checkout. A student setting up for a
lab session, or an integrator provisioning a vehicle, should not pay that.

!!! note "The trade"

    You get an installation in minutes instead of hours. In exchange you take
    the version that was built for you — `1.5.0`, pinned in `versions.yaml` —
    and you cannot patch Autoware's own sources without
    [building it yourself](../getting-started/installation/manual-environment.md).

**AutoSDV itself is still compiled.** Everything under `src/` is a colcon
workspace and `just build` builds it: the launch files, the sensor kit, the
vehicle interface, the CUDA NDT matcher.

So you are always in a hybrid: **a binary Autoware underneath, a source
workspace on top.** That shape is what the rest of this page is about.

## The two lines

Every terminal that will run anything needs both of these:

```bash
source /opt/autoware/1.5.0/setup.bash   # ROS 2 Humble AND the Autoware packages
source install/setup.bash               # the AutoSDV workspace, from src/
```

The second one is run from the repository root, and only works after
`just build` has produced an `install/` directory.

### The first line is doing more than it looks

You might expect to need three commands — one for ROS 2, one for Autoware, one
for the workspace. You need two, because `/opt/autoware/1.5.0/setup.bash`
sources ROS 2 itself before adding Autoware:

```bash
# inside /opt/autoware/1.5.0/setup.bash
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
...
    source "$AUTOWARE_HOME/local_setup.bash"
```

One command, two layers. This is not something you could guess, and it is why
instructions elsewhere that begin `source /opt/ros/humble/setup.bash` are not
wrong so much as incomplete here.

It also prints a few lines when it runs — a note that `ROS_LOCALHOST_ONLY` was
unset for CycloneDDS, and warnings if the kernel network settings are not
configured. Those are informational. The line that matters is
`Autoware 1.5.0 environment loaded.`

## What each layer actually gives you

This is the whole point of the page, so here it is measured rather than
asserted. Each row is a completely clean shell with nothing inherited:

| Sourced | `ros2` on PATH | `ros2 pkg prefix autoware_launch` | `ros2 pkg prefix autosdv_launch` |
|---------|----------------|-----------------------------------|----------------------------------|
| nothing | **not found** | — | — |
| `/opt/ros/humble/setup.bash` | yes | **Package not found** | **Package not found** |
| `/opt/autoware/1.5.0/setup.bash` | yes | `/opt/autoware/1.5.0` | **Package not found** |
| both lines above | yes | `/opt/autoware/1.5.0` | `…/AutoSDV/install/autosdv_launch` |

Read the third row carefully. With Autoware sourced you can run `ros2`, you can
launch Autoware's own files, and **AutoSDV is still invisible**. With only the
workspace sourced, the reverse: your packages are found and the Autoware
messages and nodes they are built against are not.

**This is where your dependencies come from.** Not from the build — the build
only produced `install/`. The ROS 2 and Autoware packages that AutoSDV's nodes
link against, the message types they publish, the launch files they include, all
resolve at *run* time through this chain. A missing layer is a missing
dependency, every time.

## Overlays, briefly

ROS 2 calls each sourced layer an **overlay** over the ones beneath it — the
underlay. Sourcing appends to a search path, `AMENT_PREFIX_PATH`, and packages
are found by walking it.

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n'
```

On the clean shells above that path had 1 entry with ROS 2 alone, 2 with
Autoware added — Autoware is a single merged install, so it contributes one
prefix — and 35 once the AutoSDV workspace was sourced, because a
`--symlink-install` workspace contributes one entry per package.

Two consequences worth keeping:

- **Order matters.** Source the underlay first. Sourcing the workspace before
  Autoware puts the layers in the wrong order, and although it often appears to
  work, package resolution is no longer what you think it is.
- **It is per-terminal.** Sourcing changes environment variables in *one shell*.
  A new tab, a new SSH session, a new terminal split — each one starts clean and
  needs both lines again. Nothing is written to disk and nothing persists.

## What it looks like when a layer is missing

These are the errors, and what they actually mean.

### `ros2: command not found`

Nothing is sourced. You are in a fresh terminal.

### `Package 'autosdv_launch' not found`

Autoware is sourced, the workspace is not. Either you forgot the second line, or
you are not in the repository root, or `just build` has not run yet.

```bash
ls install/setup.bash    # if this is missing, build first
```

### The error names an Autoware package, and you were working on AutoSDV

For example a launch file cannot find `autoware_launch`, or a node fails to
start on a missing message type. **This is the confusing one.** The message
names an AutoSDV file, so it looks like an AutoSDV bug, and it is not — you
sourced `install/setup.bash` without the Autoware layer under it.

Check before you debug anything else:

```bash
ros2 pkg prefix autoware_launch   # expect /opt/autoware/1.5.0
ros2 pkg prefix autosdv_launch    # expect …/AutoSDV/install/autosdv_launch
```

If the first fails, the underlay is missing. Nothing else you are looking at is
the real problem.

### It works in one terminal and not another

The first terminal is sourced and the second is not. This is also why "it works
on my machine" happens — an inherited environment carries the variables into
child processes, so a shell started *from* a sourced shell already has them.

## Now the conveniences

Everything above is the real mechanism. What follows only automates it, and you
should be able to recognise that it does nothing else.

### direnv

The repository ships an `.envrc` which runs those same two lines whenever you
enter the directory:

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc   # or your shell's equivalent
source ~/.bashrc

cd ~/AutoSDV
direnv allow
```

After `direnv allow`, entering the directory sources Autoware, then — if
`install/setup.bash` exists — the workspace. The relevant part of the file is
short enough to read:

```bash
if [ -f /opt/autoware/1.5.0/setup.bash ]; then
    source /opt/autoware/1.5.0/setup.bash
...
if [ -f install/setup.bash ]; then
    watch_file install/setup.bash
    source install/setup.bash
fi
```

That is all it is. It is genuinely convenient, and it is also why a newcomer can
work for a week without learning any of this, and then be stranded the first time
they open a terminal somewhere else — over SSH, in a container, in an editor's
built-in shell, or in a `cron` job.

**If you skip direnv, nothing breaks.** Type the two lines.

### `just`

The same relationship. `just build` runs a `colcon build` with specific flags;
`just launch` runs a `play_launch` command with specific arguments. Each recipe
sources what it needs internally, which is why `just build` works in an
unsourced terminal.

Throughout this book, a `just` recipe is shown next to the command it wraps.
Use the recipes — they are shorter and they carry flags that matter — but know
what is underneath, because the day something fails you will need to run that
directly.

### Checking your environment in one line

```bash
ros2 pkg prefix autoware_launch && ros2 pkg prefix autosdv_launch
```

Two paths printed means both layers are present and you can stop thinking about
this page.

## Summary

- Autoware is installed as **Debian packages**; **AutoSDV is compiled**.
- Two lines, in order: `/opt/autoware/1.5.0/setup.bash`, then
  `install/setup.bash`. The first also brings in ROS 2.
- Those lines are **where package dependencies are resolved**, at run time.
- Every new terminal needs them again.
- `.envrc` and `just` automate exactly this and nothing more.

## Next

- [Launch Files](./launch-files.md) — what you will be running, once the
  environment is right
- [Inspecting a Running System](./inspecting.md) — topics, QoS and frame rates
- [Software Installation](../getting-started/installation/overview.md)
