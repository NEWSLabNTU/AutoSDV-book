# Docker Setup (unmaintained)

!!! danger "This image does not build. Do not start here."

    The Dockerfile in `docker/` cannot be built against the current
    repository. Use [the standard installation](./overview.md) instead.

    Two independent reasons:

    1. **Its build step no longer exists.** The Dockerfile's final step runs
       `./scripts/setup-dev-env/setup-dev-env.sh -y`. That script was removed
       when the setup program was rewritten around a step registry; there is no
       `scripts/setup-dev-env/` directory in the repository any more, so the
       build fails at that line.
    2. **Its base image is the wrong platform generation.** It builds
       `FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel`, a JetPack 5 era image.
       Autoware 1.5.0 for arm64 targets JetPack 6.2 and the TensorRT that comes
       with it.

    Fixing it is a real piece of work — a new base image, and a Dockerfile that
    drives `./setup.sh --run --profile ci --yes` rather than a deleted script.
    Until someone does that, this page stays as a record of what was here.

## What it was for

The image built an NVIDIA L4T environment, cloned AutoSDV at the exact commit of
your local checkout, and ran the setup script inside it — so a container matched
the code state on your machine rather than a branch tip.

The files are still in the repository under `docker/`:

| File | Purpose |
|------|---------|
| `Dockerfile` | the image definition described above |
| `Makefile` | `build`, `run`, `run-rocker`, `save` targets; passes the local commit hash as a build arg |
| `nvidia-l4t-apt-source.list` | the L4T apt source used inside the image |
| `README.md` | the original instructions |

## What to do instead

Install on the host. [Software Installation](./overview.md) is the supported
path on both Jetson and amd64, and the setup program's `ci` profile exists
precisely for unattended, non-interactive environments:

```bash
./setup.sh --run --profile ci --yes
```

Be aware that `ci` deliberately omits Autoware itself, so it produces a machine
that can fetch and lint but not build the workspace. A container that needs to
build would want `--profile dev`.

## If you want to revive it

The shape of the fix is known:

1. Choose a JetPack 6.2 base image with a matching TensorRT.
2. Replace the deleted `setup-dev-env.sh` call with
   `./setup.sh --run --profile dev --yes`.
3. Decide what to do about the Autoware Debian download (2–3 GB) — baking it
   into a layer makes the image very large, fetching it at run time makes the
   container useless offline.
4. TensorRT engines cannot be baked in at all: they are tied to the TensorRT
   version **and** the specific GPU, so they must be built on the target board
   after the container starts.

Point 4 is the one that makes a truly self-contained AutoSDV image impossible
rather than merely large.
