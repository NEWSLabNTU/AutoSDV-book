# Development Guide

For developers extending, modifying or contributing to AutoSDV.

## Working on the code

- **[Source Code Walkthrough](./source-code.md)** — repository structure and how
  the packages relate
- **[Version Control](./version-control.md)** — the git superproject and its
  submodules, which is most of this workspace

## Configuring the system

- **[Presets](./presets.md)** — how `perception_preset` and
  `localization_preset` group arguments, and how to add one
- **[Localization Methods](./localization-methods.md)** — what `pose_source`
  selects between, and what each option requires
- **[Maps](./maps.md)** — which map artefact each method consumes, and how to
  build and validate one
- **[The CUDA Point Cloud Pipeline](./cuda-pipeline.md)** — GPU preprocessing,
  and the single-container constraint that governs it

## Sensors and vehicle

- **[Sensor Integration](./sensor-integration/using-sensors.md)** — the sensors
  available, their drivers and configuration, and how to add one
- **[Vehicle Control](./vehicle-control/overview.md)** — the control system, its
  hardware, and tuning
- **[Vehicle Interface](../reference/software/vehicle-interface.md)** — the
  bridge from Autoware control commands to actuators

## Before you change anything

Two habits from the repository's own guidance are worth repeating, because both
prevent work that has to be redone.

**Read configuration out of git, not out of a submodule working tree.** A
working tree can be ahead of, behind, or unrelated to the commit the
superproject actually pins, so a conclusion drawn from it may be about code no
one else has.

**Push the submodule before the pin.** A superproject pin is a commit hash, and
a hash that exists only in your local checkout is a pin nobody can resolve —
their `git submodule update` fails, and CI fails with it. See
[Version Control](./version-control.md).
