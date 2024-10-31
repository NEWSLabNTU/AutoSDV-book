# Source Code Walkthrough

The F1EIGHTH is a
[_superproject_](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects)
hosted on GitHub. It collects hundreds of packages as Git submodules
classified by their functions into directories. It is built atop of
Autoware plus additional pacakges specific to the project and
consitutes a large ROS workspace.

Here you can visit the GitHub repository:
[https://github.com/NEWSLabNTU/F1EIGHTH](https://github.com/NEWSLabNTU/F1EIGHTH).


| Directory       | Function                                                                       |
|-----------------|--------------------------------------------------------------------------------|
| `F1EIGHTH/`     |                                                                                |
| `├── book/`     | The source documents for this book.                                            |
| `├── data/`     | It includes data files used or loaded in the runtime.                          |
| `├── docker/`   | The Docker container build script.                                             |
| `├── scripts/`  | Auxiliary script files. It contanis Ansible scripts to set up the environment. |
| `├── src/`      | The source code packages.                                                      |
| `├── Makefile`  | It includes commonly used receipts.                                            |
| `└── README.md` | The introductory document to git the first impression to the project.          |


## Source Package Categories

The whole F1EIGHTH driving system have 400+ packages by the time this
article was written. It's important to understand the way these
packages are classified.

| Directory                  | Function                                                |
|----------------------------|---------------------------------------------------------|
| `F1EIGHTH/src/calibraion/` | The entry to calibration tools.                         |
| `F1EIGHTH/src/autoware/`   | The entry to the Autoware source tree.                  |
| `├── core/`                | The Autoware.Core library.                              |
| `├── launcher/`            | Includes launch files to run the entire driving system. |
| `├── vehicle/`             | Powertrain control and kinetic parameters.              |
| `├── param/`               | Parameters specific to vehicle models.                  |
| `├── sensor_component/`    | Sensor drivers and sensing data processors.             |
| `├── sensor_kit/`          | Sensor related parameters and launch files.             |
| `└── universe/`            | The Autoware.Universe library.                          |



## Vehicle Interface Packages

`F1EIGHTH/src/autoware/vehicle/f1eighth_vehicle_launch/`

| Directory                           | Function                                      |
|-------------------------------------|-----------------------------------------------|
| `.../f1eighth_vehicle_launch/`      |                                               |
| `├── f1eighth_vehicle_interface/`   | Powertrain control and its state measurement. |
| `├── f1eighth_vehicle_description/` | Vehicle shape parameters and mesh files.      |
| `└── f1eighth_vehicle_launch/`      | Launch files to start the vehicle interface.  |


## The Package for Vehicle-Specific Parameters

The `autoware_individual_params` package serves parameters that are
specific to different vehicle models. It is located at

```
F1EIGHTH/src/autoware/param/autoware_individual_params
```

You can find the parameter directories within this package.

```
.../autoware_individual_params/individual_params/default/
```

| Directory                                                  | Function                             |
|------------------------------------------------------------|--------------------------------------|
| `.../default/` |                                      |
| `├── awsim_sensor_kit`                                     | Parameters for the AWSIM vehicle.    |
| `└── f1eighth_sensor_kit`                                  | Parameters for the F1EIGHTH vehicle. |
| `    ├── imu_corrector.param.yaml`                         |                                      |
| `    ├── sensor_kit_calibration.yaml`                      |                                      |
| `    └── sensors_calibration.yaml`                         |                                      |


## Sensor Related Packages

| Directory                                      | Function                           |
|------------------------------------------------|------------------------------------|
| `F1EIGHTH/src/autoware/`                       |                                    |
| `├── sensor_component/`                        | Sensor drivers and preprocessors.  |
| `└── sensor_kit/`                              |                                    |
| `    └── f1eighth_sensor_kit_launch/`          |                                    |
| `        ├── common_sensor_launch/`            | Launch files to drive the sensors. |
| `        ├── f1eighth_sensor_kit_description/` | Coordinates of each sensor.        |
| `        └── f1eighth_sensor_kit_launch/`      | Another launch files for sensors.  |


## Autoware Packages

Autoware packages are categorized into the _Core_ and _Universe_
divisions. The Core library brings the stable and well-tested
packages, while the Universe library brings the packages with
bleeding-edge and more advanced features from the Autoware community.

The Core library is located at:

```
F1EIGHTH/src/autoware/core/
```

| Directory                  | Function |
|----------------------------|----------|
| `.../core/`                |          |
| `├── autoware_adapi_msgs/` |          |
| `├── autoware_common/`     |          |
| `├── autoware.core/`       |          |
| `└── autoware_msgs/`       |          |


The Universe library is located at:

```
F1EIGHTH/src/autoware/universe/autoware.universe/
```

| Directory                | Function |
|--------------------------|----------|
| `.../autoware.universe/` |          |
| `├── common/`            |          |
| `├── control/`           |          |
| `├── evaluator/`         |          |
| `├── launch/`            |          |
| `├── localization/`      |          |
| `├── map/`               |          |
| `├── perception/`        |          |
| `├── planning/`          |          |
| `├── sensing/`           |          |
| `├── simulator/`         |          |
| `├── system/`            |          |
| `└── vehicle/`           |          |

