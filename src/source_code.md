# Source Code Walkthrough

The AutoSDV follows the
[_superproject_](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects)
convention. It collects hundreds of packages as Git submodules
classified by their functions into directories. It is built atop of
Autoware plus additional pacakges specific to the project and
consitutes a large ROS workspace.

Here you can visit the GitHub repository:
[https://github.com/NEWSLabNTU/AutoSDV](https://github.com/NEWSLabNTU/AutoSDV).


| Directory       | Function                                                                       |
|-----------------|--------------------------------------------------------------------------------|
| `AutoSDV/`      |                                                                                |
| `├── book/`     | The source documents for this book.                                            |
| `├── data/`     | It includes data files used or loaded in the runtime.                          |
| `├── docker/`   | The Docker container build script.                                             |
| `├── scripts/`  | Auxiliary script files. It contanis Ansible scripts to set up the environment. |
| `├── src/`      | The source code packages.                                                      |
| `├── Makefile`  | It includes commonly used receipts.                                            |
| `└── README.md` | The introductory document to git the first impression to the project.          |


## Source Package Categories

The whole AutoSDV driving system have 400+ packages by the time this
article was written. It's important to understand the way these
packages are classified.

| Directory               | Function                                                |
|-------------------------|---------------------------------------------------------|
| `AutoSDV/src/`          | The entry to the Autoware source tree.                  |
| `├── core/`             | The Autoware.Core library.                              |
| `├── launcher/`         | Includes launch files to run the entire driving system. |
| `├── vehicle/`          | Powertrain control and kinetic parameters.              |
| `├── param/`            | Parameters specific to vehicle models.                  |
| `├── sensor_component/` | Sensor drivers and sensing data processors.             |
| `└── sensor_kit/`       | Sensor related parameters and launch files.             |



## Vehicle Interface Packages

`AutoSDV/src/vehicle/autosdv_vehicle_launch/`

| Directory                          | Function                                      |
|------------------------------------|-----------------------------------------------|
| `.../autosdv_vehicle_launch/`      |                                               |
| `├── autosdv_vehicle_interface/`   | Powertrain control and its state measurement. |
| `├── autosdv_vehicle_description/` | Vehicle shape parameters and mesh files.      |
| `└── autosdv_vehicle_launch/`      | Launch files to start the vehicle interface.  |


## The Package for Vehicle-Specific Parameters

The `autoware_individual_params` package serves parameters that are
specific to different vehicle models. It is located at

```
AutoSDV/src/param/autoware_individual_params
```

You can find the parameter directories within this package.

```
.../autoware_individual_params/individual_params/default/
```

| Directory                             | Function                            |
|---------------------------------------|-------------------------------------|
| `.../default/`                        |                                     |
| `├── awsim_sensor_kit`                | Parameters for the AWSIM vehicle.   |
| `└── autosdv_sensor_kit`              | Parameters for the AutoSDV vehicle. |
| `    ├── imu_corrector.param.yaml`    |                                     |
| `    ├── sensor_kit_calibration.yaml` |                                     |
| `    └── sensors_calibration.yaml`    |                                     |


## Sensor Related Packages

| Directory                                     | Function                          |
|-----------------------------------------------|-----------------------------------|
| `AutoSDV/src/`                                |                                   |
| `├── sensor_component/`                       | Sensor drivers and preprocessors. |
| `└── sensor_kit/`                             |                                   |
| `    └── autosdv_sensor_kit_launch/`          |                                   |
| `        ├── autosdv_sensor_kit_description/` | Coordinates of each sensor.       |
| `        └── autosdv_sensor_kit_launch/`      | Another launch files for sensors. |

