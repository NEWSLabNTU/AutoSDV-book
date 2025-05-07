# Build the Vehicle

The recommended vehicle build bases on 16×11×5 inch chassis plus
additional sensor and communication mounts, which can be divided into
several parts listed below.

The **core components** includes necessary components to run the
Autoware.

- Onboard computer
- Navigation system
- Power supply system
- Powertrain system
- Chassis

The vehicle can be equipped with **additional mounts** depending on
your choice.

- LiDAR sensors
- 5G/LTE communication module


## Core Components

The vehicle has three major layers shown in the Figure 1 from top to
bottom.

- <span style="color: Yellow">Yellow</span>: The onboard computer,
  navigation sensors and additional mounts.
- <span style="color: Red">Red</span>: Power supply system for the
  onboard computer and sensors in the yellow layer.
- <span style="color: DodgerBlue">Blue</span>: Powertrain system and
  power supply for the powertrain.

By using this vehicle, additional sensor and 5G mounts go to the <span
style="color: Yellow">yellow</span> layer, which power supply comes
from the <span style="color: Red">red</span> layer. The motors have an
separated battery and power supply in the <span style="color:
DodgerBlue">blue</span> layer due to distinct voltage requirement.

<figure style="width: 80%; text-align: center;">
	<img src="./figures/vehicle_parts.jpg" alt="Vehicle parts.">
	<figcaption>Figure 1. Vehicle components from top to bottom: onboard computer, power supply and powertrain.</figcaption>
</figure>

### Power Supply System

#### Batteries

There are two batteries for respective two power supplies, namely the
*upper power* and *lower power*. The batteries are shown in Figure 2.
The upper power provides electricity to the on-board computer and
sensors from 22.2V 6S battery (1), while the lower power provides the
electricity to the DC motor and powertrain from a 7.4V 2S battery (2).

Both batteries have a yellow XT60 power plug and a white JST-XT
connector as shown in Figure 3. The JST-XT connector is plugged to a
voltage monitor in Figure 4. It beeps when the voltage becomes low.

<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="./figures/lipo_batteries.webp" alt="Lithium polymer battery examples.">
	<figcaption>Figure 2. Lithium polymer battery examples.</figcaption>
</figure>

<figure style="width: 50%; text-align: center; margin: auto;">
	<img src="./figures/battery_connectors.webp" alt="Battery connectors.">
	<figcaption>Figure 3. Battery XT60 and JST-XH connector.</figcaption>
</figure>

<figure style="width: 40%; text-align: center; margin: auto;">
	<img src="./figures/voltage_monitor.webp" alt="Battery voltage monitor.">
	<figcaption>Figure 4. Battery voltage monitor.</figcaption>
</figure>

#### The Upper Power Supply

The upper power start up process is shown in Figure 4. First, install
the battery on the battery dock. Second, connect battery to the cable.
Last, switch on the power supply demonstrated in Figure 5.

Please be cautious that the power switch must be turned off before
installing or removing the battery. It's necessary to protect the
system from voltage spikes.

<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="./figures/battery_installation_steps.jpg" alt="Upper power start up procedure.">
	<figcaption>Figure 4. Upper power start up procedure.</figcaption>
</figure>


<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="./figures/upper_power_switch_with_arrow.jpg" alt="Turn on power switch">
	<figcaption>Figure 5. Turn on upper power switch.</figcaption>
</figure>

#### The Lower Power Supply

The lower power start up process is shown in Figure 6. The battery is
installed in the dock in the bottom layer of the vehicle (1). Then,
switch on the power (2).

<figure style="width: 60%; text-align: center; margin: auto;">
	<img src="./figures/lower_power_installation.webp" alt="Lower power start up procedure">
	<figcaption>Figure 6. Lower power start up procedure</figcaption>
</figure>


## Docks

The vehicle has three docks to mount your favorite sensors. The figure
below shows two kinds of builds with three docks marked on the figure:
(1) the front docker, (2) the top dock and (3) the rear dock.

<table align="center" border="0">
  <tbody>
    <tr>
      <td align="center" valign="bottom">
        <img src="figures/sensor_mounts-1.webp" alt="Sensor mounts example 1" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="figures/sensor_mounts-2.webp" alt="Sensor mounts example 2" width="80%"/>
      </td>
    </tr>
  </tbody>
</table>

The details of two builds are described in the table below.

<table align="center" border="0">
  <thead>
    <tr>
      <th align="center">No.</td>
      <th align="center">Front Dock</td>
      <th align="center">Top Dock</td>
      <th align="center">Rear Dock</td>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center" valign="middle" rowspan="2">1</td>
      <td align="center" valign="bottom">
        <img src="figures/front_dock-1.webp" alt="Front docker example 1" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="figures/top_dock-1.webp" alt="Top docker example 1" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="figures/rear_dock-1.webp" alt="Front docker example 1" width="80%"/>
      </td>
    </tr>
    <tr>
      <td align="center">Seyond Robin-W LiDAR</td>
      <td align="center">MOXA 5G Module</td>
      <td align="center">LiDAR Ethernet Adaptor</td>
    </tr>
    <tr>
      <td align="center" valign="middle" rowspan="2">2</td>
      <td align="center" valign="bottom">
        <img src="figures/front_dock-2.webp" alt="Front docker example 2" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="figures/top_dock-2.webp" alt="Top docker example 2" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="figures/rear_dock-2.webp" alt="Front docker example 2" width="80%"/>
      </td>
    </tr>
    <tr>
      <td align="center">Velodyne LiDAR Adaptor</td>
      <td align="center">Velodyne 32C LiDAR</td>
      <td align="center">Navigation Sensor Kit</td>
    </tr>
  </tbody>
</table>

## Components and Wiring

The vehicle incorporates essential components such as the chassis,
body, onboard computer, among others, along with additional LiDARs and
a 5G communication module. For detailed information on these elements
and their wiring, please refer to the comprehensive guide in [Hardware
and Wiring](hardware_and_wiring.md).
