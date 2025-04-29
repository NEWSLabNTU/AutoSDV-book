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


## Sensor Docks

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

## Materials

| Items                                                              |
|--------------------------------------------------------------------|
| **Chassis**                                                        |
| Tekno TKR9500 Truck Kit (16×11×5 inch)                             |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **Powertrain**                                                     |
| Brushless Motor4274 / 1500kv                                       |
| PCA9685 PWM Driver                                                 |
| DC-DC Converter IN 24V OUT 12V10A                                  |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **Computer**                                                       |
| NVIDIA Jetson AGX ORIN Dev. kit 64GB                               |
| Micron P3 PLUS 1000GB SSD                                          |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **Camera**                                                         |
| ZED X Mini Stereo Camera (Lens Focal Length 2.2mm, with polarizer) |
| ZED Link Capture Card                                              |
| GMSL2 Fakra Cable F-F(0.3m)                                        |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **LiDAR**                                                          |
| Blickfeld Cube1                                                    |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **Battery and Power Supply**                                       |
| Battery Gens ACE-5000mAh-45C-22.2V-6S1P-XT60                       |
| Battery Gens ACE-7200mAh-50C-7.4V-2S1P-21                          |
| Breaker 4230-T110-K0BE-8AB8                                        |

*Table 1. Materials of the vehicle.*

## GPIO/I2C Device Pinout

### AGX Orin Expansion Header Pinout

The detailed pinout specification can be found on this
[website](https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/).


| Device  | Label                                    | Pin No. (Upper) | Pin No. (Lower) | Label                                    | Device  |
|---------|------------------------------------------|-----------------|-----------------|------------------------------------------|---------|
|         | 3.3 VDC Power, 1A max                    | 1               | 2               | 5.0 VDC Power, 1A max                    | KY-003  |
| PCA9685 | I2C5\_DAT General I2C5 Data I2C Bus 7    | 3               | 4               | 5.0 VDC Power, 1A max                    | PCA9685 |
| PCA9685 | I2C5\_CLK General I2C #5 Clock I2C Bus 7 | 5               | 6               | GND                                      | PCA9685 |
|         | MCLK05 Audio Master Clock                | 7               | 8               | UART1\_TX UART #1 Transmit               |         |
|         | GND                                      | 9               | 10              | UART1\_RX UART #1 Receive                |         |
|         | UART1\_RTS UART #1 Request to Send       | 11              | 12              | I2S2\_CLK Audio I2S #2 Clock             |         |
|         | GPIO32 GPIO #32                          | 13              | 14              | GND                                      | KY-003  |
| KY-003  | GPIO27 (PWM)                             | 15              | 16              | GPIO8                                    |         |
| MPU9250 | 3.3 VDC Power, 1A max                    | 17              | 18              | GPIO35 (PWM)                             |         |
|         | SPI1\_MOSI SPI #1 Master Out/Slave In    | 19              | 20              | GND                                      | MPU9250 |
|         | SPI1\_MISO SPI #1 Master In/Slave Out    | 21              | 22              | GPIO17 GPIO                              |         |
|         | SPI1\_SCK SPI #1 Shift Clock             | 23              | 24              | SPI1\_CS0\_N SPI #1 Chip Select #0       |         |
|         | GND                                      | 25              | 26              | SPI1\_CS1\_N SPI #1 Chip Select #1       |         |
| MPU9250 | I2C2\_DAT General I2C #2 Data I2C Bus 1  | 27              | 28              | I2C2\_CLK General I2C #2 Clock I2C Bus 1 | MPU9250 |
|         | CAN0\_DIN CAN #0 Data In                 | 29              | 30              | GND                                      |         |
|         | CAN0\_DOUT CAN #0 Data Out               | 31              | 32              | GPIO9                                    |         |
|         | CAN1\_DOUT CAN #1 Data Out               | 33              | 34              | GND                                      |         |
|         | I2S\_FS AUDIO I2S #2 Left/Right Clock    | 35              | 36              | UART1\_CTS UART #1 Clear to Send         |         |
|         | CAN1\_DIN CAN #1 Data In                 | 37              | 38              | I2S\_SDIN Audio I2S #2 Data In           |         |
|         | GND                                      | 39              | 40              | I2S\_SDOUT Audio I2S #2 Data Out         |         |

*Table 2. Pinout for NVIDIA AGX Orin box.*


### I2C Device Pinout

The wires to a I2C device consists of a pair of VCC/GND for power
supply and a pair of SDA/SCL for data transmission. The pair of
SDA/SCL associates with a I2C bus number.

| Device                            | Bus No. | VCC | GND | SDA | SCL |
|-----------------------------------|---------|-----|-----|-----|-----|
| PCA9685 PWM/servo driver          | 7       | 4   | 6   | 3   | 5   |
| MPU9250 inertial measurement unit | 1       | 17  | 20  | 27  | 28  |

*Table 3. I2C device ports and connected pin numbers.*

### GPIO Device Pinout

The wires to a GPIO device consists of a pair of VCC/GND for power
supply and a GPIO wire for input or output signals.


| Device                                       | VCC | GND | GPIO |
|----------------------------------------------|-----|-----|------|
| KY-003 Hall effect sensor                    | 2   | 14  | 15   |

*Table 4. GPIO device ports and connected pin numbers.*



## Blickfeld Cube1 LiDAR

The LiDAR is connected through an Ethernet cable to Orin. A static IP
is configured on the network.

## ZED X Mini Camera

The camera is connected through a GMSL2 FAKRA interface to a ZED Box
on Orin.
