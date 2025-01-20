# Build the Vehicle

The recommended vehicle setup is 16×11×5 inch


## Materials

| Items                                                              |
|--------------------------------------------------------------------|
| **Chassis**                                                        |
| Tekno TKR9500 Truck Kit (16×11×5 inch)                             |
|                                                                    |
| **Powertrain**                                                     |
| Brushless Motor4274 / 1500kv                                       |
| PCA9685 PWM Driver                                                 |
| DC-DC Converter IN 24V OUT 12V10A                                  |
|                                                                    |
| **Computer**                                                       |
| NVIDIA Jetson AGX ORIN Dev. kit 64GB                               |
| Micron P3 PLUS 1000GB SSD                                          |
|                                                                    |
| **Camera**                                                         |
| ZED X Mini Stereo Camera (Lens Focal Length 2.2mm, with polarizer) |
| ZED Link Capture Card                                              |
| GMSL2 Fakra Cable F-F(0.3m)                                        |
|                                                                    |
| **LiDAR**                                                          |
| Blickfeld Cube1                                                    |
|                                                                    |
| **Battery and Power Supply**                                       |
| Battery Gens ACE-5000mAh-45C-22.2V-6S1P-XT60                       |
| Battery Gens ACE-7200mAh-50C-7.4V-2S1P-21                          |
| Breaker 4230-T110-K0BE-8AB8                                        |



## Battery and Power Supply

The vehicle has separate power supply for motors and on-board
computers and sensors. The purpose is to serve higher voltage on the
sensors and current protection.

![](./figures/vehicle_parts.jpg)

*Yellow: The onboard computer and sensors are connected to 25V
power. Red: Battery and power supply. Blue: Chassis and powertrain.*


## 5G and WiFi Mount

![](./figures/vehicle_top_mount.jpg)

*Blue: The vehicle mount on the top. Red: The 5G antenna.*

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

*Table: Pinout for AGX Orin.*


### I2C Device Pinout

The wires to a I2C device consists of a pair of VCC/GND for power
supply and a pair of SDA/SCL for data transmission. The pair of
SDA/SCL associates with a I2C bus number.

| Device                            | Bus No. | VCC | GND | SDA | SCL |
|-----------------------------------|---------|-----|-----|-----|-----|
| PCA9685 PWM/servo driver          | 7       | 4   | 6   | 3   | 5   |
| MPU9250 inertial measurement unit | 1       | 17  | 20  | 27  | 28  |

*Table: I2C device ports and connected pin numbers.*

### GPIO Device Pinout

The wires to a GPIO device consists of a pair of VCC/GND for power
supply and a GPIO wire for input or output signals.


| Device                                       | VCC | GND | GPIO |
|----------------------------------------------|-----|-----|------|
| KY-003 Hall effect sensor                    | 2   | 14  | 15   |

*Table: GPIO device ports and connected pin numbers.*



## Blickfeld Cube1 LiDAR

The LiDAR is connected through an Ethernet cable to Orin. A static IP
is configured on the network.

## ZED X Mini Camera

The camera is connected through a GMSL2 FAKRA interface to a ZED Box
on Orin.
