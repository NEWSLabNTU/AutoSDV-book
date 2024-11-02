# Vehicle Setup



## GPIO/I2C Device Pinout

### AGX Orin Expansion Header Pinout

The detailed pinout specification can be found on this
[website](https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/).


| Device  | Label                                   | Pin No. (Upper) | Pin No. (Lower) | Label                                   | Device  |
|---------|-----------------------------------------|-----------------|-----------------|-----------------------------------------|---------|
|         | 3.3 VDC Power, 1A max                   | 1               | 2               | 5.0 VDC Power, 1A max                   | KY-003  |
| PCA9685 | I2C5_DAT General I2C5 Data I2C Bus 7    | 3               | 4               | 5.0 VDC Power, 1A max                   | PCA9685 |
| PCA9685 | I2C5_CLK General I2C #5 Clock I2C Bus 7 | 5               | 6               | GND                                     | PCA9685 |
|         | MCLK05 Audio Master Clock               | 7               | 8               | UART1_TX UART #1 Transmit               |         |
|         | GND                                     | 9               | 10              | UART1_RX UART #1 Receive                |         |
|         | UART1_RTS UART #1 Request to Send       | 11              | 12              | I2S2_CLK Audio I2S #2 Clock             |         |
|         | GPIO32 GPIO #32                         | 13              | 14              | GND                                     | KY-003  |
| KY-003  | GPIO27 (PWM)                            | 15              | 16              | GPIO8                                   |         |
| MPU9250 | 3.3 VDC Power, 1A max                   | 17              | 18              | GPIO35 (PWM)                            |         |
|         | SPI1_MOSI SPI #1 Master Out/Slave In    | 19              | 20              | GND                                     | MPU9250 |
|         | SPI1_MISO SPI #1 Master In/Slave Out    | 21              | 22              | GPIO17 GPIO                             |         |
|         | SPI1_SCK SPI #1 Shift Clock             | 23              | 24              | SPI1_CS0_N SPI #1 Chip Select #0        |         |
|         | GND                                     | 25              | 26              | SPI1_CS1_N SPI #1 Chip Select #1        |         |
| MPU9250 | I2C2_DAT General I2C #2 Data I2C Bus 1  | 27              | 28              | I2C2_CLK General I2C #2 Clock I2C Bus 1 | MPU9250 |
|         | CAN0_DIN CAN #0 Data In                 | 29              | 30              | GND                                     |         |
|         | CAN0_DOUT CAN #0 Data Out               | 31              | 32              | GPIO9                                   |         |
|         | CAN1_DOUT CAN #1 Data Out               | 33              | 34              | GND                                     |         |
|         | I2S_FS AUDIO I2S #2 Left/Right Clock    | 35              | 36              | UART1_CTS UART #1 Clear to Send         |         |
|         | CAN1_DIN CAN #1 Data In                 | 37              | 38              | I2S_SDIN Audio I2S #2 Data In           |         |
|         | GND                                     | 39              | 40              | I2S_SDOUT Audio I2S #2 Data Out         |         |

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

TODO

## ZED X Mini Camera

TODO
