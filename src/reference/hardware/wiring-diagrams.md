# Wiring Diagrams

Detailed electrical connections and power distribution for the AutoSDV platform.

⚠️ **WARNING**: Always disconnect batteries before wiring changes

## Power Distribution Overview

### Upper Power System (22.2V Battery)
```
Main Battery (22.2V) ──── DC-DC (12V) ──┬── Jetson AGX Orin
                                        ├── ZED Link Capture Card
                                        ├── LiDAR (optional)
                                        └── 5G Module (optional)

Jetson AGX Orin GPIO ──┬── 3.3V Pin 17 ──── MPU9250 (IMU)
                       ├── 5V Pin 2 ─────── KY-003 Hall Sensors
                       └── 5V Pin 4 ─────── PCA9685 PWM Driver

ZED Link ──────────── GMSL2 Cable ───────── ZED X Mini Camera
```

### Motor Power System (7.4V Battery)
```
Motor Battery (7.4V) ──┬── ESC ──────────── Brushless Motor
                       └── PCA9685 ──────── Steering Servo
```

## NVIDIA Jetson AGX Orin Connections

### Main Interfaces

```
Jetson AGX Orin
├── Power Input
│   └── DC Jack: 12V from DC-DC Converter
│
├── Ethernet Port
│   └── LiDAR Connection (if equipped)
│
├── USB Ports
│   ├── USB Port: USB-to-Ethernet Adapter → 5G Module (if equipped)
│   └── USB Port: GPS Receiver (optional)
│
├── SAMTEC CSI Port (bottom mounted)
│   └── ZED Link Capture Card
│       ├── GMSL2 Ports → ZED X Mini Camera
│       └── DC Power Input (12V)
│
└── GPIO Header (40-pin) → See GPIO Connections section
```

### GPIO Header Connections

```
I2C Bus 7 (PCA9685 PWM Driver)
├── Pin 3: I2C5_DAT (SDA)
├── Pin 4: 5V Power
├── Pin 5: I2C5_CLK (SCL)
└── Pin 6: Ground

I2C Bus 1 (MPU9250 IMU)
├── Pin 17: 3.3V Power
├── Pin 20: Ground
├── Pin 27: I2C2_DAT (SDA)
└── Pin 28: I2C2_CLK (SCL)

GPIO (KY-003 Hall Sensor)
├── Pin 2: 5V Power
├── Pin 14: Ground
└── Pin 15: GPIO27 Signal
```

## Sensor Connections

### LiDAR Connections (Model Specific)

**Velodyne VLP-32C:**
```
VLP-32C
├── Power: 12V from DC-DC converter
├── Ethernet: Direct to Jetson
│   ├── IP: 192.168.1.201
│   └── Port: 2368 (data), 8308 (position)
└── GPS Sync: Optional PPS input
```

**Seyond Robin-W:**
```
Robin-W
├── Power: 12V from DC-DC converter
├── Ethernet: Cat6 to Jetson
│   ├── IP: 192.168.1.10
│   └── Port: 2368
└── Sync: IEEE 1588 PTP
```

**Blickfeld Cube1:**
```
Cube1
├── Power: 12V from DC-DC converter
├── Ethernet: RJ45 to Jetson
│   ├── IP: 192.168.1.21
│   └── Port: 8000
└── Status LED: RGB indicator
```

### Camera System (ZED X Mini)

```
ZED X Mini Camera
└── GMSL2 Fakra Cable ──── ZED Link Capture Card
```

*Note: Camera receives power through GMSL2 connection from ZED Link. The ZED Link is attached to the bottom of the Jetson AGX Orin via SAMTEC CSI port.*

### IMU (MPU9250)

```
MPU9250 Module (I2C Bus 1)
├── VCC ────── 3.3V (Jetson Pin 17)
├── GND ────── GND (Jetson Pin 20)
├── SDA ────── I2C2_DAT (Jetson Pin 27)
└── SCL ────── I2C2_CLK (Jetson Pin 28)
```

### Hall Sensor (KY-003)

```
KY-003 Hall Sensor
├── VCC ────── 5V (Jetson Pin 2)
├── GND ────── GND (Jetson Pin 14)
└── Signal ── GPIO27 (Jetson Pin 15)
```

### GPS Module (Optional)

```
u-blox GPS (if equipped)
├── VCC ────── 5V (USB powered)
├── GND ────── Ground
├── TX ─────── UART RX (Jetson Pin 10)
├── RX ─────── UART TX (Jetson Pin 8)
└── PPS ────── GPIO (Optional timing)
```

## Motor Control System

### PWM Driver (PCA9685)

```
PCA9685 Board (I2C Bus 7)
├── Power
│   ├── VCC: 5V (Jetson Pin 4)
│   ├── GND: Ground (Jetson Pin 6)
│   └── V+: 7.4V Motor Battery
│
├── I2C Interface
│   ├── SDA ── I2C5_DAT (Jetson Pin 3)
│   ├── SCL ── I2C5_CLK (Jetson Pin 5)
│   └── Addr: 0x40 (default)
│
└── PWM Outputs
    ├── Channel 0: Steering Servo
    ├── Channel 1: ESC Throttle
    └── Channel 2-15: Available
```

### Motor and ESC

```
Brushless Motor System
├── ESC (Electronic Speed Controller)
│   ├── Input: PWM from PCA9685 Ch1
│   ├── Power: 7.4V from battery
│   └── Output: 3-phase to motor
│
└── Motor (4274/1500kv)
    ├── Phase A,B,C from ESC
    └── Hall sensors (optional)
```

## 5G Module (Optional)

```
MOXA OnCell G4302 (or similar)
├── Power
│   ├── Input: 12V @ 2A
│   └── Ground: Common ground
│
├── Data Connection
│   └── Ethernet → USB-to-Ethernet Adapter → Jetson USB
│
└── SIM Slots
    ├── SIM1: Primary carrier
    └── SIM2: Backup carrier
```

## Battery Management

### Battery Monitoring

```
6S Battery (22.2V) - Upper Power
├── Cell 1-6 Balance Leads
└── JST-XH to Battery Monitor/Alarm

2S Battery (7.4V) - Motor Power
├── Cell 1-2 Balance Leads
└── JST-XH to Battery Monitor/Alarm
```

*Note: Each battery has its own independent monitor with built-in alarm.*

## GPIO 40-Pin Header Reference

The detailed pinout specification can be found on [JetsonHacks](https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/).

**Physical Layout Note:** Pin 1-2 are physically on the rightmost side of the header when viewing the Jetson from above. Pin 39-40 are on the leftmost side.

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
|         | Reserved                                | 29              | 30              | GND                                     |         |
|         | Reserved                                | 31              | 32              | GPIO9                                   |         |
|         | Reserved                                | 33              | 34              | GND                                     |         |
|         | I2S_FS AUDIO I2S #2 Left/Right Clock    | 35              | 36              | UART1_CTS UART #1 Clear to Send         |         |
|         | Reserved                                | 37              | 38              | I2S_SDIN Audio I2S #2 Data In           |         |
|         | GND                                     | 39              | 40              | I2S_SDOUT Audio I2S #2 Data Out         |         |


## Next Steps

- [Hardware Assembly Guide](../../getting-started/hardware-assembly.md) - Physical assembly
- [Software Installation](../../getting-started/installation/overview.md) - Software setup
- [Core Components](../hardware/core-components.md) - Component specifications
