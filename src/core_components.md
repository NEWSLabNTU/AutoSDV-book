# Hardware Components and Wiring

## Hardware Components

The vehicle is assembled using core components and optional supplementary components. The core components include the chassis and other essential parts. Supplementary components, such as the LiDAR and 5G/LTE module, are optional and can be selected based on your specific requirements.

### Core Components

| Items                                                              |
|--------------------------------------------------------------------|
| **# Chassis**                                                      |
| Tekno TKR9500 Truck Kit 16×11×5 inch                               |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# Powertrain**                                                   |
| Brushless Motor4274 / 1500kv                                       |
| PCA9685 PWM Driver                                                 |
| DC-DC Converter IN 24V OUT 12V10A                                  |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# Computer**                                                     |
| NVIDIA Jetson AGX ORIN Dev. kit 64GB / 32GB                        |
| Micron P3 PLUS 1000GB SSD                                          |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# Camera**                                                       |
| ZED X Mini Stereo Camera (Lens Focal Length 2.2mm, with polarizer) |
| ZED Link Capture Card                                              |
| GMSL2 Fakra Cable F-F(0.3m)                                        |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# Navigation Sensors**                                           |
| KY-003 Hall Effect Sensor                                          |
| MPU9250 9-axis Motion Processing Unit                              |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# Battery and Power Supply**                                     |
| Battery Gens ACE-5000mAh-45C-22.2V-6S1P-XT60                       |
| Battery Gens ACE-7200mAh-50C-7.4V-2S1P-21                          |
| Breaker 4230-T110-K0BE-8AB8                                        |

*Table 1. Core Materials for the vehicle.*

### Supplementary: LiDAR Sensors

When choosing a LiDAR sensor, it depends on the specific localization method and desired vision quality. If point cloud-based NDT localization is used, the Velodyne VLP-32C LiDAR is often selected for its panoramic view. In contrast, solid-state LiDARs offer higher point density, making them better suited for detailed scene and object feature extraction, as well as vision-based localization that collaborates with cameras.

| LiDAR Sensor          |                                           |
|-----------------------|-------------------------------------------|
| (Choose one of below) |                                           |
| Seyond Robin-W        | Solid-State LiDAR with 120° FOV           |
| Blickfeld Cube 1      | Solid-State LiDAR with 70° FOV. (EOL)     |
| Velodyne VLP-32C      | Mechanical spinning LiDAR with 360° (EOL) |

*Table 2. Recommended LiDAR sensor for the vehicle.*

### Supplementary: 5G/LTE Communication

The Ataya 5G Harmony kit was successfully deployed on the vehicle and underwent examination by NEWSLab at National Taiwan University. The following table lists the key components of the 5G kit. For more detailed specifications and quotes, please visit [Ataya's website](https://www.ataya.io/). Additionally, consulting the Global Mobile Frequencies Database at [Spectrum Monitoring](https://www.spectrummonitoring.com/frequencies.php/) to know available bands in your region.

| 5G/LTE Kit                        |                                                                |
|-----------------------------------|----------------------------------------------------------------|
| **# 5G/LTE**                      |                                                                |
| Ataya Harmony 5G Core Network Kit | Included within a 28-inch suitcase containing the core router. |
| Askey 5G Sub-6 Indoor Small Cell  | The base station connected to the core network.                |
| MOXA CCG-1500 Gateway             | Installed on the vehicle as the connector to 5G.               |

*Table 3. Recommended LiDAR sensor for the vehicle.*


## Procurement Information

The vehicle can be ordered through Hennes Co., including customizable options for additional parts. Note that batteries are excluded from assembly due to shipping constraints and should be sourced locally. You can request a quote via their [Robot Kingdom](https://robotkingdom.com.tw/contact/) website.

The optional supplementary parts such as LiDARs and 5G modules are up to your specific needs. It is advised to consult with your local agent for procurement assistance.

