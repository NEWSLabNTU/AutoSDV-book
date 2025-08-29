# Technical Reference

This section provides detailed technical specifications, wiring diagrams, and component details for the AutoSDV platform. This information supplements the model-specific guides with in-depth technical documentation.

## Contents

### Core Components
- [Core Components Specification](./core_components.md) - Detailed specifications for all base platform components

### Advanced Topics
- [5G/LTE Deployment](./5g_deployment.md) - Detailed guide for cellular connectivity setup
- [Sensor Components and Drivers](./sensor_component.md) - Low-level sensor integration
- [Sensor Kit Configuration](./sensor_kit.md) - ROS 2 sensor configuration details
- [Vehicle Interface](./vehicle_interface.md) - CAN bus and actuator control

## Quick Reference Tables

### Power Requirements
| Component | Voltage | Current (Typical) | Current (Peak) |
|-----------|---------|------------------|----------------|
| Jetson AGX Orin | 9-20V | 2.5A @ 12V | 4A @ 12V |
| Velodyne VLP-32C | 9-32V | 1A @ 12V | 1.5A @ 12V |
| Robin-W LiDAR | 12-24V | 0.7A @ 12V | 1A @ 12V |
| Cube1 LiDAR | 12-24V | 0.7A @ 12V | 1A @ 12V |
| ZED X Mini | 12V | 0.5A | 0.8A |
| 5G Module | 12V | 1.2A | 2A |

### Communication Interfaces
| Interface | Purpose | Protocol | Bandwidth |
|-----------|---------|----------|-----------|
| Ethernet (GbE) | LiDAR data | UDP | 1 Gbps |
| USB 3.0 | Camera data | USB | 5 Gbps |
| CAN Bus | Vehicle control | CAN 2.0B | 1 Mbps |
| I2C | Sensors | I2C | 400 kbps |
| UART | GPS/IMU | Serial | 115200 baud |

### Environmental Specifications
| Parameter | Operating Range | Storage Range |
|-----------|----------------|---------------|
| Temperature | -10°C to +50°C | -20°C to +60°C |
| Humidity | 10% to 90% RH | 5% to 95% RH |
| Vibration | 2G RMS | 5G RMS |
| Shock | 15G peak | 30G peak |
| Ingress Protection | IP54 (with enclosure) | - |

## System Architecture

### Data Flow
```
Sensors → Processing → Decision → Control → Actuators
   ↓          ↓           ↓         ↓          ↓
LiDAR    Perception   Planning   Commands   Motors
Camera   Localization  Safety    Validation  Steering
IMU/GPS  Fusion        Behavior  Monitoring  Brakes
```

### Software Stack
```
Application Layer:     User Applications
     ↓
Autoware Layer:       Perception, Planning, Control
     ↓
ROS 2 Middleware:     DDS Communication
     ↓
Driver Layer:         Sensor/Actuator Drivers
     ↓
OS Layer:            Ubuntu 22.04 + RT Kernel
     ↓
Hardware Layer:       Jetson AGX Orin
```

## Maintenance Schedule

### Daily Checks
- Visual inspection of components
- Battery voltage check
- Sensor lens cleaning

### Weekly Maintenance
- Connector inspection
- Wheel bearing check
- Software updates

### Monthly Service
- Full system diagnostic
- Calibration verification
- Performance benchmarking

### Annual Overhaul
- Complete disassembly and inspection
- Bearing replacement (if mechanical LiDAR)
- Sensor recalibration
- Structural integrity check

## Troubleshooting Quick Reference

### Common Issues
| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| No LiDAR data | Network config | Check IP settings |
| Poor localization | Map mismatch | Regenerate map |
| Erratic motion | IMU calibration | Recalibrate IMU |
| Short battery life | High load | Check CPU usage |
| Communication loss | Interference | Check wireless channels |

## Safety Specifications

### Emergency Stop
- Hardware E-stop button
- Software emergency brake
- Remote kill switch (5G model)
- Automatic fault detection

### Operating Limits
- Maximum speed: 15 km/h
- Maximum payload: 5 kg
- Maximum incline: 15°
- Minimum lighting: 10 lux (with lights)

## Compliance & Certification

### Standards
- ROS 2: REP-2000 compliant
- Safety: ISO 26262 ASIL-B capable
- EMC: FCC Part 15 Class B
- Environmental: MIL-STD-810G tested

### Documentation
- Full schematics available
- Open-source software
- Hardware design files
- Test reports

## Additional Resources

- [Hardware Assembly Guide](./hardware_assembly.md) - Step-by-step assembly
- [Software Installation](./software_installation.md) - Software setup
- [Development Guide](./development_guide.md) - Custom development
