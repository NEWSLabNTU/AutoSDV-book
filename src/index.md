<figure style="text-align: center">
	<img src="figures/logo/logo_brand_gray.png" alt="The AutoSDV Project logo">
</figure>

# AutoSDV Documentation

Welcome to the AutoSDV (Autonomous Software-Defined Vehicle) documentation.

The AutoSDV project, namely the *Autoware Software-Defined Vehicle*, features an affordable autonomous driving platform with practical vehicle equipment for educational and research institutes. This project allows you to build a self-driving platform at home and use it in real outdoor road environments. Driven by Autoware, the leading open-source software project for autonomous driving, it gives you great flexibility and extensibility with the vehicle software.

AutoSDV provides a complete stack from hardware specifications to software implementation, offering an accessible entry point into real-world autonomous systems using industry-standard tools and practices.

<table align="center" border="0">
  <tr>
    <td align="center" valign="bottom">
      <a href="figures/model_robin-w.webp" target="_blank">
        <img src="figures/model_robin-w.webp" alt="Robin-W Solid-State LiDAR Kit" width="80%"/>
      </a>
    </td>
    <td align="center" valign="bottom">
      <a href="figures/model_velodyne_32c.webp" target="_blank">
        <img src="figures/model_velodyne_32c.webp" alt="Velodyne 32C LiDAR Kit" width="80%"/>
      </a>
    </td>
    <td align="center" valign="bottom">
      <a href="figures/model_cube1_moxa-5g.webp" target="_blank">
        <img src="figures/model_cube1_moxa-5g.webp" alt="Blickfeld Cube1 + MOXA 5G Kit" width="80%"/>
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <b>Robin-W Solid-State LiDAR Kit</b>
    </td>
    <td align="center">
      <b>Velodyne 32C LiDAR Kit</b>
    </td>
    <td align="center">
      <b>Cube1 LiDAR + MOXA 5G Kit</b>
    </td>
  </tr>
</table>

## Key Features

- **ROS 2 Humble** - Built on the latest long-term support ROS 2 distribution
- **Autoware Integration** - Full autonomous driving stack
- **Flexible Sensor Support** - Multiple LiDAR, camera, IMU, and GNSS configurations
- **Multi-Mode Control** - Sophisticated vehicle control with PID and feedback
- **Production Ready** - Tested on real hardware platforms

## Getting Started

This guide walks you through setting up and using the AutoSDV platform. Follow these steps sequentially to build a fully functional autonomous vehicle:

1. **[Hardware Setup](getting-started/hardware-assembly.md)** - Setting up and using the vehicle hardware
2. **[Software Installation](getting-started/installation/overview.md)** - Installing AutoSDV software
   - **[ZED SDK Installation](getting-started/installation/zed-sdk.md)** - ZED camera driver setup
   - **[Automatic Setup](getting-started/installation/overview.md)** - Automated installation (recommended)
   - **[Manual Setup](getting-started/installation/manual-environment.md)** - Advanced customization
   - **[Docker Setup](getting-started/installation/docker.md)** - Containerized installation
3. **[Operating the Vehicle](getting-started/usage.md)** - Launching and controlling the system

### Quick Start Paths

**For Vehicle Deployment**: Follow the steps in order: Hardware Setup → Install → Operate

**For Simulation/Development**: Skip to [Software Installation](getting-started/installation/overview.md) or use [Docker Setup](getting-started/installation/docker.md) for quick testing

**For Customization**: See [Manual Setup](getting-started/installation/manual-environment.md) for advanced configuration options

## Quick Links

- [**Platform Models**](platform-models.md) - Explore different hardware configurations
- [**Guides**](guides/development.md) - Tutorials for developers and operators
- [**Technical Reference**](reference/overview.md) - Detailed technical specifications

## Citation

If you use AutoSDV in your research or educational projects, please cite our work using the following BibTeX entry:

```latex
@misc{autosdv2025,
  author = {Hsiang-Jui Lin, Chi-Sheng Shih},
  title = {AutoSDV: A Software-Defined Vehicle Platform for Research and Education},
  year = {2025},
  institution = {National Taiwan University},
  url = {https://github.com/NEWSLabNTU/AutoSDV},
  note = {Accessed: 2025-04-28}
}
```

## Getting Help

- **Documentation**: You're reading it!
- **Issues**: [GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV-book/issues)
- **Source Code**: [GitHub Repository](https://github.com/NEWSLabNTU/AutoSDV-book)

---

*This documentation is maintained by the AutoSDV project team.*
