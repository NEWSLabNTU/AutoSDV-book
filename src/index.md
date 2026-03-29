<figure style="text-align: center">
	<img src="figures/logo/logo_brand_gray.png" alt="The AutoSDV Project logo">
</figure>

# AutoSDV Documentation

Welcome to the AutoSDV (Autonomous Software-Defined Vehicle) documentation.

The AutoSDV project, namely the *Autoware Software-Defined Vehicle*, features an affordable autonomous driving platform with practical vehicle equipment for educational and research institutes. This project allows you to build a self-driving platform at home and use it in real outdoor road environments. Driven by Autoware, the leading open-source software project for autonomous driving, it gives you great flexibility and extensibility with the vehicle software.

AutoSDV provides a complete stack from hardware specifications to software implementation, offering an accessible entry point into real-world autonomous systems using industry-standard tools and practices.

<figure style="text-align: center; margin: 1.5em auto; max-width: 640px;">
  <video autoplay loop muted playsinline style="width: 100%; border-radius: 8px;">
    <source src="figures/coss_outdoor_run_video/coss_outdoor_run.webm" type="video/webm">
  </video>
  <figcaption>Autonomous Navigation</figcaption>
</figure>

<table align="center" border="0">
  <tr>
    <td align="center" valign="middle" width="50%">
      <a href="figures/coss_park_outdoor_daytime.png" target="_blank">
        <img src="figures/coss_park_outdoor_daytime.png" alt="AutoSDV autonomous operation during daytime" style="width: 100%; border-radius: 4px;"/>
      </a>
    </td>
    <td align="center" valign="middle" width="50%">
      <a href="figures/coss_park_outdoor_night.png" target="_blank">
        <img src="figures/coss_park_outdoor_night.png" alt="AutoSDV autonomous operation at night" style="width: 100%; border-radius: 4px;"/>
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">Daytime Operation</td>
    <td align="center">Night Operation</td>
  </tr>
</table>

## Getting Started

- **[Software Installation](getting-started/installation/overview.md)** — Set up the development environment on Ubuntu or Jetson
- **[Hardware Setup](getting-started/hardware-assembly.md)** — Assemble the vehicle platform
- **[Operating the Vehicle](getting-started/usage.md)** — Launch the system, monitor, and record data

You do not need a physical vehicle to get started. AutoSDV includes a planning simulator and rosbag replay tools that run entirely in software. See the [Software Installation](getting-started/installation/overview.md) guide to begin.

## Explore

- [**Platform Models**](platform-models.md) — Hardware configurations and build variants
- [**Sensor Integration**](guides/sensor-integration/using-sensors.md) — Configure LiDAR, camera, IMU, GNSS
- [**Vehicle Control**](guides/vehicle-control/overview.md) — Motor, steering, and PID tuning
- [**Technical Reference**](reference/overview.md) — Specifications and wiring diagrams

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
- **Issues**: [GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
- **Source Code**: [GitHub Repository](https://github.com/NEWSLabNTU/AutoSDV)

---

*This documentation is maintained by the AutoSDV project team.*
