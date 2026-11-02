"""The signal and data overview: every part, every link that carries data.

Power wiring is wiring_power.py; the Orin header, pin by pin, is
wiring_orin_header.py.
"""

from __future__ import annotations

from wiring_schematic import Device, Wire, legend, render

ETH, USB, CAM, I2C, SIG, MOTOR = "#1f77b4", "#17becf", "#9467bd", "#2ca02c", "#ff7f0e", "#7f7f7f"

DEVICES = [
    Device("zed", "zed-x-mini", 60, 60, "ZED X Mini", "stereo camera + IMU"),
    Device("zedlink", "zed-link", 320, 60, "ZED Link capture card"),
    Device("lidar", "lidar", 320, 300, "LiDAR", "VLP-32C, Robin-W or Cube1"),
    Device("g5", "moxa-5g", 320, 540, "5G router (optional)", "MOXA OnCell G4302"),
    Device("gps", "gps", 320, 780, "GNSS receiver (optional)"),
    Device("orin", "orin", 640, 380, "Jetson AGX Orin", "", bold=True),
    Device("hall", "ky003", 1000, 60, "KY-003 hall sensor", "rear wheel, 12 magnets"),
    Device("pca", "pca9685", 1000, 540, "PCA9685 PWM driver", "I2C bus 7, address 0x40"),
    Device("rx", "receiver", 1300, 60, "RC receiver", "paired with the joystick"),
    Device("sel", "selector-switch", 1300, 540, "selector switch", "Orin or joystick"),
    Device("esc", "esc", 1600, 300, "motor ESC", "Dynamite Fuze brushless"),
    Device("motor", "motor", 1900, 300, "brushless motor", "4274, 1500 kV"),
    Device("servo", "servo", 1600, 780, "steering servo"),
]

WIRES = [
    # Camera chain
    Wire(("zed", "right", 0.4), ("zedlink", "left", 0.4), CAM, "GMSL2", arrow=False),
    Wire(("zedlink", "right", 0.4), ("orin", "left", 0.2), CAM, "Samtec CSI", via=[(560, None)],
         arrow=False, label_on=1),
    # Ethernet and USB
    Wire(("lidar", "right", 0.4), ("orin", "left", 0.4), ETH, "Ethernet", via=[(580, None)],
         arrow=False),
    Wire(("g5", "right", 0.4), ("orin", "left", 0.6), USB, "Ethernet via USB adapter",
         via=[(580, None)], dashed=True, arrow=False, label_on=1),
    Wire(("gps", "right", 0.4), ("orin", "left", 0.8), USB, "USB", via=[(560, None)],
         dashed=True, arrow=False, label_on=1),
    # The 40-pin header
    Wire(("hall", "bottom", 0.5), ("orin", "top", 0.6), SIG, "GPIO, header pin 16",
         via=[(None, 300)]),
    Wire(("orin", "right", 0.75), ("pca", "left", 0.4), I2C, "I2C bus 7", via=[(940, None)]),
    # The drive chain: the switch picks the Orin's pulses or the joystick's
    Wire(("pca", "right", 0.4), ("sel", "left", 0.4), SIG, "ch0 throttle, ch1 steer"),
    Wire(("rx", "bottom", 0.5), ("sel", "top", 0.5), SIG, "throttle, steer"),
    Wire(("sel", "right", 0.3), ("esc", "left", 0.5), SIG, "throttle pulse",
         via=[(1540, None)]),
    Wire(("sel", "right", 0.7), ("servo", "left", 0.4), SIG, "steer pulse",
         via=[(1560, None)]),
    Wire(("esc", "right", 0.4), ("motor", "left", 0.4), MOTOR, "3 phases"),
]

LEGEND = legend(60, 300, "Connections", [
    ("Ethernet", ETH, False), ("USB", USB, False), ("camera (CSI, GMSL2)", CAM, False),
    ("I2C", I2C, False), ("PWM, GPIO, UART", SIG, False), ("motor phases", MOTOR, False),
    ("dashed: optional", "#9e9e9e", True),
])


def render_svg() -> str:
    return render(2140, 1000, DEVICES, WIRES, LEGEND)
