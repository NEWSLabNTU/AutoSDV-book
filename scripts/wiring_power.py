"""Battery and power wiring only. Data and signal wiring is wiring_overview.py.

Two independent batteries: the 6S pack feeds the computer and sensors
through a 12 V DC-DC converter; the 2S pack feeds the drive. Each has its
own cell monitor with an alarm on the balance lead.
"""

from __future__ import annotations

from wiring_schematic import Device, Wire, render

PWR = "#d32f2f"

DEVICES = [
    # Drive, top band
    Device("bat2s", "battery-2s", 60, 80, "2S LiPo, 7.4 V", "own cell monitor", bold=True),
    Device("ptsw", "powertrain-switch", 360, 80, "powertrain switch"),
    Device("esc", "esc", 660, 80, "motor ESC", "Dynamite Fuze brushless"),
    Device("pca", "pca9685", 660, 320, "PCA9685, V+ terminal", "servo rail"),
    Device("servo", "servo", 960, 320, "steering servo"),
    Device("rx", "receiver", 960, 80, "RC receiver", "5 V from the ESC's BEC, to confirm"),
    # Upper power, bottom band
    Device("bat6s", "battery-6s", 60, 640, "6S LiPo, 22.2 V", "own cell monitor", bold=True),
    Device("upsw", "upper-power-switch", 360, 640, "upper power switch"),
    Device("dcdc", "dcdc-12v", 660, 640, "DC-DC", "22.2 V to 12 V"),
    Device("orin", "orin", 1060, 580, "Jetson AGX Orin", "DC jack", bold=True),
    Device("zedlink", "zed-link", 1360, 640, "ZED Link", "powers the ZED X Mini over GMSL2"),
    Device("lidar", "lidar", 1660, 640, "LiDAR"),
    Device("g5", "moxa-5g", 1960, 640, "5G router (optional)", "12 V at 2 A"),
]

WIRES = [
    Wire(("bat2s", "right", 0.5), ("ptsw", "left", 0.5), PWR, "7.4 V"),
    Wire(("ptsw", "right", 0.5), ("esc", "left", 0.5), PWR, "7.4 V switched"),
    Wire(("ptsw", "right", 0.5), ("pca", "left", 0.5), PWR, "", via=[(600, None)]),
    Wire(("pca", "right", 0.5), ("servo", "left", 0.5), PWR, "servo power"),
    Wire(("esc", "right", 0.5), ("rx", "left", 0.5), PWR, "5 V BEC", dashed=True),
    Wire(("bat6s", "right", 0.5), ("upsw", "left", 0.5), PWR, "22.2 V"),
    Wire(("upsw", "right", 0.5), ("dcdc", "left", 0.5), PWR, "22.2 V switched"),
    # A 12 V bus along the bottom, one drop per load; the optional 5G router
    # hangs off a dashed line under the bus.
    Wire(("dcdc", "bottom", 0.5), ("g5", "bottom", 0.5), PWR, "12 V, optional",
         via=[(None, 930)], dashed=True, label_on=1),
    Wire(("dcdc", "right", 0.5), ("orin", "left", 0.5), PWR, "12 V"),
    Wire(("dcdc", "bottom", 0.5), ("zedlink", "bottom", 0.5), PWR, "12 V", via=[(None, 900)],
         label_on=1),
    Wire(("dcdc", "bottom", 0.5), ("lidar", "bottom", 0.5), PWR, "", via=[(None, 900)]),
]

TITLES = (
    '<text x="60" y="50" font-family="DejaVu Sans, sans-serif" font-size="18" font-weight="bold" '
    'fill="#212121">Drive: 2S battery</text>'
    '<text x="60" y="610" font-family="DejaVu Sans, sans-serif" font-size="18" font-weight="bold" '
    'fill="#212121">Computer and sensors: 6S battery</text>'
    '<line x1="60" y1="560" x2="2200" y2="560" stroke="#bdbdbd" stroke-dasharray="8 6"/>'
)


def render_svg() -> str:
    return render(2240, 980, DEVICES, WIRES, TITLES)
