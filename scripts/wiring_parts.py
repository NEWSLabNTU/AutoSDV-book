"""Part drawings for the wiring figures: our own flat vector icons.

Each function returns the body of an SVG drawn in a 160 x 120 box (the Orin
in 200 x 150). They are drawn for this book from the parts' general look,
not traced from anyone's photo, so they carry no third-party image rights.

`icon(name)` wraps one in a standalone <svg>, which the Graphviz figures use
as node images; the Orin header figure places the bodies itself.
"""

from __future__ import annotations

PCB_GREEN = "#2e7d32"
PCB_BLUE = "#1565c0"
PCB_PURPLE = "#6a1b9a"
PCB_BLACK = "#263238"
METAL = "#9e9e9e"
DARK = "#37474f"
GOLD = "#c9a227"
WHITE = "#ffffff"
STROKE = 'stroke="#212121" stroke-width="1.5"'


def _pins(x: float, y: float, n: int, dx: float = 8, dy: float = 0, r: float = 2.2,
          color: str = GOLD) -> str:
    return "".join(f'<circle cx="{x + i * dx}" cy="{y + i * dy}" r="{r}" fill="{color}"/>'
                   for i in range(n))


def _text(x: float, y: float, s: str, size: int = 9, color: str = WHITE,
          anchor: str = "middle", weight: str = "normal") -> str:
    return (f'<text x="{x}" y="{y}" font-family="DejaVu Sans, sans-serif" font-size="{size}" '
            f'fill="{color}" text-anchor="{anchor}" font-weight="{weight}">{s}</text>')


def orin() -> str:
    """Jetson AGX Orin developer kit, three-quarter front: vents, ports."""
    return (
        f'<rect x="10" y="10" width="180" height="130" rx="8" fill="#455a64" {STROKE}/>'
        f'<rect x="10" y="10" width="180" height="62" rx="8" fill="#90a4ae" {STROKE}/>'
        + "".join(f'<line x1="{26 + i * 12}" y1="18" x2="{18 + i * 12}" y2="66" '
                  f'stroke="#546e7a" stroke-width="5"/>' for i in range(14))
        + '<circle cx="100" cy="41" r="22" fill="#37474f" opacity="0.6"/>'
        + f'<rect x="22" y="96" width="22" height="14" rx="2" fill="#cfd8dc" {STROKE}/>'
        + f'<rect x="22" y="114" width="22" height="14" rx="2" fill="#cfd8dc" {STROKE}/>'
        + f'<rect x="54" y="104" width="120" height="14" rx="1" fill="#212121"/>'
        + _pins(59, 108, 20, dx=5.6, r=1.4) + _pins(59, 114, 20, dx=5.6, r=1.4)
        + _text(114, 90, "40-pin header", 9)
        + _text(150, 132, "JETSON AGX ORIN", 8, "#cfd8dc", weight="bold")
    )


def battery(cells: str, volts: str) -> str:
    """LiPo pack: shrink-wrapped brick, power leads and a balance lead."""
    return (
        f'<rect x="14" y="30" width="120" height="66" rx="8" fill="#f9a825" {STROKE}/>'
        f'<rect x="14" y="44" width="120" height="38" fill="#fbc02d"/>'
        + _text(74, 60, cells + " LiPo", 13, "#3e2723", weight="bold")
        + _text(74, 76, volts, 11, "#3e2723")
        + '<path d="M134 48 H150 V40" stroke="#d32f2f" stroke-width="4" fill="none"/>'
        + '<path d="M134 58 H156 V40" stroke="#212121" stroke-width="4" fill="none"/>'
        + '<path d="M134 80 H146 V100" stroke="#fafafa" stroke-width="2.5" fill="none"/>'
        + f'<rect x="140" y="100" width="14" height="7" fill="{WHITE}" {STROKE}/>'
    )


def breaker() -> str:
    """DIN-rail circuit breaker used as a power switch."""
    return (
        f'<rect x="50" y="10" width="60" height="100" rx="4" fill="#eceff1" {STROKE}/>'
        f'<rect x="58" y="16" width="44" height="14" fill="{METAL}"/>'
        f'<rect x="58" y="90" width="44" height="14" fill="{METAL}"/>'
        f'<rect x="68" y="42" width="24" height="36" rx="3" fill="#1e88e5" {STROKE}/>'
        f'<rect x="73" y="46" width="14" height="14" rx="2" fill="#90caf9"/>'
        + _text(80, 72, "ON", 8, WHITE, weight="bold")
    )


def dcdc() -> str:
    """Step-down converter: aluminium case, terminal block each end."""
    return (
        f'<rect x="20" y="30" width="120" height="60" rx="4" fill="#b0bec5" {STROKE}/>'
        + "".join(f'<line x1="{36 + i * 10}" y1="36" x2="{36 + i * 10}" y2="84" '
                  f'stroke="#90a4ae" stroke-width="4"/>' for i in range(10))
        + f'<rect x="8" y="44" width="14" height="32" fill="#43a047" {STROKE}/>'
        + f'<rect x="138" y="44" width="14" height="32" fill="#43a047" {STROKE}/>'
        + _text(80, 66, "DC-DC 12 V", 12, "#263238", weight="bold")
    )


def zed_link() -> str:
    """ZED Link capture card: board with GMSL2 Fakra sockets and DC jack."""
    return (
        f'<rect x="16" y="24" width="128" height="72" rx="4" fill="{PCB_BLACK}" {STROKE}/>'
        + "".join(f'<rect x="{28 + i * 26}" y="80" width="18" height="18" rx="3" '
                  f'fill="#1e88e5" {STROKE}/>' for i in range(4))
        + f'<circle cx="128" cy="40" r="8" fill="{DARK}" {STROKE}/>'
        + f'<rect x="30" y="34" width="56" height="30" fill="#455a64"/>'
        + _text(58, 54, "ZED Link", 10)
    )


def zed_x_mini() -> str:
    """ZED X Mini stereo camera: bar with two lenses."""
    return (
        f'<rect x="14" y="40" width="132" height="44" rx="14" fill="#cfd8dc" {STROKE}/>'
        + f'<circle cx="44" cy="62" r="14" fill="#212121"/><circle cx="44" cy="62" r="7" fill="#3949ab"/>'
        + f'<circle cx="116" cy="62" r="14" fill="#212121"/><circle cx="116" cy="62" r="7" fill="#3949ab"/>'
        + _text(80, 66, "ZED X", 9, "#263238", weight="bold")
    )


def lidar() -> str:
    """Spinning LiDAR: cylinder with a dark window band."""
    return (
        f'<rect x="48" y="16" width="64" height="88" rx="6" fill="#eceff1" {STROKE}/>'
        f'<ellipse cx="80" cy="16" rx="32" ry="8" fill="#cfd8dc" {STROKE}/>'
        f'<rect x="48" y="40" width="64" height="34" fill="#263238"/>'
        f'<ellipse cx="80" cy="104" rx="32" ry="8" fill="#b0bec5" {STROKE}/>'
        + _text(80, 92, "LiDAR", 10, "#263238", weight="bold")
    )


def mpu9250() -> str:
    """MPU9250 breakout: small purple board, chip, 10-pin row."""
    return (
        f'<rect x="40" y="24" width="80" height="72" rx="3" fill="{PCB_PURPLE}" {STROKE}/>'
        + f'<rect x="68" y="44" width="24" height="24" fill="#212121"/>'
        + _pins(46, 88, 10, dx=7.5, r=2.4)
        + _text(80, 38, "MPU9250", 9)
    )


def ky003() -> str:
    """KY-003 hall module: black board, A3144 sensor, LED, 3 pins."""
    return (
        f'<rect x="44" y="30" width="72" height="56" rx="3" fill="{PCB_BLACK}" {STROKE}/>'
        + f'<rect x="58" y="16" width="16" height="18" fill="#212121" {STROKE}/>'
        + '<circle cx="98" cy="46" r="5" fill="#e53935"/>'
        + _pins(68, 78, 3, dx=12, r=3)
        + _text(80, 64, "KY-003", 9)
    )


def modem_5g() -> str:
    """Industrial cellular router: DIN box with two antennas."""
    return (
        f'<rect x="34" y="40" width="92" height="66" rx="4" fill="#cfd8dc" {STROKE}/>'
        + f'<rect x="44" y="12" width="8" height="30" rx="3" fill="#212121"/>'
        + f'<rect x="108" y="12" width="8" height="30" rx="3" fill="#212121"/>'
        + "".join(f'<circle cx="{52 + i * 14}" cy="56" r="3" fill="#43a047"/>' for i in range(4))
        + _text(80, 86, "5G", 16, "#37474f", weight="bold")
    )


def gps() -> str:
    """GNSS puck antenna/receiver."""
    return (
        f'<ellipse cx="80" cy="70" rx="46" ry="26" fill="#37474f" {STROKE}/>'
        f'<ellipse cx="80" cy="62" rx="46" ry="26" fill="#546e7a" {STROKE}/>'
        + _text(80, 66, "GNSS", 12, WHITE, weight="bold")
        + '<path d="M126 70 H146 V100" stroke="#212121" stroke-width="3" fill="none"/>'
    )


def pca9685() -> str:
    """PCA9685 servo driver: blue board, 4 x (3 x 4) servo headers, terminal block."""
    rows = "".join(_pins(30 + g * 26, 74 + r * 8, 4, dx=5.5, r=1.8,
                         color=("#212121", "#d32f2f", "#fdd835")[r])
                   for g in range(4) for r in range(3))
    return (
        f'<rect x="12" y="24" width="136" height="80" rx="3" fill="{PCB_BLUE}" {STROKE}/>'
        + f'<rect x="64" y="36" width="22" height="22" fill="#212121"/>'
        + f'<rect x="100" y="30" width="26" height="16" fill="#43a047" {STROKE}/>'
        + _pins(18, 40, 6, dx=0, dy=6, r=1.8)
        + rows
        + _text(110, 60, "PCA9685", 8)
    )


def receiver() -> str:
    """RC receiver: small box, antenna lead, channel pins."""
    return (
        f'<rect x="38" y="40" width="84" height="54" rx="5" fill="#212121" {STROKE}/>'
        + '<path d="M122 52 H140 V18" stroke="#757575" stroke-width="2" fill="none"/>'
        + "".join(_pins(46 + c * 10, 50, 1, r=2.2) for c in range(7))
        + _text(80, 80, "RX", 13, WHITE, weight="bold")
    )


def selector_switch() -> str:
    """Selector switch, Orin or joystick: body, nut, lever, solder lugs."""
    return (
        f'<rect x="54" y="50" width="52" height="46" rx="3" fill="#eceff1" {STROKE}/>'
        + f'<rect x="64" y="38" width="32" height="14" fill="{METAL}" {STROKE}/>'
        + f'<rect x="76" y="10" width="8" height="30" rx="4" fill="{METAL}" {STROKE}/>'
        + "".join(_pins(64 + c * 16, 102 + r * 8, 1, r=2.5, color=METAL)
                  for c in range(3) for r in range(2))
        + _text(80, 78, "SEL", 9, "#263238", weight="bold")
    )


def arbiter() -> str:
    """Arbiter carrier: green perfboard, XIAO with USB-C, 74LVC245."""
    return (
        f'<rect x="14" y="20" width="132" height="84" rx="3" fill="{PCB_GREEN}" {STROKE}/>'
        + f'<rect x="24" y="32" width="46" height="56" rx="3" fill="#212121" {STROKE}/>'
        + f'<rect x="38" y="26" width="18" height="10" rx="2" fill="{METAL}" {STROKE}/>'
        + '<circle cx="47" cy="78" r="4" fill="#43a047"/>'
        + _text(47, 62, "XIAO", 9)
        + f'<rect x="86" y="44" width="44" height="22" fill="#212121"/>'
        + _text(108, 59, "245", 8)
        + _pins(22, 98, 16, dx=8, r=1.8)
    )


def esc() -> str:
    """Brushless ESC: finned case, battery leads in, three phase leads out."""
    return (
        f'<rect x="22" y="28" width="104" height="70" rx="4" fill="#78909c" {STROKE}/>'
        + f'<rect x="30" y="36" width="88" height="54" rx="2" fill="{PCB_BLACK}"/>'
        + _text(74, 68, "ESC", 14, WHITE, weight="bold")
        + "".join(f'<path d="M126 {44 + i * 14} H150" stroke="{c}" stroke-width="4"/>'
                  for i, c in enumerate(("#1e88e5", "#fdd835", "#212121")))
        + '<path d="M22 52 H6" stroke="#d32f2f" stroke-width="4"/>'
        + '<path d="M22 66 H6" stroke="#212121" stroke-width="4"/>'
    )


def motor() -> str:
    """Outrunner/inrunner brushless can with shaft and three leads."""
    return (
        f'<rect x="40" y="30" width="80" height="60" rx="10" fill="#607d8b" {STROKE}/>'
        + "".join(f'<line x1="{48 + i * 9}" y1="34" x2="{48 + i * 9}" y2="86" '
                  f'stroke="#78909c" stroke-width="3"/>' for i in range(8))
        + f'<rect x="120" y="54" width="26" height="12" fill="{METAL}" {STROKE}/>'
        + "".join(f'<path d="M40 {48 + i * 12} H16" stroke="{c}" stroke-width="3"/>'
                  for i, c in enumerate(("#1e88e5", "#fdd835", "#212121")))
    )


def servo() -> str:
    """Standard servo: case, mounting tabs, output horn, lead."""
    return (
        f'<rect x="20" y="46" width="120" height="16" rx="2" fill="#424242" {STROKE}/>'
        + f'<rect x="38" y="30" width="84" height="64" rx="4" fill="#212121" {STROKE}/>'
        + f'<circle cx="96" cy="44" r="8" fill="{WHITE}" {STROKE}/>'
        + f'<rect x="68" y="40" width="56" height="8" rx="4" fill="{WHITE}" {STROKE}/>'
        + '<path d="M38 82 H18 V104" stroke="#ff8f00" stroke-width="3" fill="none"/>'
        + _text(72, 78, "SERVO", 9)
    )


DRAWINGS = {
    "orin": (orin, 200, 150),
    "battery-6s": (lambda: battery("6S", "22.2 V"), 160, 120),
    "battery-2s": (lambda: battery("2S", "7.4 V"), 160, 120),
    "upper-power-switch": (breaker, 160, 120),
    "powertrain-switch": (breaker, 160, 120),
    "dcdc-12v": (dcdc, 160, 120),
    "zed-link": (zed_link, 160, 120),
    "zed-x-mini": (zed_x_mini, 160, 120),
    "lidar": (lidar, 160, 120),
    "ky003": (ky003, 160, 120),
    "moxa-5g": (modem_5g, 160, 120),
    "gps": (gps, 160, 120),
    "pca9685": (pca9685, 160, 120),
    "receiver": (receiver, 160, 120),
    "selector-switch": (selector_switch, 160, 120),
    "esc": (esc, 160, 120),
    "motor": (motor, 160, 120),
    "servo": (servo, 160, 120),
}


def icon(name: str) -> str:
    draw, w, h = DRAWINGS[name]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
            f'viewBox="0 0 {w} {h}">{draw()}</svg>')
