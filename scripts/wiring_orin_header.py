"""The Jetson AGX Orin 40-pin header figure: every pin, every wire.

The header is drawn as it sits on the developer kit: one row of 20 pins
along the side beside the USB-A pair, pin 1 at the USB end, odd pins on the
lower row (NVIDIA marks pin 1 with a white triangle on the PCB). Pin names
are NVIDIA's (carrier board specification, figure 3-4). Wires leave every
pin downward and run with right angles only; a wire from the upper row drops
through the gap between two columns.

`render()` returns the SVG text; build-diagrams.py writes it out.
"""

from __future__ import annotations

import wiring_parts as parts

# Header, BOARD numbers -> NVIDIA name.
NAMES = {
    1: "3.3V", 2: "5.0V", 3: "I2C5_DAT", 4: "5.0V", 5: "I2C5_CLK", 6: "GND",
    7: "MCLK05", 8: "UART1_TX", 9: "GND", 10: "UART1_RX", 11: "UART1_RTS",
    12: "I2S2_CLK", 13: "GPIO32", 14: "GND", 15: "GPIO27", 16: "GPIO08",
    17: "3.3V", 18: "GPIO35", 19: "SPI1_MOSI", 20: "GND", 21: "SPI1_MISO",
    22: "GPIO17", 23: "SPI1_SCK", 24: "SPI1_CS0_N", 25: "GND", 26: "SPI1_CS1_N",
    27: "I2C2_DAT", 28: "I2C2_CLK", 29: "CAN0_DIN", 30: "GND", 31: "CAN0_DOUT",
    32: "GPIO09", 33: "CAN1_DOUT", 34: "GND", 35: "I2S_FS", 36: "UART1_CTS",
    37: "CAN1_DIN", 38: "I2S_SDIN", 39: "GND", 40: "I2S_SDOUT",
}

# Wire colours by what the wire carries.
RED, ORANGE, BLACK, BLUE, YELLOW, GREEN, PURPLE = (
    "#d32f2f", "#ef6c00", "#212121", "#1565c0", "#f9a825", "#2e7d32", "#7b1fa2")


def pin_fill(name: str) -> str:
    if name.startswith("5.0V"):
        return "#e53935"
    if name.startswith("3.3V"):
        return "#fb8c00"
    if name == "GND":
        return "#43a047"
    if name.startswith("I2C"):
        return "#1e88e5"
    if name.startswith("GPIO"):
        return "#8e24aa"
    return "#bdbdbd"


# Parts below the header: (key, drawing, x, title, subtitle, [(pin label, header pin, colour)])
PARTS = [
    ("pca", "pca9685", 60, "PCA9685 PWM driver", "I2C bus 7, address 0x40",
     [("VCC", 4, RED), ("GND", 6, BLACK), ("SDA", 3, BLUE), ("SCL", 5, YELLOW)]),
    ("hall", "ky003", 360, "KY-003 hall sensor", "rear wheel, 12 magnets",
     [("+", 2, RED), ("-", 14, BLACK), ("S", 16, GREEN)]),
    ("imu", "mpu9250", 660, "MPU9250 IMU", "I2C bus 1",
     [("VCC", 17, ORANGE), ("GND", 20, BLACK), ("SDA", 27, BLUE), ("SCL", 28, YELLOW)]),
    ("arb", "arbiter", 960, "Arbiter board", "XIAO RP2350",
     [("D10 AUTO", 22, PURPLE), ("GND", 25, BLACK)]),
]

NOTES = {
    "hall": ["velocity_report pin: 16.",
             "At 5 V its pull-up puts 5 V on S, a",
             "3.3 V pin: add a 10k/20k divider or",
             "move the pull-up to 3.3 V."],
    "arb": ["High only in AUTO; 1 kohm series",
            "resistor on the board. Pin 22 is",
            "proposed: nothing reads it yet."],
    "pca": ["V+ (servo rail) comes from the",
            "7.4 V side: see the power figure."],
}

PITCH = 54          # column spacing of the enlarged header
PIN = 22            # pin square
HX0 = 130           # x of column 0 (pins 1, 2)
ROW_EVEN = 330      # y of the upper row (even pins)
ROW_ODD = ROW_EVEN + 34
PART_Y = 760        # top of the part boxes
W, H = 1240, 1160


def col_x(pin: int) -> float:
    return HX0 + ((pin - 1) // 2) * PITCH


def row_y(pin: int) -> float:
    return ROW_EVEN if pin % 2 == 0 else ROW_ODD


def text(x, y, s, size=12, anchor="middle", weight="normal", color="#212121", rotate=None,
         halo=True):
    rot = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
    # A white halo keeps a label readable where a wire passes under it.
    h = ' stroke="white" stroke-width="3" paint-order="stroke"' if halo else ""
    return (f'<text x="{x}" y="{y}" font-family="DejaVu Sans, sans-serif" font-size="{size}" '
            f'fill="{color}" text-anchor="{anchor}" font-weight="{weight}"{h}{rot}>{s}</text>')


def wire(points, color) -> str:
    d = "M" + " L".join(f"{x:.1f} {y:.1f}" for x, y in points)
    return (f'<path d="{d}" fill="none" stroke="white" stroke-width="7" stroke-linejoin="miter"/>'
            f'<path d="{d}" fill="none" stroke="{color}" stroke-width="3.5" stroke-linejoin="miter"/>')


def render() -> str:
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}">', f'<rect width="{W}" height="{H}" fill="white"/>']

    # Context: the kit, and where the header is on it.
    out.append('<g transform="translate(40 30) scale(0.9)">' + parts.orin() + "</g>")
    out.append(text(250, 60, "Jetson AGX Orin developer kit, 40-pin header", 18, "start", "bold"))
    out.append(text(250, 86, "The header runs along the side beside the USB-A pair. Pin 1 is at the "
                    "USB end, on the lower row,", 13, "start"))
    out.append(text(250, 106, "marked by a white triangle on the PCB. Pin numbers are BOARD numbers, "
                    "as in Jetson.GPIO.", 13, "start"))
    # callout from the small header to the enlarged one
    out.append(f'<path d="M89 130 V{ROW_EVEN - 60} H{HX0 - 40} V{ROW_EVEN - 24}" fill="none" '
               'stroke="#9e9e9e" stroke-dasharray="5 4" stroke-width="1.5"/>')

    # Enlarged header.
    x_last = col_x(39)
    out.append(f'<rect x="{HX0 - 22}" y="{ROW_EVEN - 22}" width="{x_last - HX0 + 44}" '
               f'height="{ROW_ODD - ROW_EVEN + 44}" rx="4" fill="#212121"/>')
    out.append(f'<path d="M{HX0 - 8} {ROW_ODD + 30} l8 -10 l8 10 z" fill="white"/>')
    out.append(text(HX0 - 30, ROW_ODD + 6, "pin 1", 11, "end", color="#616161"))
    out.append(text(HX0 - 30, ROW_ODD + 46, "USB end", 11, "end", color="#616161"))
    used = {p for *_, pins in PARTS for _, p, _ in pins}
    pin_labels = []  # drawn after the wires, so a wire never covers a label
    for pin, name in NAMES.items():
        x, y = col_x(pin), row_y(pin)
        stroke = 'stroke="#ffeb3b" stroke-width="3"' if pin in used else 'stroke="#424242" stroke-width="1"'
        out.append(f'<rect x="{x - PIN / 2}" y="{y - PIN / 2}" width="{PIN}" height="{PIN}" '
                   f'rx="3" fill="{pin_fill(name)}" {stroke}/>')
        pin_labels.append(text(x, y + 4, str(pin), 10, weight="bold", color="white", halo=False))
        if pin % 2 == 0:
            pin_labels.append(text(x + 4, ROW_EVEN - 30, name, 11, "start", rotate=-60,
                            weight="bold" if pin in used else "normal",
                            color="#212121" if pin in used else "#757575"))
        else:
            pin_labels.append(text(x - 4, ROW_ODD + 30, name, 11, "end", rotate=-60,
                            weight="bold" if pin in used else "normal",
                            color="#212121" if pin in used else "#757575"))

    # Parts, with a labelled connector along the top of each.
    entries = {}
    for key, drawing, px, title, sub, pins in PARTS:
        box_w = 240
        out.append(f'<rect x="{px}" y="{PART_Y}" width="{box_w}" height="200" rx="6" '
                   f'fill="#fafafa" stroke="#9e9e9e"/>')
        out.append(f'<g transform="translate({px + 40} {PART_Y + 38}) scale(1.0)">'
                   + getattr(parts, drawing.replace("-", "_"))() + "</g>")
        out.append(text(px + box_w / 2, PART_Y + 176, title, 13, weight="bold"))
        out.append(text(px + box_w / 2, PART_Y + 193, sub, 11, color="#616161"))
        n = len(pins)
        for i, (label, hp, color) in enumerate(pins):
            ex = px + box_w * (i + 1) / (n + 1)
            entries[(key, hp)] = (ex, color)
            out.append(f'<rect x="{ex - 7}" y="{PART_Y - 8}" width="14" height="14" '
                       f'fill="{color}" stroke="#212121"/>')
            out.append(text(ex, PART_Y + 22, label, 11, weight="bold"))
        for j, line in enumerate(NOTES.get(key, [])):
            out.append(text(px, PART_Y + 228 + j * 16, line, 11, "start", color="#424242"))

    # Wires. Each gets its own horizontal level between header and parts;
    # levels are assigned so that a wire with a shorter horizontal run sits
    # nearer the parts, which keeps most crossings out.
    runs = []
    for (key, hp), (ex, color) in entries.items():
        sx = col_x(hp) + (PITCH / 2 if hp % 2 == 0 else 0)
        runs.append((abs(ex - sx), key, hp, sx, ex, color))
    runs.sort(reverse=True)
    top, bottom = ROW_ODD + 130, PART_Y - 40
    step = (bottom - top) / max(1, len(runs) - 1)
    for i, (_, key, hp, sx, ex, color) in enumerate(runs):
        level = top + i * step
        x0, y0 = col_x(hp), row_y(hp)
        if hp % 2 == 0:  # upper row: down between the rows, into the column gap, down
            mid = (ROW_EVEN + ROW_ODD) / 2
            pts = [(x0, y0), (x0, mid), (sx, mid), (sx, level), (ex, level), (ex, PART_Y - 8)]
        else:  # lower row: straight down
            pts = [(x0, y0), (x0, level), (ex, level), (ex, PART_Y - 8)]
        out.append(wire(pts, color))
        out.append(f'<circle cx="{x0}" cy="{y0}" r="4" fill="{color}"/>')

    out.extend(pin_labels)

    # Legend.
    lx, ly = 40, 1110
    out.append(text(lx, ly, "Wires:", 12, "start", "bold"))
    for i, (label, color) in enumerate((("5 V", RED), ("3.3 V", ORANGE), ("GND", BLACK),
                                        ("SDA", BLUE), ("SCL", YELLOW), ("hall signal", GREEN),
                                        ("AUTO line", PURPLE))):
        x = lx + 70 + i * 120
        out.append(f'<rect x="{x}" y="{ly - 10}" width="28" height="6" fill="{color}"/>')
        out.append(text(x + 34, ly - 2, label, 12, "start"))
    out.append(text(lx, ly + 30, "Pins: yellow outline = in use. Colours: 5 V red, 3.3 V orange, "
                    "GND green, I2C blue, GPIO purple, other grey.", 12, "start", color="#424242"))
    out.append("</svg>")
    return "".join(out)
