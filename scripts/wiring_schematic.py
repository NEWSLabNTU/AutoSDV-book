"""A small schematic drawer for the overview and power figures.

Devices are cards (icon plus caption) at fixed positions; wires are
right-angle polylines between card edges, with their label sitting on the
wire's longest segment. Everything is placed by hand in the figure modules,
which is what keeps labels on lines and line ends on devices.

Coordinates are SVG pixels. A card is CARD_W x CARD_H; an anchor on a card
side is given as a fraction along that side (0.5 = the middle).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import wiring_parts as parts

CARD_W, CARD_H = 184, 172   # icon 160 x 120 with a 12 px margin, caption under it
FONT = "DejaVu Sans, sans-serif"


@dataclass
class Device:
    key: str
    drawing: str
    x: float
    y: float
    title: str
    sub: str = ""
    bold: bool = False

    @property
    def w(self) -> float:
        return CARD_W + (40 if self.drawing == "orin" else 0)

    @property
    def h(self) -> float:
        return CARD_H + (30 if self.drawing == "orin" else 0)

    def anchor(self, side: str, t: float = 0.5) -> tuple[float, float]:
        if side == "left":
            return self.x, self.y + self.h * t
        if side == "right":
            return self.x + self.w, self.y + self.h * t
        if side == "top":
            return self.x + self.w * t, self.y
        return self.x + self.w * t, self.y + self.h


@dataclass
class Wire:
    src: tuple[str, str, float]       # (device key, side, fraction)
    dst: tuple[str, str, float]
    color: str
    label: str = ""
    via: list[tuple[float | None, float | None]] = field(default_factory=list)
    dashed: bool = False
    arrow: bool = True                # arrowhead at dst
    label_on: int | None = None       # segment index for the label; default: longest
    label_dy: float = -6


def _text(x, y, s, size=12, anchor="middle", weight="normal", color="#212121",
          rotate=None, halo=True):
    rot = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
    h = ' stroke="white" stroke-width="4" paint-order="stroke"' if halo else ""
    return (f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" fill="{color}" '
            f'text-anchor="{anchor}" font-weight="{weight}"{h}{rot}>{s}</text>')


def _card(d: Device) -> str:
    scale = 1.0
    ix = d.x + (d.w - 160 * scale) / 2
    iy = d.y + 12
    if d.drawing == "orin":
        ix, iy = d.x + (d.w - 200) / 2, d.y + 12
    body = parts.DRAWINGS[d.drawing][0]()
    out = [f'<rect x="{d.x}" y="{d.y}" width="{d.w}" height="{d.h}" rx="8" '
           f'fill="#fafafa" stroke="#bdbdbd" stroke-width="1.5"/>',
           f'<g transform="translate({ix} {iy})">{body}</g>']
    ty = d.y + d.h - (30 if d.sub else 16)
    out.append(_text(d.x + d.w / 2, ty, d.title, 13, weight="bold" if d.bold else "normal",
                     halo=False))
    if d.sub:
        out.append(_text(d.x + d.w / 2, ty + 16, d.sub, 10, color="#616161", halo=False))
    return "".join(out)


def _route(p0, side0, p1, side1, via):
    """Right-angle polyline from p0 (leaving a card by `side0`) to p1 (entering
    a card by `side1`) through `via` points, where a via point may leave x or
    y as None to take the previous point's value. The first segment is
    perpendicular to the side it leaves and the last to the side it enters,
    so a wire never runs along a card's border."""
    horizontal0 = side0 in ("left", "right")
    horizontal1 = side1 in ("left", "right")
    pts = [p0]
    for vx, vy in via:
        px, py = pts[-1]
        nxt = (px if vx is None else vx, py if vy is None else vy)
        if len(pts) == 1 and (nxt[0] == px) == horizontal0:
            # The via asks for a first leg along the card: take the
            # perpendicular leg to it first.
            pts.append((nxt[0], py) if horizontal0 else (px, nxt[1]))
        pts.append(nxt)
    px, py = pts[-1]
    if px != p1[0] and py != p1[1]:
        # The last leg must be perpendicular to the side entered.
        pts.append((px, p1[1]) if horizontal1 else (p1[0], py))
    pts.append(p1)
    clean = [pts[0]]
    for q in pts[1:]:
        if q != clean[-1]:
            clean.append(q)
    return clean


def _wire_svg(w: Wire, devices: dict[str, Device]) -> tuple[str, str]:
    s = devices[w.src[0]].anchor(w.src[1], w.src[2])
    e = devices[w.dst[0]].anchor(w.dst[1], w.dst[2])
    pts = _route(s, w.src[1], e, w.dst[1], w.via)
    d = "M" + " L".join(f"{x:.1f} {y:.1f}" for x, y in pts)
    dash = ' stroke-dasharray="9 6"' if w.dashed else ""
    out = [f'<path d="{d}" fill="none" stroke="white" stroke-width="7"/>',
           f'<path d="{d}" fill="none" stroke="{w.color}" stroke-width="3"{dash} '
           f'stroke-linejoin="miter"/>']
    # The end: an arrowhead whose tip touches the card, or a plain end.
    (ax, ay), (bx, by) = pts[-2], pts[-1]
    if w.arrow:
        if ax == bx:
            sgn = 1 if by > ay else -1
            tri = f"M{bx} {by} l-6 {-10 * sgn} h12 z"
        else:
            sgn = 1 if bx > ax else -1
            tri = f"M{bx} {by} l{-10 * sgn} -6 v12 z"
        out.append(f'<path d="{tri}" fill="{w.color}"/>')
    label = ""
    if w.label:
        segs = list(zip(pts, pts[1:]))
        i = w.label_on if w.label_on is not None else max(
            range(len(segs)), key=lambda k: abs(segs[k][1][0] - segs[k][0][0]) * 1.0
            + abs(segs[k][1][1] - segs[k][0][1]) * 0.6)
        (x0, y0), (x1, y1) = segs[i]
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        if y0 == y1:
            label = _text(mx, my + w.label_dy, w.label, 11, color=w.color, weight="bold")
        else:
            label = _text(mx - 7, my, w.label, 11, color=w.color, weight="bold", rotate=-90)
    return "".join(out), label


def render(width: int, height: int, devices: list[Device], wires: list[Wire],
           extra: str = "") -> str:
    by_key = {d.key: d for d in devices}
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
           f'viewBox="0 0 {width} {height}">',
           f'<rect width="{width}" height="{height}" fill="white"/>', extra]
    drawn = [_wire_svg(w, by_key) for w in wires]
    out.extend(path for path, _ in drawn)
    out.extend(_card(d) for d in devices)
    out.extend(label for _, label in drawn)
    out.append("</svg>")
    return "".join(out)


def legend(x: float, y: float, title: str, rows: list[tuple[str, str, bool]]) -> str:
    """A legend box: rows of (label, colour, dashed)."""
    h = 26 + 20 * len(rows)
    out = [f'<rect x="{x}" y="{y}" width="230" height="{h}" rx="6" fill="white" '
           f'stroke="#9e9e9e"/>', _text(x + 12, y + 18, title, 12, "start", "bold", halo=False)]
    for i, (label, color, dashed) in enumerate(rows):
        yy = y + 38 + i * 20
        dash = ' stroke-dasharray="6 4"' if dashed else ""
        out.append(f'<line x1="{x + 12}" y1="{yy - 4}" x2="{x + 44}" y2="{yy - 4}" '
                   f'stroke="{color}" stroke-width="3"{dash}/>')
        out.append(_text(x + 52, yy, label, 11, "start", halo=False))
    return "".join(out)
