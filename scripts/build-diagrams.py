#!/usr/bin/env python3
"""Render the wiring figures to src/figures/wiring/<name>.svg.

    just diagrams                 # every figure
    just diagrams power           # one, by name

Each figure is a Python module here (wiring_overview, wiring_power,
wiring_orin_header) that places its parts and routes its wires by hand, on
top of wiring_schematic (cards, right-angle wires, on-wire labels) and
wiring_parts (our own vector drawings of the parts). Nothing is laid out by
a tool: that is what keeps every label on its wire and every wire end on its
device. The SVGs are self-contained, so the book can show them with <img>.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wiring_orin_header  # noqa: E402
import wiring_overview  # noqa: E402
import wiring_power  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "src" / "figures" / "wiring"

FIGURES = {
    "overview": wiring_overview.render_svg,
    "power": wiring_power.render_svg,
    "orin-header": wiring_orin_header.render,
}


def main() -> int:
    wanted = set(sys.argv[1:])
    done = 0
    for name, draw in FIGURES.items():
        if wanted and name not in wanted:
            continue
        (FIG / f"{name}.svg").write_text(draw())
        print(f"{name}.svg")
        done += 1
    if not done:
        print("build-diagrams: nothing to render", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
