#!/usr/bin/env python3
"""Render the wiring figures in src/figures/wiring/ to self-contained SVGs.

    just diagrams                 # every figure
    just diagrams power           # one, by name

Figures:

- `overview.dot`, `power.dot` (Graphviz, right-angle edges): part icons as
  nodes, named `icons/<part>.png` in the source.
- `orin-header` (scripts/wiring_orin_header.py): drawn directly, because the
  header's pins and their wires need a fixed geometry Graphviz cannot hold.

Part icons are our own vector drawings (scripts/wiring_parts.py). Graphviz
here cannot size an SVG image, so it lays out a same-size PNG, and this
script then swaps every icon reference for the SVG itself, inlined as a data
URI: the book shows figures with <img>, and an <img>-loaded SVG cannot load
external images, so every figure must be self-contained anyway.

Needs `dot` (Graphviz) and ImageMagick `convert` on PATH.
"""

from __future__ import annotations

import base64
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wiring_orin_header  # noqa: E402
import wiring_parts  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "src" / "figures" / "wiring"
WORK = ROOT / "tmp" / "diagrams"
ICON_REF = re.compile(r'((?:xlink:)?href)="icons/([A-Za-z0-9_-]+)\.png"')


def stage_icons() -> None:
    icons = WORK / "icons"
    icons.mkdir(parents=True, exist_ok=True)
    for name in wiring_parts.DRAWINGS:
        svg = icons / f"{name}.svg"
        svg.write_text(wiring_parts.icon(name))
        subprocess.run(["convert", "-background", "none", str(svg), str(icons / f"{name}.png")],
                       check=True)


def inline_icons(svg: str) -> str:
    def repl(m: re.Match) -> str:
        data = base64.b64encode(wiring_parts.icon(m.group(2)).encode()).decode()
        return f'{m.group(1)}="data:image/svg+xml;base64,{data}"'

    return ICON_REF.sub(repl, svg)


def render_dot(source: Path) -> None:
    shutil.copy(source, WORK / source.name)
    out = WORK / f"{source.stem}.svg"
    subprocess.run(["dot", "-Tsvg", source.name, "-o", out.name], cwd=WORK, check=True)
    (FIG / out.name).write_text(inline_icons(out.read_text()))


def main() -> int:
    wanted = set(sys.argv[1:])
    if WORK.exists():
        shutil.rmtree(WORK)
    WORK.mkdir(parents=True)
    stage_icons()
    figures: dict[str, Path | None] = {p.stem: p for p in FIG.glob("*.dot")}
    figures["orin-header"] = None
    done = 0
    for name, source in sorted(figures.items()):
        if wanted and name not in wanted:
            continue
        if source is None:
            (FIG / f"{name}.svg").write_text(wiring_orin_header.render())
        else:
            render_dot(source)
        print(f"{name}.svg")
        done += 1
    if not done:
        print("build-diagrams: nothing to render", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
