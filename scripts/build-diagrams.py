#!/usr/bin/env python3
"""Render the wiring figures in src/figures/wiring/ to self-contained SVGs.

    just diagrams           # every source
    just diagrams power     # one, by stem

Sources sit beside their output: `<stem>.dot` (Graphviz, the overview) and
`<stem>.yml` (WireViz, the pin-level harness drawings). Both name part photos
as `photos/<part>.jpg`, relative to src/figures/wiring/.

Two things this script does that the tools do not:

- A photo that is not there yet is drawn as a labelled grey placeholder, so a
  figure builds before every part has been photographed. Replacing the
  placeholder is dropping the real photo at the same path and re-running.
- Every image is inlined into the SVG as a resized data URI. The book shows
  figures with <img>, and a browser does not load images an <img>-loaded SVG
  references, so an SVG pointing at photos/*.jpg renders with empty frames.

Needs `dot` (Graphviz) on PATH; WireViz runs through `uvx wireviz`, and the
recipe adds Pillow for this script only (`uv run --with pillow`).
"""

from __future__ import annotations

import base64
import io
import re
import shutil
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
FIG = ROOT / "src" / "figures" / "wiring"
WORK = ROOT / "tmp" / "diagrams"
PHOTO_REF = re.compile(r"photos/([A-Za-z0-9_-]+)\.(?:jpg|jpeg|png)")
THUMB_PX = 360  # longest side of an inlined photo


def placeholder(name: str, out: Path) -> None:
    img = Image.new("RGB", (400, 300), (225, 225, 225))
    draw = ImageDraw.Draw(img)
    draw.rectangle([4, 4, 395, 295], outline=(150, 150, 150), width=4)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 30)
    except OSError:
        font = ImageFont.load_default()
    text = f"photo:\n{name}"
    box = draw.multiline_textbbox((0, 0), text, font=font, align="center")
    w, h = box[2] - box[0], box[3] - box[1]
    draw.multiline_text(((400 - w) / 2, (300 - h) / 2), text, fill=(110, 110, 110),
                        font=font, align="center")
    img.save(out, "JPEG", quality=85)


def stage_photos(source: Path) -> list[str]:
    """Copy each referenced photo (or a placeholder) into the work dir."""
    names = sorted(set(PHOTO_REF.findall(source.read_text())))
    (WORK / "photos").mkdir(parents=True, exist_ok=True)
    missing = []
    for name in names:
        dest = WORK / "photos" / f"{name}.jpg"
        real = next((p for p in (FIG / "photos").glob(f"{name}.*")
                     if p.suffix.lower() in (".jpg", ".jpeg", ".png")), None)
        if real is None:
            placeholder(name, dest)
            missing.append(name)
        else:
            # Graphviz draws an image at its pixel size (WireViz gives it no
            # box), so a camera photo would fill the figure: shrink it first.
            img = Image.open(real).convert("RGB")
            img.thumbnail((400, 300))
            img.save(dest, "JPEG", quality=90)
    # Sources name .jpg; a real .png is converted above, so rewrite the refs.
    text = PHOTO_REF.sub(lambda m: f"photos/{m.group(1)}.jpg", source.read_text())
    (WORK / source.name).write_text(text)
    return missing


def inline_images(svg: str) -> str:
    def repl(m: re.Match) -> str:
        path = WORK / m.group(2)
        img = Image.open(path).convert("RGB")
        img.thumbnail((THUMB_PX, THUMB_PX))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=80)
        data = base64.b64encode(buf.getvalue()).decode()
        return f'{m.group(1)}="data:image/jpeg;base64,{data}"'

    return re.sub(r'((?:xlink:)?href)="((?:\./)?photos/[^"]+)"', repl, svg)


def render(source: Path) -> None:
    if WORK.exists():
        shutil.rmtree(WORK)
    WORK.mkdir(parents=True)
    missing = stage_photos(source)
    stem = source.stem
    if source.suffix == ".dot":
        subprocess.run(["dot", "-Tsvg", source.name, "-o", f"{stem}.svg"],
                       cwd=WORK, check=True)
    else:
        subprocess.run(["uvx", "wireviz", "-f", "s", source.name], cwd=WORK, check=True)
    svg = (WORK / f"{stem}.svg").read_text()
    (FIG / f"{stem}.svg").write_text(inline_images(svg))
    note = f"  (placeholders: {', '.join(missing)})" if missing else ""
    print(f"{stem}.svg{note}")


def main() -> int:
    wanted = set(sys.argv[1:])
    sources = sorted(p for p in FIG.iterdir() if p.suffix in (".dot", ".yml"))
    sources = [p for p in sources if not wanted or p.stem in wanted]
    if not sources:
        print("build-diagrams: nothing to render", file=sys.stderr)
        return 1
    for source in sources:
        render(source)
    return 0


if __name__ == "__main__":
    sys.exit(main())
