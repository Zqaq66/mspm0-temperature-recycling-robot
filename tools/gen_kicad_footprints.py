#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate simple .kicad_mod footprints for the carrier board, then zip
the schematic + footprints for LCEDA's kicad importer."""

from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "pcb" / "DZZH_carrier.kicad_sch"
OUT = ROOT / "pcb" / "_lceda_import"
ZIP = ROOT / "pcb" / "_lceda_import.zip"

def head(name, attr):
    return f'''(footprint "{name}"
  (version 20240108)
  (generator "pcbnew")
  (layer "F.Cu")
  (attr {attr})
  (fp_text reference "REF**" (at 0 -2.54) (layer "F.SilkS")
    (effects (font (size 1 1) (thickness 0.15))))
  (fp_text value "{name}" (at 0 2.54) (layer "F.Fab")
    (effects (font (size 1 1) (thickness 0.15))))
  (model "{name}.step"
    (offset (xyz 0 0 0))
    (scale (xyz 1 1 1))
    (rotate (xyz 0 0 0)))
'''

def tht(name, pads):
    s = head(name, "through_hole")
    for num, x, y in pads:
        s += f'  (pad "{num}" thru_hole oval (at {x} {y}) (size 1.7 1.7) (drill 1.0) (layers "*.Cu" "*.Mask"))\n'
    return s + ")\n"

def smd(name, pads):
    s = head(name, "smd")
    for num, x, y, w, h in pads:
        s += f'  (pad "{num}" smd roundrect (at {x} {y}) (size {w} {h}) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.2))\n'
    return s + ")\n"

def header_1x(n, pitch=2.54):
    return tht(f"HDR-1X{n}", [(str(i + 1), i * pitch, 0) for i in range(n)])

def header_2x(n, pitch=2.54):
    pads = []
    for i in range(n):
        pads.append((str(2 * i + 1), i * pitch, 0))
        pads.append((str(2 * i + 2), i * pitch, pitch))
    return tht(f"HDR-2X{n}", pads)

def tmx_fp():
    pads = []
    for i in range(45):
        row = 0 if i < 23 else 1
        col = i if i < 23 else i - 23
        pads.append((str(i + 1), col * 2.54, row * 2.54))
    return tht("TMX", pads)

FP = {
    "HDR-1X3": header_1x(3),
    "HDR-1X4": header_1x(4),
    "HDR-1X5": header_1x(5),
    "HDR-1X8": header_1x(8),
    "HDR-2X7": header_2x(7),
    "HDR-2X20": header_2x(20),
    "TMX": tmx_fp(),
    "R0603": smd("R0603", [("1", -0.8, 0, 0.9, 1.0), ("2", 0.8, 0, 0.9, 1.0)]),
    "C0603": smd("C0603", [("1", -0.8, 0, 0.9, 1.0), ("2", 0.8, 0, 0.9, 1.0)]),
    "C0805": smd("C0805", [("1", -1.0, 0, 1.0, 1.4), ("2", 1.0, 0, 1.0, 1.4)]),
    "LED0603": smd("LED0603", [("1", -0.8, 0, 0.9, 1.0), ("2", 0.8, 0, 0.9, 1.0)]),
    "Fuse1206": smd("Fuse1206", [("1", -1.5, 0, 1.4, 1.8), ("2", 1.5, 0, 1.4, 1.8)]),
    "SOT-23": smd("SOT-23", [("1", -1.1, -0.95, 1.0, 0.6), ("2", -1.1, 0.95, 1.0, 0.6), ("3", 1.1, 0, 1.0, 0.6)]),
    "SMA": smd("SMA", [("1", -2.0, 0, 2.0, 2.1), ("2", 2.0, 0, 2.0, 2.1)]),
    "CAP-6.3x7.7": tht("CAP-6.3x7.7", [("1", -2.5, 0), ("2", 2.5, 0)]),
    "SW-TH_6x6": tht("SW-TH_6x6", [("1", -3.0, 0), ("2", 3.0, 0)]),
    "KF301-2P": tht("KF301-2P", [("1", -2.54, 0), ("2", 2.54, 0)]),
}

def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    shutil.copy2(SRC, OUT / SRC.name)
    for name, text in FP.items():
        (OUT / f"{name}.kicad_mod").write_text(text, encoding="utf-8")
    if ZIP.exists():
        ZIP.unlink()
    with zipfile.ZipFile(ZIP, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(OUT.iterdir()):
            z.write(p, p.name)
    print(f"wrote {len(FP)} footprints + schematic -> {ZIP} ({ZIP.stat().st_size} bytes)")

if __name__ == "__main__":
    main()
