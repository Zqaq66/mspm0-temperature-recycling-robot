#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Parse the generated .kicad_sch and export a JSON preview description."""

from pathlib import Path
import json
from kiutils.schematic import Schematic
from kiutils.symbol import SyRect

ROOT = Path(__file__).resolve().parents[1]
SCH = ROOT / "pcb" / "DZZH_carrier.kicad_sch"
OUT = ROOT / "pcb" / "_sch_preview.json"

sch = Schematic.from_file(str(SCH), encoding="utf-8")

lib = {}
for s in sch.libSymbols:
    items = list(s.graphicItems)
    for u in s.units:
        items += list(u.graphicItems)
    rect = next((g for g in items if isinstance(g, SyRect)), None)
    lib[s.entryName] = rect

symbols = []
for s in sch.schematicSymbols:
    rect = lib.get(s.entryName)
    if rect is None:
        continue
    x0 = s.position.X + min(rect.start.X, rect.end.X)
    y0 = s.position.Y - max(rect.start.Y, rect.end.Y)
    w = abs(rect.end.X - rect.start.X)
    h = abs(rect.end.Y - rect.start.Y)
    props = {p.key: p.value for p in s.properties}
    symbols.append({
        "ref": props.get("Reference", s.entryName),
        "value": props.get("Value", ""),
        "x": round(x0, 2), "y": round(y0, 2),
        "w": round(w, 2), "h": round(h, 2),
    })

labels = []
for l in sch.labels:
    j = l.effects.justify.horizontally if l.effects and l.effects.justify else None
    labels.append({"text": l.text,
                   "x": round(l.position.X, 2), "y": round(l.position.Y, 2),
                   "just": j})

wires = []
for g in sch.graphicalItems:
    if getattr(g, "type", None) == "wire":
        p = g.points
        wires.append({"x1": round(p[0].X, 2), "y1": round(p[0].Y, 2),
                      "x2": round(p[1].X, 2), "y2": round(p[1].Y, 2)})

OUT.write_text(json.dumps({"symbols": symbols, "labels": labels, "wires": wires},
                          ensure_ascii=False), encoding="utf-8")
print(f"wrote {OUT}: {len(symbols)} symbols, {len(labels)} labels, {len(wires)} wires")
