#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate pcb/DZZH_carrier.kicad_sch from the same data as gen_carrier_hw.py.

Every component is a labelled block; every pin gets a short wire stub plus a
local net label.  Identical labels form a net (KiCad single-sheet semantics).
This keeps the file easy to import into 立创EDA and easy to review.
"""

from pathlib import Path
import sys
import uuid
import math

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_carrier_hw as hw

from kiutils.schematic import (Schematic, SchematicSymbol, LocalLabel,
                               Connection, NoConnect, Text as SchText,
                               SymbolProjectInstance, SymbolProjectPath)
from kiutils.symbol import Symbol, SymbolPin, SyRect
from kiutils.items.common import (Position, Effects, Font, Stroke, Fill, Property,
                                  PageSettings, TitleBlock, Justify)

OUT = Path(__file__).resolve().parents[1] / "pcb" / "DZZH_carrier.kicad_sch"
PIN_LEN = 5.08
STUB = 0.0
SPACING = 5.08

def uid():
    return str(uuid.uuid4())

def fnt(h=1.27):
    return Font(height=h)

def eff(h=1.27, horiz=None):
    return Effects(font=fnt(h), justify=Justify(horizontally=horiz))

# ---------------------------------------------------------------- data
items = []   # (ref, value, footprint, [(pin_name, net), ...])
for ref, name, fp, pins in hw.COMPONENTS:
    items.append((ref, name, fp, [(p, n) for p, n, _note in pins]))

tmx_pins = [(pin, net) for pin, (net, _note) in hw.TMX_PINS.items()]
items.append(("TMX", "天猛星 MSPM0G3507", "模块（双排母）", tmx_pins))

# ---------------------------------------------------------------- layout
LAYOUT = {
    "TMX":  (420, 60, 90),
    "J1":   (60, 40, 30),  "F1": (120, 40, 30),  "D1": (180, 40, 34),
    "D2":   (245, 40, 30), "TVS1": (310, 40, 30),
    "U1":   (60, 105, 40), "U2": (150, 105, 40), "U3": (250, 105, 36),
    "C1":   (60, 170, 28), "C2": (110, 170, 28), "C3": (160, 170, 28),
    "C4":   (210, 170, 28), "C5": (260, 170, 28), "C6": (310, 170, 28),
    "R1":   (60, 220, 28), "R2": (110, 220, 28), "R3": (160, 220, 28),
    "R4":   (210, 220, 28), "R5": (260, 220, 28), "R6": (310, 220, 28),
    "R7":   (60, 270, 28), "R8": (110, 270, 28), "LED1": (160, 270, 32),
    "J3":   (600, 60, 80), "J4": (600, 200, 80),
    "J5":   (600, 360, 50), "J6": (600, 430, 50),
    "J7":   (700, 360, 50), "J8": (700, 430, 50),
    "J9":   (800, 60, 60), "J10": (800, 140, 60),
    "J11":  (800, 220, 60), "J12": (800, 300, 60),
    "J13":  (800, 380, 60), "J14": (800, 460, 60),
    "J15":  (960, 60, 70), "J16": (960, 170, 60),
    "J17":  (960, 250, 60), "SW1": (960, 330, 40),
    "K1":   (300, 360, 34), "K2": (300, 410, 34), "K3": (300, 460, 34),
}

# EasyEDA/LCEDA-style footprint names (module sockets are 2.54 headers)
FOOTPRINTS = {
    "J1": "KF301-2P", "F1": "Fuse1206",
    "D1": "SOT-23", "D2": "SMA", "TVS1": "SMA",
    "U1": "HDR-1X4", "U2": "HDR-1X4", "U3": "HDR-1X3",
    "C1": "CAP-6.3x7.7", "C2": "C0603", "C3": "C0805",
    "C4": "C0603", "C5": "C0805", "C6": "C0603",
    "R1": "R0603", "R2": "R0603", "R3": "R0603", "R4": "R0603",
    "R5": "R0603", "R6": "R0603", "R7": "R0603", "R8": "R0603",
    "LED1": "LED0603",
    "J3": "HDR-2X7", "J4": "HDR-2X7",
    "J5": "HDR-1X5", "J6": "HDR-1X5", "J7": "HDR-1X5", "J8": "HDR-1X5",
    "J9": "HDR-1X4", "J10": "HDR-1X4", "J11": "HDR-1X4", "J12": "HDR-1X4",
    "J13": "HDR-1X4", "J14": "HDR-1X4", "J16": "HDR-1X4", "J17": "HDR-1X4",
    "J15": "HDR-1X8",
    "SW1": "SW-TH_6x6", "K1": "SW-TH_6x6", "K2": "SW-TH_6x6", "K3": "SW-TH_6x6",
    "TMX": "TMX",
}

def split_pins(pins):
    half = math.ceil(len(pins) / 2)
    return pins[:half], pins[half:]

def block_height(pins):
    left, right = split_pins(pins)
    n = max(len(left), len(right))
    return max(26, 10 + (n - 1) * SPACING + 10)

# ---------------------------------------------------------------- build
lib_symbols = []
schematic_symbols = []
graphical_items = []
labels = []
no_connects = []
expected = {}

for ref, value, fp, pins in items:
    if ref not in LAYOUT:
        raise SystemExit(f"layout missing for {ref}")
    X, Y, W = LAYOUT[ref]
    H = block_height(pins)
    left, right = split_pins(pins)

    # ---- library symbol ----
    sym_pins = []
    for i, (pn, net) in enumerate(left):
        y = -(10 + i * SPACING)
        sym_pins.append(SymbolPin(
            electricalType="passive", graphicalStyle="line",
            position=Position(-PIN_LEN, y, 0), length=PIN_LEN,
            name=pn, number=str(i + 1),
            nameEffects=Effects(font=Font(height=1.27)),
            numberEffects=Effects(font=Font(height=1.27))))
    for i, (pn, net) in enumerate(right):
        y = -(10 + i * SPACING)
        sym_pins.append(SymbolPin(
            electricalType="passive", graphicalStyle="line",
            position=Position(W + PIN_LEN, y, 180), length=PIN_LEN,
            name=pn, number=str(len(left) + i + 1),
            nameEffects=Effects(font=Font(height=1.27)),
            numberEffects=Effects(font=Font(height=1.27))))

    unit = Symbol(
        libraryNickname=None, entryName=f"{ref}_1_1",
        graphicItems=[SyRect(start=Position(0, 0), end=Position(W, -H),
                             stroke=Stroke(width=0.15), fill=Fill(type="none"))],
        pins=sym_pins,
    )

    lib_symbols.append(Symbol(
        libraryNickname=None, entryName=ref,
        pinNames=True, pinNamesOffset=1.016, hidePinNumbers=False,
        inBom=True, onBoard=True,
        properties=[
            Property(key="Reference", value=ref, id=0,
                     position=Position(0, 0, 0), effects=eff(1.27)),
            Property(key="Value", value=value, id=1,
                     position=Position(0, 0, 0), effects=eff(1.27)),
            Property(key="Footprint", value=FOOTPRINTS.get(ref, fp), id=2,
                     position=Position(0, 0, 0), effects=eff(1.0)),
        ],
        graphicItems=[], pins=[], units=[unit],
    ))

    # ---- instance ----
    schematic_symbols.append(SchematicSymbol(
        libraryNickname=None, entryName=ref,
        libName=ref,
        position=Position(X, Y, 0),
        unit=1,
        inBom=True, onBoard=True,
        properties=[
            Property(key="Reference", value=ref, id=0,
                     position=Position(X + 2, Y - 8, 0), effects=eff(1.27)),
            Property(key="Value", value=value, id=1,
                     position=Position(X + 2, Y - 4, 0), effects=eff(1.15)),
            Property(key="Footprint", value=FOOTPRINTS.get(ref, fp), id=2,
                     position=Position(X + 2, Y + H + 4, 0), effects=eff(0.9)),
        ],
        pins={str(i + 1): uid() for i in range(len(pins))},
        instances=[SymbolProjectInstance(
            name="",
            paths=[SymbolProjectPath(sheetInstancePath="/", reference=ref, unit=1)])],
    ))

    # ---- wires + labels ----
    for i, (pn, net) in enumerate(left):
        y = Y + 10 + i * SPACING
        x0 = X - PIN_LEN
        x1 = x0 - STUB
        if STUB > 0:
            graphical_items.append(Connection(type="wire", points=[Position(x0, y), Position(x1, y)],
                                              stroke=Stroke(width=0)))
        if net == "NC":
            no_connects.append(NoConnect(position=Position(x0, y)))
        else:
            labels.append(LocalLabel(text=net, position=Position(x1, y, 0),
                                     effects=eff(1.27, "right")))
            expected[(ref, pn)] = net

    for i, (pn, net) in enumerate(right):
        y = Y + 10 + i * SPACING
        x0 = X + W + PIN_LEN
        x1 = x0 + STUB
        if STUB > 0:
            graphical_items.append(Connection(type="wire", points=[Position(x0, y), Position(x1, y)],
                                              stroke=Stroke(width=0)))
        if net == "NC":
            no_connects.append(NoConnect(position=Position(x0, y)))
        else:
            labels.append(LocalLabel(text=net, position=Position(x1, y, 0),
                                     effects=eff(1.27, "left")))
            expected[(ref, pn)] = net

# ---------------------------------------------------------------- write
sch = Schematic(
    version=20231120,
    generator="kiutils",
    uuid=uid(),
    paper=PageSettings(paperSize="A0"),
    titleBlock=TitleBlock(
        title="DZZH 载板 — 天猛星 MSPM0G3507 + 全模块",
        revision="A",
        company="DZZH",
        comments={"1": "所有外设均为插件模块；网络用同名 label 连接。"}),
    libSymbols=lib_symbols,
    schematicSymbols=schematic_symbols,
    graphicalItems=graphical_items,
    labels=labels,
    noConnects=no_connects,
    texts=[
        SchText(text="DZZH 载板 — 天猛星 MSPM0G3507 + 全模块",
                position=Position(60, 22, 0), effects=eff(2.5)),
        SchText(text="同名 label 即同一网络；所有外设均为插件模块。落板前用万用表核对天猛星排针顺序。",
                position=Position(60, 28, 0), effects=eff(1.5)),
    ],
)
OUT.parent.mkdir(exist_ok=True)
OUT.write_text(sch.to_sexpr(), encoding="utf-8")   # KiCad requires UTF-8

# ---------------------------------------------------------------- validate
back = Schematic.from_file(str(OUT), encoding="utf-8")
errs = []
if len(back.libSymbols) != len(items):
    errs.append(f"libSymbols {len(back.libSymbols)} != {len(items)}")
if len(back.schematicSymbols) != len(items):
    errs.append(f"symbols {len(back.schematicSymbols)} != {len(items)}")
if len(back.labels) != len(labels):
    errs.append(f"labels {len(back.labels)} != {len(labels)}")

by_ref = {}
for s in back.schematicSymbols:
    by_ref.setdefault(s.entryName, 0)
    by_ref[s.entryName] += 1
for ref, _v, _f, _p in items:
    if by_ref.get(ref, 0) != 1:
        errs.append(f"{ref}: instance count {by_ref.get(ref, 0)}")

# LCEDA importer requirements
for s in back.libSymbols:
    if len(s.units) != 1:
        errs.append(f"{s.entryName}: expected 1 unit, got {len(s.units)}")
        continue
    for p in s.units[0].pins:
        if p.position.angle is None:
            errs.append(f"{s.entryName}.{p.number}: pin angle missing")
for s in back.schematicSymbols:
    if not s.libName:
        errs.append(f"{s.entryName}: lib_name missing")
    if s.position.angle is None:
        errs.append(f"{s.entryName}: instance angle missing")

# geometry: sheet bounds + block overlap
pw, ph = 1189, 841   # A0 landscape, mm
boxes = []
for ref, _v, _f, pins in items:
    X, Y, W = LAYOUT[ref]
    H = block_height(pins)
    boxes.append((ref, X, Y, X + W, Y + H))
    if X < 0 or Y < 0 or X + W > pw or Y + H > ph:
        errs.append(f"{ref}: body outside sheet")
for i in range(len(boxes)):
    for j in range(i + 1, len(boxes)):
        a, b = boxes[i], boxes[j]
        if a[1] < b[3] and b[1] < a[3] and a[2] < b[4] and b[2] < a[4]:
            errs.append(f"body overlap: {a[0]} / {b[0]}")
for lb in labels:
    if not (0 <= lb.position.X <= pw and 0 <= lb.position.Y <= ph):
        errs.append(f"label outside sheet: {lb.text}")

# estimated label bounding boxes (ASCII net names; 0.85 mm/char at 1.27 mm)
def lbox(lb):
    w = max(1.5, len(lb.text) * 0.85)
    j = lb.effects.justify.horizontally if lb.effects and lb.effects.justify else None
    x0 = lb.position.X - w if j == "right" else lb.position.X
    return (x0, lb.position.Y - 0.7, x0 + w, lb.position.Y + 0.7)

lb_boxes = [lbox(lb) for lb in labels]
for i in range(len(lb_boxes)):
    for j in range(i + 1, len(lb_boxes)):
        a, b = lb_boxes[i], lb_boxes[j]
        if (labels[i].text != labels[j].text
                and a[0] < b[2] and b[0] < a[2]
                and a[1] < b[3] and b[1] < a[3]):
            errs.append(f"label overlap: {labels[i].text} / {labels[j].text}")

report = [
    "KiCad schematic validation",
    f"  file      : {OUT}",
    f"  components: {len(items)}",
    f"  symbols   : {len(schematic_symbols)}",
    f"  nets      : {len(set(expected.values()))}",
    f"  labels    : {len(labels)}",
    f"  NC pins   : {len(no_connects)}",
    f"  result    : {'PASS' if not errs else 'FAIL'}",
]
if errs:
    report += ["  " + e for e in errs]
Path(str(OUT).replace(".kicad_sch", "_validation.txt")).write_text(
    "\n".join(report) + "\n", encoding="utf-8")
print("\n".join(report))
sys.exit(1 if errs else 0)
