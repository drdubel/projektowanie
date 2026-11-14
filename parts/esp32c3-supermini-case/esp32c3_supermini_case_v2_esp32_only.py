"""ESP32-C3 SuperMini Plus (red, V2) - two-part case with the GPIO header pins on TOP.

Run inside FreeCAD:
    exec(open("/Users/antek/projektowanie/parts/esp32c3-supermini-case/esp32c3_supermini_case_v2_esp32_only.py").read())

Frame (mm): board corner at the antenna end / left pin row = (0, 0); +Y runs toward the USB-C,
+X across the 18 mm width, +Z up; the case floor underside is z=0. Edit a parameter and re-run to rebuild.

Header orientation assumed: 2.54 mm male headers soldered with the plastic housing ON the top face of the
PCB and the long pins pointing up through two slots in the lid; the 3 mm tails go down into floor slots.
"""

import FreeCAD as App
import Part
from FreeCAD import Vector as V

DOC_NAME = "ESP32C3_SuperMini_Case_v2_esp32_only"
SAVE_PATH = "/Users/antek/projektowanie/parts/esp32c3-supermini-case/ESP32C3_SuperMini_Case_v2_esp32_only.FCStd"
EXPLODE_LID = 18.0  # mm to lift the lid for display only (0 = assembled)
SAVE = True  # save SAVE_PATH after building (only if every check passes)

# ------------------------------------------------------------------ board (red SuperMini Plus V2)
# PCB size / row spacing: manufacturer datasheet. Component positions: pixel-measured on the to-scale
# ESPboards illustration (58.9 px/mm). USB-C: standard 16-pin receptacle. PCB thickness is NOT published.
PCB_W, PCB_L, PCB_R = 18.0, 22.52, 1.0
PCB_T = 1.2  # thickness used for the reference model
PCB_T_MIN, PCB_T_MAX = (
    1.0,
    1.6,
)  # the case is sized to work for any thickness in this range
PIN_COL_X = (1.38, PCB_W - 1.38)  # 15.24 mm row-to-row
PIN_COUNT, PIN_PITCH = 8, 2.54
PIN_Y_FIRST = PCB_L - 1.58  # USB-end pin; last pin is 3.16 from the antenna end
PIN_HOLE_D = 1.0
USB_CX, USB_W, USB_LEN, USB_H, USB_OVERHANG = PCB_W / 2, 8.94, 7.3, 3.26, 1.87
BUTTONS = {"BOOT": (5.96, 13.96), "RST": (12.05, 13.96)}  # plunger centres
BTN_W, BTN_D, BTN_H = 4.0, 3.0, 2.0  # SMD tact switch "3*4*2" (schematic)
HDR_W, HDR_H, HDR_TAIL, HDR_LONG = (
    2.54,
    2.5,
    3.0,
    6.0,
)  # 2.54 mm male header: housing, tail, long pin

# ------------------------------------------------------------------ case
WALL, FLOOR_T, LID_T = (
    1.2,
    2.0,
    1.2,
)  # floor is thick enough to swallow the 3 mm header tails
SIDE_CLR = 0.2  # PCB to side walls
GAP_ANT = 0.3  # PCB end to inner wall, antenna end
GAP_USB = 0.1  # PCB end to inner wall, USB end (keeps the USB shell proud of the wall)
CAV_R = 0.6  # inner corner radius
HEADROOM = 0.3  # USB-C top to lid underside, at PCB_T_MAX
USB_NOTCH_W = 10.0
BTN_HOLE_D = 3.0
SLOT_CLR = 0.25  # clearance around the header housing in the lid slots
FLOOR_SLOT_PAD = 1.5  # floor relief slots: pin centres +/- this along Y
FLOOR_SLOT_X = 1.35  # ... reaching this far inboard of the pin centre line
SKIRT_CLR, SKIRT_T, SKIRT_D = (
    0.1,
    0.8,
    2.0,
)  # lid skirt: gap to the base, thickness, depth
BUMP_R, BUMP_OUT, BUMP_LEN = 0.4, 0.25, 6.0  # snap bumps on the inside of the lid skirt
GROOVE_R, GROOVE_IN, GROOVE_LEN = (
    0.5,
    0.30,
    6.4,
)  # matching grooves on the outside of the base
RIB_X, RIB_Y0, RIB_Y1 = 0.4, 12.9, 15.3  # lid hold-down rib between the buttons
USB_CUT_W = (
    11.0  # lid edge / skirt cut back in front of the USB-C so cable plugs can seat
)

# ------------------------------------------------------------------ derived
Z_PCB0, Z_PCB1 = FLOOR_T, FLOOR_T + PCB_T
CX0, CX1, CY0, CY1 = -SIDE_CLR, PCB_W + SIDE_CLR, -GAP_ANT, PCB_L + GAP_USB
OX0, OX1, OY0, OY1 = CX0 - WALL, CX1 + WALL, CY0 - WALL, CY1 + WALL
OUT_R = CAV_R + WALL
RIM_Z = FLOOR_T + PCB_T_MAX + USB_H + HEADROOM
TOP_Z = RIM_Z + LID_T
OVH = SKIRT_CLR + SKIRT_T  # lid overhang beyond the base
HDR_Y1 = PIN_Y_FIRST + PIN_PITCH / 2
HDR_Y0 = PIN_Y_FIRST - (PIN_COUNT - 0.5) * PIN_PITCH
PIN_YS = [PIN_Y_FIRST - i * PIN_PITCH for i in range(PIN_COUNT)]
LID_SLOT_X = (
    PIN_COL_X[0] + HDR_W / 2 + SLOT_CLR,
    PIN_COL_X[1] - HDR_W / 2 - SLOT_CLR,
)  # inner edges of the two lid slots
LID_SLOT_Y0, LID_SLOT_Y1 = HDR_Y0 - 0.5, HDR_Y1 + 0.24


# ------------------------------------------------------------------ helpers
def box(x0, y0, z0, x1, y1, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(x, y, z, r, h, axis=(0, 0, 1)):
    return Part.makeCylinder(r, h, V(x, y, z), V(*axis))


def fillet_where(shape, r, pred):
    edges = [e for e in shape.Edges if pred(e.Vertexes[0].Point, e.Vertexes[-1].Point)]
    return shape.makeFillet(r, edges)


def rbox(x0, y0, z0, x1, y1, z1, r):
    """Box with its four vertical edges rounded."""
    return fillet_where(
        box(x0, y0, z0, x1, y1, z1),
        r,
        lambda a, b: abs(a.x - b.x) < 1e-7 and abs(a.y - b.y) < 1e-7,
    )


def solid(shape):
    """Unwrap a single-solid compound into a plain Solid."""
    return shape.Solids[0] if len(shape.Solids) == 1 else shape


def fuse_all(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.removeSplitter()


# ------------------------------------------------------------------ board model (reference only)
def build_board(pcb_t=PCB_T, with_headers=True):
    parts = {}  # key -> (label, shape, rgb)
    z0 = FLOOR_T  # PCB rests on the floor
    z1 = z0 + pcb_t

    pcb = rbox(0, 0, z0, PCB_W, PCB_L, z1, PCB_R)
    for x in PIN_COL_X:
        for y in PIN_YS:
            pcb = pcb.cut(cyl(x, y, z0 - 0.1, PIN_HOLE_D / 2, pcb_t + 0.2))
    parts["PCB"] = (
        "PCB red 22.52x18 (thickness %.1f)" % pcb_t,
        pcb,
        (0.78, 0.15, 0.10),
    )

    uy1 = PCB_L + USB_OVERHANG
    usb = box(
        USB_CX - USB_W / 2, uy1 - USB_LEN, z1, USB_CX + USB_W / 2, uy1, z1 + USB_H
    )
    usb = fillet_where(
        usb, 1.0, lambda a, b: abs(a.x - b.x) < 1e-7 and abs(a.z - b.z) < 1e-7
    )
    parts["USB_C"] = ("USB-C receptacle 8.94x3.26", usb, (0.80, 0.80, 0.83))

    for name, (bx, by) in BUTTONS.items():
        body = box(
            bx - BTN_W / 2,
            by - BTN_D / 2,
            z1,
            bx + BTN_W / 2,
            by + BTN_D / 2,
            z1 + BTN_H - 0.4,
        )
        stem = cyl(bx, by, z1 + BTN_H - 0.4, 1.0, 0.4)
        parts["Btn_" + name] = (
            "%s button 3x4x2" % name,
            body.fuse(stem),
            (0.55, 0.56, 0.60),
        )

    cx, cy = 9.03, 8.59
    chip = box(cx - 2.5, cy - 2.5, z1, cx + 2.5, cy + 2.5, z1 + 0.9).copy()
    chip.rotate(V(cx, cy, 0), V(0, 0, 1), 45)
    parts["Chip"] = ("ESP32-C3FN4 QFN32 5x5", chip, (0.10, 0.10, 0.11))

    def smd(key, label, cx, cy, w, d, h, rgb):
        parts[key] = (
            label,
            box(cx - w / 2, cy - d / 2, z1, cx + w / 2, cy + d / 2, z1 + h),
            rgb,
        )

    smd(
        "LED_RGB",
        "WS2812 RGB LED (GPIO8)",
        14.26,
        8.31,
        2.0,
        2.0,
        0.84,
        (0.95, 0.95, 0.95),
    )
    smd(
        "UFL", "U.FL antenna connector", 13.82, 2.61, 3.0, 3.0, 1.25, (0.80, 0.65, 0.20)
    )
    smd(
        "Antenna", "Ceramic chip antenna", 7.15, 1.36, 6.5, 2.2, 0.8, (0.92, 0.80, 0.76)
    )
    smd(
        "Regulator", "Regulator SOT23-5", 4.10, 10.40, 1.6, 2.9, 1.1, (0.20, 0.20, 0.22)
    )
    smd("Crystal", "Crystal 3225", 4.27, 5.77, 2.5, 3.2, 0.8, (0.70, 0.70, 0.72))
    smd(
        "Diode", "Schottky diode SOD323", 3.20, 21.04, 0.9, 2.1, 0.9, (0.15, 0.15, 0.16)
    )

    if (
        with_headers
    ):  # 2.54 mm male headers: housing on the PCB top, long pins up, 3 mm tails down
        for tag, x in zip(("L", "R"), PIN_COL_X):
            parts["Hdr_%s_housing" % tag] = (
                "Header %s housing 1x8" % tag,
                box(x - HDR_W / 2, HDR_Y0, z1, x + HDR_W / 2, HDR_Y1, z1 + HDR_H),
                (0.10, 0.10, 0.10),
            )
            pins = [
                box(
                    x - 0.32,
                    y - 0.32,
                    z1 - HDR_TAIL,
                    x + 0.32,
                    y + 0.32,
                    z1 + HDR_H + HDR_LONG,
                )
                for y in PIN_YS
            ]
            parts["Hdr_%s_pins" % tag] = (
                "Header %s pins" % tag,
                Part.makeCompound(pins),
                (0.85, 0.70, 0.25),
            )
    return parts


# ------------------------------------------------------------------ case model
def build_case():
    ymid = (CY0 + CY1) / 2
    bump_z = RIM_Z - 1.0

    # ---- base: tray, floor relief slots for the header tails, USB-C notch, outside snap grooves
    outer = rbox(OX0, OY0, 0, OX1, OY1, RIM_Z, OUT_R)
    cavity = rbox(CX0, CY0, FLOOR_T, CX1, CY1, RIM_Z + 1, CAV_R)

    fy0, fy1 = PIN_YS[-1] - FLOOR_SLOT_PAD, PIN_YS[0] + FLOOR_SLOT_PAD
    prism_dn = rbox(
        CX0, CY0, -1, CX1, CY1, FLOOR_T + 0.5, CAV_R
    )  # slots follow the cavity outline
    fx_l, fx_r = PIN_COL_X[0] + FLOOR_SLOT_X, PIN_COL_X[1] - FLOOR_SLOT_X
    slots = prism_dn.common(box(CX0 - 1, fy0, -1, fx_l, fy1, FLOOR_T + 0.5)).fuse(
        prism_dn.common(box(fx_r, fy0, -1, CX1 + 1, fy1, FLOOR_T + 0.5))
    )
    base = outer.cut(cavity.fuse(slots))

    nz0 = (
        FLOOR_T + PCB_T_MIN - 0.1
    )  # notch floor sits below the connector even on a 1.0 mm PCB
    notch = box(
        USB_CX - USB_NOTCH_W / 2,
        PCB_L,
        nz0,
        USB_CX + USB_NOTCH_W / 2,
        OY1 + 1,
        RIM_Z + 1,
    )
    notch = fillet_where(
        notch,
        1.0,
        lambda a, b: (
            abs(a.x - b.x) < 1e-7 and abs(a.z - b.z) < 1e-7 and abs(a.z - nz0) < 1e-7
        ),
    )
    base = base.cut(notch)

    for xo, s in ((OX0, +1), (OX1, -1)):  # s points from the outer face into the wall
        base = base.cut(
            cyl(
                xo + s * (GROOVE_IN - GROOVE_R),
                ymid - GROOVE_LEN / 2,
                bump_z,
                GROOVE_R,
                GROOVE_LEN,
                (0, 1, 0),
            )
        )
    base = solid(base.removeSplitter())

    # ---- lid: plate + outside skirt, pin slots, button holes, hold-down rib, snap bumps
    LX0, LX1, LY0, LY1 = OX0 - OVH, OX1 + OVH, OY0 - OVH, OY1 + OVH
    plate = rbox(LX0, LY0, RIM_Z, LX1, LY1, TOP_Z, OUT_R + OVH)
    skirt = rbox(LX0, LY0, RIM_Z - SKIRT_D, LX1, LY1, RIM_Z, OUT_R + OVH).cut(
        rbox(
            OX0 - SKIRT_CLR,
            OY0 - SKIRT_CLR,
            RIM_Z - SKIRT_D - 1,
            OX1 + SKIRT_CLR,
            OY1 + SKIRT_CLR,
            RIM_Z + 1,
            OUT_R + SKIRT_CLR,
        )
    )
    rib = box(
        USB_CX - RIB_X,
        RIB_Y0,
        FLOOR_T + PCB_T_MAX + 0.1,
        USB_CX + RIB_X,
        RIB_Y1,
        RIM_Z + 0.01,
    )
    bumps = [
        cyl(
            OX0 - SKIRT_CLR + (BUMP_OUT - BUMP_R),
            ymid - BUMP_LEN / 2,
            bump_z,
            BUMP_R,
            BUMP_LEN,
            (0, 1, 0),
        ),
        cyl(
            OX1 + SKIRT_CLR - (BUMP_OUT - BUMP_R),
            ymid - BUMP_LEN / 2,
            bump_z,
            BUMP_R,
            BUMP_LEN,
            (0, 1, 0),
        ),
    ]
    lid = fuse_all([plate, skirt, rib] + bumps)

    # cut the lid edge and skirt back in front of the USB-C so cable overmoulds can seat
    lid = lid.cut(
        box(
            USB_CX - USB_CUT_W / 2,
            OY1,
            RIM_Z - SKIRT_D - 1,
            USB_CX + USB_CUT_W / 2,
            LY1 + 1,
            TOP_Z + 1,
        )
    )
    # GPIO slots over the header housings (clipped to the cavity outline)
    prism_up = rbox(CX0, CY0, RIM_Z - 1, CX1, CY1, TOP_Z + 1, CAV_R)
    lid_slots = prism_up.common(
        box(CX0 - 1, LID_SLOT_Y0, RIM_Z - 1, LID_SLOT_X[0], LID_SLOT_Y1, TOP_Z + 1)
    ).fuse(
        prism_up.common(
            box(LID_SLOT_X[1], LID_SLOT_Y0, RIM_Z - 1, CX1 + 1, LID_SLOT_Y1, TOP_Z + 1)
        )
    )
    lid = lid.cut(lid_slots)
    for bx, by in BUTTONS.values():  # BOOT / RST access holes
        lid = lid.cut(cyl(bx, by, RIM_Z - 1.0, BTN_HOLE_D / 2, LID_T + 2.0))
    lid = solid(lid.removeSplitter())
    return base, lid


# ------------------------------------------------------------------ document
SPECS = [
    ("Board", "ESP32-C3 SuperMini Plus (V2), red PCB", "ESPboards, Mischianti"),
    (
        "MCU",
        "ESP32-C3FN4: RISC-V 160 MHz, 4 MB flash, 400 KB SRAM",
        "manufacturer datasheet",
    ),
    (
        "Wireless",
        "Wi-Fi 802.11 b/g/n 2.4 GHz + BLE 5; ceramic antenna AND U.FL connector",
        "datasheet, retailer",
    ),
    (
        "PCB size",
        "22.52 x 18.00 mm; USB-C shell overhangs 1.87 mm beyond the PCB edge",
        "datasheet; overhang measured",
    ),
    (
        "PCB thickness",
        "NOT published (one listing says 1.6). Modelled 1.2; case verified for 1.0-1.6 mm",
        "assumption - measure yours",
    ),
    (
        "Pin rows",
        "2 x 8, 2.54 mm pitch, 15.24 mm row spacing, through-hole + castellated",
        "datasheet",
    ),
    (
        "Pin positions",
        "x = 1.38 / 16.62; first pin 1.58 mm from USB end, last 3.16 mm from antenna end",
        "measured on illustration",
    ),
    (
        "USB-C",
        "16-pin SMD receptacle 8.94 x 7.3 x 3.26 mm, centred on the width (x = 9.0)",
        "standard part + illustration",
    ),
    (
        "BOOT button",
        "SMD tact 3x4x2 mm, plunger centre x = 5.96, y = 13.96; GPIO9",
        "schematic + measured",
    ),
    (
        "RST button",
        "SMD tact 3x4x2 mm, plunger centre x = 12.05, y = 13.96; EN",
        "schematic + measured",
    ),
    (
        "LEDs",
        "red power; blue user LED on GPIO8; WS2812 RGB on GPIO8 (centre x = 14.26, y = 8.31)",
        "datasheet, retailer",
    ),
    (
        "U.FL",
        "3.0 x 3.0 x 1.25 mm, centre x = 13.82, y = 2.61 (antenna end)",
        "measured on illustration",
    ),
    (
        "Power",
        "5 V via USB-C or 5V pin (3.3-6 V external); 3.3 V logic, not 5 V tolerant",
        "datasheet",
    ),
    ("Underside", "flat - single-sided SMD, no components", "datasheet"),
    (
        "Case: GPIO",
        "2.54 mm headers soldered housing-up, long pins through two lid slots; tails drop into floor slots",
        "design choice",
    ),
]


def add_specs(doc):
    sh = doc.addObject("Spreadsheet::Sheet", "Board_Specs")
    sh.Label = "Board specs (documentation)"
    for col, head in zip("ABC", ("Property", "Value", "Source / confidence")):
        sh.set(col + "1", "'" + head)
    for i, row in enumerate(SPECS, start=2):
        for col, val in zip("ABC", row):
            sh.set("%s%d" % (col, i), "'" + val)
    sh.setColumnWidth("A", 110)
    sh.setColumnWidth("B", 560)
    sh.setColumnWidth("C", 210)
    return sh


def new_part(doc, name, label, shape, rgb, group=None, transparency=0):
    o = doc.addObject("Part::Feature", name)
    o.Label = label
    o.Shape = shape
    if group is not None:
        group.addObject(o)
    if App.GuiUp:
        o.ViewObject.ShapeColor = rgb
        o.ViewObject.Transparency = transparency
    return o


def build():
    if DOC_NAME in App.listDocuments():
        App.closeDocument(DOC_NAME)
    doc = App.newDocument(DOC_NAME)

    base_shape, lid_shape = build_case()
    base = new_part(
        doc, "Case_Base", "Case base (bottom)", base_shape, (0.82, 0.84, 0.88)
    )
    lid = new_part(
        doc,
        "Case_Lid",
        "Case lid (top)",
        lid_shape,
        (0.25, 0.55, 0.85),
        transparency=20,
    )
    lid.Placement.Base = V(0, 0, EXPLODE_LID)

    grp = doc.addObject("App::DocumentObjectGroup", "Board_Reference")
    grp.Label = "ESP32-C3 SuperMini Plus (red) - reference model"
    objs = {}
    for key, (label, shape, rgb) in build_board().items():
        objs[key] = new_part(doc, "Board_" + key, label, shape, rgb, group=grp)
    add_specs(doc)
    doc.recompute()
    return doc, base, lid, objs


def verify(doc, board_objs):
    """Numeric fit check: overlaps, clearances, PCB-thickness sweep, and cut-out alignment."""
    base = doc.Case_Base.Shape.copy()
    lid = doc.Case_Lid.Shape.copy()
    lid.translate(V(0, 0, -doc.Case_Lid.Placement.Base.z))  # undo display offset
    fails = []

    def check(ok, msg):
        if not ok:
            fails.append(msg)

    check(base.common(lid).Volume < 1e-6, "base and lid overlap")
    print(
        "%-22s %9s %9s %10s %10s"
        % ("board part", "ovl base", "ovl lid", "gap base", "gap lid")
    )
    for key, o in board_objs.items():
        s = o.Shape
        ob, ol = base.common(s).Volume, lid.common(s).Volume
        db, dl = s.distToShape(base)[0], s.distToShape(lid)[0]
        print("%-22s %9.4f %9.4f %10.3f %10.3f" % (key, ob, ol, db, dl))
        check(ob < 1e-6, "%s intersects base" % key)
        check(ol < 1e-6, "%s intersects lid" % key)

    print(
        "PCB thickness sweep (case sized for %.1f-%.1f mm), fixed case, board rebuilt per thickness:"
        % (PCB_T_MIN, PCB_T_MAX)
    )
    for t in (PCB_T_MIN, 1.2, 1.4, PCB_T_MAX):
        parts = build_board(pcb_t=t)
        ovl = sum(
            base.common(s).Volume + lid.common(s).Volume for _, s, _ in parts.values()
        )
        usb = parts["USB_C"][1]
        btn = min(
            parts["Btn_BOOT"][1].distToShape(lid)[0],
            parts["Btn_RST"][1].distToShape(lid)[0],
        )
        print(
            "  T=%.1f  overlap=%.4f mm3 | USB gap: lid %.2f base %.2f | button-to-lid %.2f"
            % (t, ovl, usb.distToShape(lid)[0], usb.distToShape(base)[0], btn)
        )
        check(ovl < 1e-6, "interference at PCB thickness %.1f" % t)

    mid_lid = RIM_Z + LID_T / 2
    for name, (bx, by) in BUTTONS.items():
        check(
            not lid.isInside(V(bx, by, mid_lid), 1e-6, False), "%s hole missing" % name
        )
        check(
            lid.isInside(V(bx + BTN_HOLE_D / 2 + 0.3, by, mid_lid), 1e-6, False),
            "%s hole too big" % name,
        )
    zc = Z_PCB1 + USB_H / 2
    check(
        not base.isInside(V(USB_CX, (OY1 + PCB_L) / 2 + 0.3, zc), 1e-6, False),
        "USB notch missing",
    )
    check(
        base.isInside(
            V(USB_CX + USB_NOTCH_W / 2 + 0.5, OY1 - WALL / 2, zc), 1e-6, False
        ),
        "wall beside USB missing",
    )
    check(
        not lid.isInside(V(USB_CX, OY1 + 0.4, mid_lid), 1e-6, False),
        "lid not cut back at USB",
    )
    check(
        lid.isInside(V(USB_CX, OY1 - 0.4, mid_lid), 1e-6, False),
        "lid plate missing over USB-end wall",
    )
    check(
        lid.isInside(V(USB_CX, 11, mid_lid), 1e-6, False),
        "lid plate missing over the board centre",
    )
    check(
        base.isInside(V(USB_CX, 11, FLOOR_T / 2), 1e-6, False),
        "floor strip under PCB missing",
    )
    for x in PIN_COL_X:
        for y in PIN_YS:
            check(
                not lid.isInside(V(x, y, mid_lid), 1e-6, False),
                "no lid slot over pin %.2f,%.2f" % (x, y),
            )
            check(
                not base.isInside(V(x, y, FLOOR_T / 2), 1e-6, False),
                "no floor relief under pin %.2f,%.2f" % (x, y),
            )
    ymid, bump_z = (CY0 + CY1) / 2, RIM_Z - 1.0
    for xo, xs, s in ((OX0, OX0 - SKIRT_CLR, +1), (OX1, OX1 + SKIRT_CLR, -1)):
        tip = V(xs + s * (BUMP_OUT - 0.05), ymid, bump_z)  # just inside the bump tip
        deep = V(
            xo + s * (GROOVE_IN - 0.05), ymid, bump_z
        )  # just inside the groove floor
        check(lid.isInside(tip, 1e-6, False), "snap bump missing (side %+d)" % s)
        check(
            not base.isInside(tip, 1e-6, False),
            "bump collides with wall (side %+d)" % s,
        )
        check(not base.isInside(deep, 1e-6, False), "groove missing (side %+d)" % s)
    check(
        base.ShapeType == "Solid" and lid.ShapeType == "Solid",
        "case parts are not single solids",
    )

    lb, ll = base.BoundBox, lid.BoundBox
    print(
        "base %.1f x %.1f x %.1f mm (%.0f mm3) | lid %.1f x %.1f x %.1f mm (%.0f mm3) | assembled height %.2f mm"
        % (
            lb.XLength,
            lb.YLength,
            lb.ZLength,
            base.Volume,
            ll.XLength,
            ll.YLength,
            ll.ZLength,
            lid.Volume,
            TOP_Z,
        )
    )
    print(
        "RESULT:", "ALL CHECKS PASSED" if not fails else "FAILED -> " + "; ".join(fails)
    )
    return fails


doc, base, lid, board_objs = build()
fails = verify(doc, board_objs)
if App.GuiUp:
    import FreeCADGui as Gui

    Gui.ActiveDocument = Gui.getDocument(DOC_NAME)
    Gui.ActiveDocument.ActiveView.viewIsometric()
    Gui.SendMsgToActiveView("ViewFit")
if SAVE and not fails:
    doc.saveAs(SAVE_PATH)
    print("saved:", SAVE_PATH)
