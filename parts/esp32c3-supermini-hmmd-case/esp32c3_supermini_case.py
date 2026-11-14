"""ESP32-C3 SuperMini Plus (red, V2) + Waveshare HMMD mmWave sensor - one two-part case, header pins on TOP.

Run inside FreeCAD:
    exec(open("/Users/antek/projektowanie/parts/esp32c3-supermini-hmmd-case/esp32c3_supermini_case.py").read())

Frame (mm): ESP32 PCB corner at the antenna end / left pin row = (0, 0); +Y runs toward the USB-C, +X across the
18 mm width (the HMMD bay sits on the +X side), +Z up; the case underside is z=0. Edit a parameter and re-run.

Both boards sit component-side up with their header pins pointing UP through slots in the lid.
 * ESP32: 2.54 mm headers soldered housing-up. The 3 mm tails drop into BLIND pockets in a solid platform, so the
   underside of the case is completely closed.
 * HMMD: its header is on the BACK, the side opposite its antennas, and the pins point away from the antennas. With the
   pins up, the antennas therefore look DOWN through a thin floor of the base (the radome, designed to Waveshare's
   radome guide). Mount the case with the base facing the area to be sensed (e.g. lid against the ceiling).
 * External (U.FL) antenna: a 3 mm cable notch in the antenna-end wall, closed by the lid (ANT_NOTCH = False seals it).
 * Print the base floor-down on a SMOOTH bed (a textured sheet roughens the radome) and the lid top-down.
"""

import FreeCAD as App
import Part
from FreeCAD import Vector as V

DOC_NAME = "ESP32C3_SuperMini_Case"
SAVE_PATH = "/Users/antek/projektowanie/parts/esp32c3-supermini-hmmd-case/ESP32C3_SuperMini_Case.FCStd"
EXPLODE_LID = 22.0  # mm to lift the lid for display only (0 = assembled)
SAVE = True  # save SAVE_PATH after building (only if every check passes)

# ------------------------------------------------------------------ ESP32-C3 SuperMini Plus (red V2)
# PCB size / row spacing: manufacturer datasheet. Component positions: pixel-measured on the to-scale ESPboards
# illustration (58.9 px/mm). USB-C: standard 16-pin receptacle. PCB thickness is NOT published.
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

# ------------------------------------------------------------------ Waveshare HMMD mmWave sensor (S3KM1110, 24 GHz)
# Outline, pin line, patches and parts: scaled from Waveshare's dimensioned drawing (17.4 px/mm).
# "Plan" coordinates (u, v) = the view from the BACK with the header at the left: u from the header-side edge,
# v from the bottom edge. PCB thickness is NOT published -> assumed; the case is verified over HM_T_MIN..HM_T_MAX.
HM_W, HM_L, HM_R = 20.0, 20.0, 1.4
HM_T = 1.0
HM_T_MIN, HM_T_MAX = 0.8, 1.6
HM_PIN_X = 1.5  # J2 pin line, from the header-side edge
HM_PIN_V = [
    18.38 - 2.54 * i for i in range(5)
]  # first pin 1.62 mm from the corner, 2.54 mm pitch
HM_PIN_NAMES = ("OT2", "RX", "TX", "GND", "3V3")  # top to bottom in plan view
HM_PATCHES = {
    "A": (11.66, 17.56, 15.27, 17.73),
    "B": (2.43, 8.32, 2.17, 4.65),
}  # antenna patches (u0, u1, v0, v1)
HM_KEEPOUT = 1.4  # no case material within this distance (in plan) of an antenna patch
# (centre u, centre v, width, depth, height) of parts on the FRONT face (hang toward the radome) ...
HM_FRONT_PARTS = {
    "chip": (9.96, 9.0, 5.0, 5.0, 0.9),
    "L2": (16.95, 8.0, 2.7, 2.4, 1.2),
    "U3": (15.5, 4.4, 2.6, 4.0, 0.8),
    "R14C15": (18.0, 5.5, 1.6, 1.6, 0.6),
    "R15": (16.1, 1.45, 1.6, 0.7, 0.5),
    "R16": (13.8, 1.45, 1.8, 0.7, 0.5),
}
# ... and on the BACK face (the lid side)
HM_BACK_PARTS = {
    "U4": (9.5, 15.35, 3.0, 3.1, 0.9),
    "Y1": (14.95, 8.55, 3.2, 2.7, 0.8),
    "L1": (14.95, 4.3, 3.0, 3.0, 1.6),
    "C4": (14.95, 1.9, 2.3, 1.2, 1.0),
}
# Ledges: strips under the front face (u0, u1, v0, v1), kept clear of the antennas and of the parts above
HM_REACH = 0.8  # how far a ledge reaches in under the PCB edge
HM_RAILS = {
    "R": (HM_W - HM_REACH, HM_W, 2.0, 12.5),
    "T": (3.6, 10.0, HM_L - HM_REACH, HM_L),
    "L": (0.0, HM_REACH, 1.8, 6.5),
}
HM_POSTS = (
    (18.2, 6.5),
    (18.2, 2.4),
)  # lid hold-down posts, on bare solder mask on the back (u, v)
HM_POST = 1.4  # post size (square)

# ------------------------------------------------------------------ radome (Waveshare "radome design guide", 24.125 GHz)
RADOME_T = (
    0.8  # floor under the antennas: <= 1/8 wavelength in the plastic for eps_r <= 3.7
)
HM_H = 6.2  # antenna face to the inside of the radome: half a wavelength in air (+-1.2 mm tolerance)
HM_CLR = 0.2  # PCB to pocket walls
HM_HEADROOM = 0.2  # header housing top to the lid underside, at HM_T_MAX

# ------------------------------------------------------------------ case
WALL, LID_T = 1.2, 1.2
SIDE_CLR = 0.2  # ESP32 PCB to side walls
GAP_ANT = 0.3  # ESP32 PCB end to inner wall, antenna end
GAP_USB = (
    0.1  # ESP32 PCB end to inner wall, USB end (keeps the USB shell proud of the wall)
)
CAV_R = 0.6  # inner corner radius
HEADROOM = 0.3  # ESP32 USB-C top to lid underside, at PCB_T_MAX
USB_NOTCH_W = 10.0
BTN_HOLE_D = 3.0
SLOT_CLR = 0.25  # clearance around a header housing in the lid slots
POCKET_CLR = 0.3  # clearance under the ESP32 header tails (at PCB_T_MIN)
POCKET_PAD = 1.5  # tail pockets: pin centres +/- this along Y
POCKET_X = 1.35  # ... reaching this far inboard of the pin centre line
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
RIB_X, RIB_Y0, RIB_Y1 = 0.4, 12.9, 15.3  # ESP32 hold-down rib between the buttons
USB_CUT_W = (
    11.0  # lid edge / skirt cut back in front of the USB-C so cable plugs can seat
)

# ------------------------------------------------------------------ external (U.FL) antenna cable exit
# The red SuperMini Plus has a U.FL socket at the antenna end. A mated plug stacks 1.25 mm on the socket (mated height
# 2.5 mm) and its pigtail leaves sideways, so it exits through a notch in the antenna-end wall (open at the top, closed
# by the lid plate); the lid skirt is opened to match.
ANT_NOTCH = True  # cut the exit; False = sealed case
UFL_X, UFL_Y, UFL_W, UFL_H = (
    13.82,
    2.61,
    3.0,
    1.25,
)  # socket centre, size, height (measured)
UFL_PLUG_W, UFL_PLUG_H = 2.6, 1.25  # mated plug body, stacked on the socket
UFL_CABLE_D, UFL_CABLE_Z = (
    1.13,
    1.9,
)  # pigtail coax (1.13 mm is the common one); axis height above the PCB top
ANT_NOTCH_W = 3.0  # fits coax up to ~1.4 mm with sideways slack
ANT_NOTCH_MARGIN = (
    0.4  # notch floor this far below the lowest cable position (at PCB_T_MIN)
)

# ------------------------------------------------------------------ derived
HM_FRONT_Z = RADOME_T + HM_H  # the HMMD antenna face rests on the ledges at this height
RIM_Z = (
    HM_FRONT_Z + HM_T_MAX + HDR_H + HM_HEADROOM
)  # parting plane; set by the HMMD stack
TOP_Z = RIM_Z + LID_T
Z_PCB0 = RIM_Z - (PCB_T_MAX + USB_H + HEADROOM)  # solid platform under the ESP32
Z_PCB1 = Z_PCB0 + PCB_T
ANT_NOTCH_Z0 = (
    Z_PCB0 + PCB_T_MIN + UFL_CABLE_Z - UFL_CABLE_D / 2 - ANT_NOTCH_MARGIN
)  # antenna notch floor
POCKET_D = HDR_TAIL - PCB_T_MIN + POCKET_CLR  # blind tail pocket depth
HM_POST_Z0 = (
    HM_FRONT_Z + HM_T_MAX + 0.1
)  # post bottoms stay clear of the thickest board
CX0, CX1, CY0, CY1 = (
    -SIDE_CLR,
    PCB_W + SIDE_CLR,
    -GAP_ANT,
    PCB_L + GAP_USB,
)  # ESP32 cavity
HX = CX1 + WALL + HM_CLR  # HMMD PCB origin (header-side edge, bottom edge)
HY = (CY0 + CY1) / 2 - HM_L / 2
PX0, PX1, PY0, PY1 = (
    HX - HM_CLR,
    HX + HM_W + HM_CLR,
    HY - HM_CLR,
    HY + HM_L + HM_CLR,
)  # HMMD pocket
OX0, OX1, OY0, OY1 = CX0 - WALL, PX1 + WALL, CY0 - WALL, CY1 + WALL
OUT_R = CAV_R + WALL
OVH = SKIRT_CLR + SKIRT_T  # lid overhang beyond the base
HDR_Y1 = PIN_Y_FIRST + PIN_PITCH / 2
HDR_Y0 = PIN_Y_FIRST - (PIN_COUNT - 0.5) * PIN_PITCH
PIN_YS = [PIN_Y_FIRST - i * PIN_PITCH for i in range(PIN_COUNT)]
LID_SLOT_X = (
    PIN_COL_X[0] + HDR_W / 2 + SLOT_CLR,
    PIN_COL_X[1] - HDR_W / 2 - SLOT_CLR,
)  # inner edges, ESP32 slots
LID_SLOT_Y0, LID_SLOT_Y1 = HDR_Y0 - 0.5, HDR_Y1 + 0.24
HM_SLOT_X = (HX + HM_PIN_X - HDR_W / 2 - SLOT_CLR, HX + HM_PIN_X + HDR_W / 2 + SLOT_CLR)
HM_SLOT_Y = (
    HY + HM_PIN_V[-1] - HDR_W / 2 - SLOT_CLR,
    HY + HM_PIN_V[0] + HDR_W / 2 + SLOT_CLR,
)


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


def ovl(a, b):
    """Overlap volume, skipping the boolean when the bounding boxes are apart."""
    return a.common(b).Volume if a.BoundBox.intersect(b.BoundBox) else 0.0


def rail_box(u0, u1, v0, v1):
    """Ledge block: reaches from the pocket wall (and 0.5 mm into it) to the contact strip, radome up to the PCB."""
    x0, x1, y0, y1 = HX + u0, HX + u1, HY + v0, HY + v1
    if u0 <= 0:
        x0 = PX0 - 0.5
    if u1 >= HM_W:
        x1 = PX1 + 0.5
    if v0 <= 0:
        y0 = PY0 - 0.5
    if v1 >= HM_L:
        y1 = PY1 + 0.5
    return box(x0, y0, RADOME_T - 0.2, x1, y1, HM_FRONT_Z)


# ------------------------------------------------------------------ ESP32 board model (reference only)
def build_board(pcb_t=PCB_T, with_headers=True, with_antenna=ANT_NOTCH):
    parts = {}  # key -> (label, shape, rgb)
    z0 = Z_PCB0  # the PCB rests on the platform
    z1 = z0 + pcb_t

    pcb = rbox(0, 0, z0, PCB_W, PCB_L, z1, PCB_R)
    for x in PIN_COL_X:
        for y in PIN_YS:
            pcb = pcb.cut(cyl(x, y, z0 - 0.1, PIN_HOLE_D / 2, pcb_t + 0.2))
    parts["PCB"] = (
        "ESP32 PCB red 22.52x18 (thickness %.1f)" % pcb_t,
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
        "UFL",
        "U.FL antenna socket",
        UFL_X,
        UFL_Y,
        UFL_W,
        UFL_W,
        UFL_H,
        (0.80, 0.65, 0.20),
    )
    if with_antenna:  # mated U.FL plug + pigtail leaving toward the antenna-end wall
        parts["UFL_plug"] = (
            "U.FL plug (mated)",
            box(
                UFL_X - UFL_PLUG_W / 2,
                UFL_Y - UFL_PLUG_W / 2,
                z1 + UFL_H,
                UFL_X + UFL_PLUG_W / 2,
                UFL_Y + UFL_PLUG_W / 2,
                z1 + UFL_H + UFL_PLUG_H,
            ),
            (0.55, 0.55, 0.58),
        )
        parts["UFL_cable"] = (
            "U.FL pigtail %.2f mm coax, leaving through the wall notch" % UFL_CABLE_D,
            cyl(
                UFL_X,
                UFL_Y - UFL_PLUG_W / 2,
                z1 + UFL_CABLE_Z,
                UFL_CABLE_D / 2,
                8.0,
                (0, -1, 0),
            ),
            (0.10, 0.10, 0.10),
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
                "ESP32 header %s housing 1x8" % tag,
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
                "ESP32 header %s pins" % tag,
                Part.makeCompound(pins),
                (0.85, 0.70, 0.25),
            )
    return parts


# ------------------------------------------------------------------ HMMD board model (reference only)
def build_hmmd(t=HM_T):
    parts = {}
    zf = HM_FRONT_Z  # front (antenna) face, resting on the ledges
    zb = zf + t  # back face, header side

    def P(u):
        return HX + u

    def Q(v):
        return HY + v

    pcb = rbox(HX, HY, zf, HX + HM_W, HY + HM_L, zb, HM_R)
    for v in HM_PIN_V:
        pcb = pcb.cut(cyl(P(HM_PIN_X), Q(v), zf - 0.1, 0.45, t + 0.2))
    parts["PCB"] = ("HMMD PCB 20x20 (thickness %.1f)" % t, pcb, (0.10, 0.22, 0.55))

    for k, (
        u0,
        u1,
        v0,
        v1,
    ) in HM_PATCHES.items():  # antenna patches: thin plates on the front face
        parts["Patch_" + k] = (
            "HMMD antenna patch " + k,
            box(P(u0), Q(v0), zf - 0.05, P(u1), Q(v1), zf),
            (0.93, 0.78, 0.25),
        )
    for key, (cx, cy, w, d, h) in HM_FRONT_PARTS.items():
        parts["F_" + key] = (
            "HMMD front: " + key,
            box(P(cx - w / 2), Q(cy - d / 2), zf - h, P(cx + w / 2), Q(cy + d / 2), zf),
            (0.15, 0.15, 0.17),
        )
    for key, (cx, cy, w, d, h) in HM_BACK_PARTS.items():
        parts["B_" + key] = (
            "HMMD back: " + key,
            box(P(cx - w / 2), Q(cy - d / 2), zb, P(cx + w / 2), Q(cy + d / 2), zb + h),
            (0.45, 0.45, 0.48),
        )

    hx = P(
        HM_PIN_X
    )  # J2: housing on the back, long pins up, tails poking out of the front
    parts["Hdr_housing"] = (
        "HMMD J2 header housing 1x5",
        box(
            hx - HDR_W / 2,
            Q(HM_PIN_V[-1]) - HDR_W / 2,
            zb,
            hx + HDR_W / 2,
            Q(HM_PIN_V[0]) + HDR_W / 2,
            zb + HDR_H,
        ),
        (0.10, 0.10, 0.10),
    )
    pins = [
        box(
            hx - 0.32,
            Q(v) - 0.32,
            zb - HDR_TAIL,
            hx + 0.32,
            Q(v) + 0.32,
            zb + HDR_H + HDR_LONG,
        )
        for v in HM_PIN_V
    ]
    parts["Hdr_pins"] = (
        "HMMD J2 pins (OT2 RX TX GND 3V3)",
        Part.makeCompound(pins),
        (0.85, 0.70, 0.25),
    )
    return parts


# ------------------------------------------------------------------ case model
def build_case():
    ymid = (CY0 + CY1) / 2
    bump_z = RIM_Z - 1.0

    # ---- base: one block, two bays
    outer = rbox(OX0, OY0, 0, OX1, OY1, RIM_Z, OUT_R)

    # ESP32 bay on a solid platform; the header tails drop into BLIND pockets, so the underside stays closed
    esp_bay = rbox(CX0, CY0, Z_PCB0, CX1, CY1, RIM_Z + 1, CAV_R)
    py0, py1 = PIN_YS[-1] - POCKET_PAD, PIN_YS[0] + POCKET_PAD
    prism = rbox(
        CX0, CY0, Z_PCB0 - POCKET_D, CX1, CY1, Z_PCB0 + 0.5, CAV_R
    )  # pockets follow the bay outline
    px_l, px_r = PIN_COL_X[0] + POCKET_X, PIN_COL_X[1] - POCKET_X
    zlo, zhi = Z_PCB0 - POCKET_D - 1, Z_PCB0 + 1
    pockets = prism.common(box(CX0 - 1, py0, zlo, px_l, py1, zhi)).fuse(
        prism.common(box(px_r, py0, zlo, CX1 + 1, py1, zhi))
    )

    # HMMD pocket: thin floor = radome; ledges at the PCB edge hold the antenna face HM_H above it
    hm_pocket = rbox(PX0, PY0, RADOME_T, PX1, PY1, RIM_Z + 1, CAV_R)
    for u0, u1, v0, v1 in HM_RAILS.values():
        hm_pocket = hm_pocket.cut(rail_box(u0, u1, v0, v1))

    base = outer.cut(esp_bay.fuse(pockets).fuse(hm_pocket))

    nz0 = (
        Z_PCB0 + PCB_T_MIN - 0.1
    )  # USB-C notch floor sits below the connector even on a 1.0 mm PCB
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

    if ANT_NOTCH:  # cable exit for an external U.FL antenna: open at the top, closed by the lid plate
        ant = box(
            UFL_X - ANT_NOTCH_W / 2,
            OY0 - 1,
            ANT_NOTCH_Z0,
            UFL_X + ANT_NOTCH_W / 2,
            CY0 + 0.05,
            RIM_Z + 1,
        )
        ant = fillet_where(
            ant,
            0.5,
            lambda a, b: (
                abs(a.x - b.x) < 1e-7
                and abs(a.z - b.z) < 1e-7
                and abs(a.z - ANT_NOTCH_Z0) < 1e-7
            ),
        )
        base = base.cut(ant)

    for xo, s in (
        (OX0, +1),
        (OX1, -1),
    ):  # snap grooves; s points from the outer face into the wall
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

    # ---- lid: plate + outside skirt, hold-down rib/posts, snap bumps, then the cut-outs
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
        Z_PCB0 + PCB_T_MAX + 0.1,
        USB_CX + RIB_X,
        RIB_Y1,
        RIM_Z + 0.01,
    )
    posts = [
        box(
            HX + u - HM_POST / 2,
            HY + v - HM_POST / 2,
            HM_POST_Z0,
            HX + u + HM_POST / 2,
            HY + v + HM_POST / 2,
            RIM_Z + 0.01,
        )
        for u, v in HM_POSTS
    ]
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
    lid = fuse_all([plate, skirt, rib] + posts + bumps)

    if ANT_NOTCH:  # skirt opened in front of the antenna exit; the plate overhang stays above the cable
        lid = lid.cut(
            box(
                UFL_X - (ANT_NOTCH_W + 1.0) / 2,
                LY0 - 1,
                RIM_Z - SKIRT_D - 1,
                UFL_X + (ANT_NOTCH_W + 1.0) / 2,
                OY0,
                RIM_Z,
            )
        )

    # lid edge and skirt cut back in front of the USB-C so cable overmoulds can seat
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
    # ESP32 header slots (clipped to the bay outline); a connector body can sink into them down to the housing
    prism_up = rbox(CX0, CY0, RIM_Z - 1, CX1, CY1, TOP_Z + 1, CAV_R)
    lid_slots = prism_up.common(
        box(CX0 - 1, LID_SLOT_Y0, RIM_Z - 1, LID_SLOT_X[0], LID_SLOT_Y1, TOP_Z + 1)
    ).fuse(
        prism_up.common(
            box(LID_SLOT_X[1], LID_SLOT_Y0, RIM_Z - 1, CX1 + 1, LID_SLOT_Y1, TOP_Z + 1)
        )
    )
    lid = lid.cut(lid_slots)
    lid = lid.cut(
        box(
            HM_SLOT_X[0], HM_SLOT_Y[0], RIM_Z - 1, HM_SLOT_X[1], HM_SLOT_Y[1], TOP_Z + 1
        )
    )  # HMMD J2 slot
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
        "3.0 x 3.0 x 1.25 mm, centre x = 13.82, y = 2.61 (antenna end); mated plug stack 2.5 mm. Check the 0-ohm pad between the ceramic antenna and the socket: sources disagree on which antenna is selected by default",
        "measured on illustration; retailers",
    ),
    (
        "Power",
        "5 V via USB-C or 5V pin (3.3-6 V external); 3.3 V logic, not 5 V tolerant",
        "datasheet",
    ),
    ("Underside", "flat - single-sided SMD, no components", "datasheet"),
    (
        "Case: ESP32 GPIO",
        "headers soldered housing-up, long pins through two lid slots; tails sit in blind pockets in a solid platform (underside closed)",
        "design choice",
    ),
    (
        "Case: ext. antenna",
        "3.0 mm wide cable notch in the antenna-end wall, centred on the U.FL, open to the lid (floor 8.1 mm above the underside); lid skirt opened to match; fits 1.13 / 1.37 mm pigtails",
        "design choice",
    ),
    (
        "HMMD module",
        "Waveshare HMMD mmWave Sensor: S3KM1110 24 GHz FMCW, 20 x 20 mm, 3.3 V, UART 115200 + OT2 presence output",
        "Waveshare wiki",
    ),
    (
        "HMMD pins",
        "J2 5-pin 2.54 mm header on the BACK: OT2, RX, TX, GND, 3V3; pin line 1.5 mm from the edge, first pin 1.62 mm from the corner",
        "dimensioned drawing",
    ),
    (
        "HMMD antennas",
        "two 5.9 x 2.5 mm patches on the FRONT, opposite corners; header pins point AWAY from them -> antennas face down in this case",
        "dimensioned drawing",
    ),
    (
        "HMMD thickness",
        "NOT published. Modelled 1.0; case verified for 0.8-1.6 mm",
        "assumption - measure yours",
    ),
    (
        "HMMD radome",
        "floor 0.8 mm (<= 1/8 wavelength for eps_r <= 3.7); antenna face to radome inner surface H = 6.2 mm (half wavelength in air, +-1.2 mm)",
        "Waveshare radome design guide",
    ),
    (
        "HMMD mount",
        "20.4 x 20.4 mm pocket; 3 ledges 0.8 mm under the front-face edge, >= 1.4 mm from the patches; 2 lid posts on bare solder mask",
        "design choice",
    ),
    (
        "Wiring hint",
        "HMMD 3V3 -> ESP32 3.3, GND -> G, TX -> an ESP32 RX GPIO, RX -> a TX GPIO, OT2 -> any GPIO (3.3 V logic both sides)",
        "Waveshare wiki",
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
    sh.setColumnWidth("A", 120)
    sh.setColumnWidth("B", 700)
    sh.setColumnWidth("C", 220)
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

    objs = {}
    for builder, gname, glabel, prefix in (
        (
            build_board,
            "Board_Reference",
            "ESP32-C3 SuperMini Plus (red) - reference model",
            "Board_",
        ),
        (
            build_hmmd,
            "HMMD_Reference",
            "Waveshare HMMD mmWave sensor - reference model",
            "HMMD_",
        ),
    ):
        grp = doc.addObject("App::DocumentObjectGroup", gname)
        grp.Label = glabel
        objs[gname] = {}
        for key, (label, shape, rgb) in builder().items():
            objs[gname][key] = new_part(doc, prefix + key, label, shape, rgb, group=grp)
    add_specs(doc)
    doc.recompute()
    return doc, base, lid, objs["Board_Reference"], objs["HMMD_Reference"]


def verify(doc, esp_objs, hm_objs):
    """Numeric fit check: overlaps, clearances, thickness sweeps, radome/ledge/keep-out geometry, closed underside."""
    base = doc.Case_Base.Shape.copy()
    lid = doc.Case_Lid.Shape.copy()
    lid.translate(V(0, 0, -doc.Case_Lid.Placement.Base.z))  # undo display offset
    fails = []

    def check(ok, msg):
        if not ok:
            fails.append(msg)

    def inside(shape, x, y, z):
        return shape.isInside(V(x, y, z), 1e-6, False)

    check(ovl(base, lid) < 1e-6, "base and lid overlap")

    def table(title, objs):
        print(
            "%-22s %9s %9s %10s %10s   [%s]"
            % ("part", "ovl base", "ovl lid", "gap base", "gap lid", title)
        )
        for key, o in objs.items():
            s = o.Shape
            ob, ol = ovl(base, s), ovl(lid, s)
            db, dl = s.distToShape(base)[0], s.distToShape(lid)[0]
            print("%-22s %9.4f %9.4f %10.3f %10.3f" % (key, ob, ol, db, dl))
            check(ob < 1e-6, "%s intersects base" % key)
            check(ol < 1e-6, "%s intersects lid" % key)

    table("ESP32", esp_objs)
    table("HMMD (nominal %.1f mm PCB)" % HM_T, hm_objs)

    print(
        "ESP32 PCB thickness sweep (sized for %.1f-%.1f mm):" % (PCB_T_MIN, PCB_T_MAX)
    )
    for t in (PCB_T_MIN, 1.2, 1.4, PCB_T_MAX):
        parts = build_board(pcb_t=t)
        ovlt = sum(ovl(base, s) + ovl(lid, s) for _, s, _ in parts.values())
        usb = parts["USB_C"][1]
        btn = min(
            parts["Btn_BOOT"][1].distToShape(lid)[0],
            parts["Btn_RST"][1].distToShape(lid)[0],
        )
        prot = Z_PCB0 + t + HDR_H + HDR_LONG - TOP_Z
        cab = parts.get("UFL_cable")
        cab_s = (
            " | antenna cable gap base %.2f lid %.2f"
            % (cab[1].distToShape(base)[0], cab[1].distToShape(lid)[0])
            if cab
            else ""
        )
        print(
            "  T=%.1f  overlap=%.4f | USB gap lid %.2f base %.2f | button-to-lid %.2f | pins above lid %.2f%s"
            % (
                t,
                ovlt,
                usb.distToShape(lid)[0],
                usb.distToShape(base)[0],
                btn,
                prot,
                cab_s,
            )
        )
        check(ovlt < 1e-6, "ESP32 interference at PCB thickness %.1f" % t)
        check(prot > 3.0, "ESP32 pins stick out < 3 mm at PCB thickness %.1f" % t)

    print("HMMD PCB thickness sweep (sized for %.1f-%.1f mm):" % (HM_T_MIN, HM_T_MAX))
    for t in (HM_T_MIN, HM_T, 1.2, HM_T_MAX):
        parts = build_hmmd(t)
        ovlt = sum(ovl(base, s) + ovl(lid, s) for _, s, _ in parts.values())
        hdr = parts["Hdr_housing"][1].distToShape(lid)[0]
        prot = HM_FRONT_Z + t + HDR_H + HDR_LONG - TOP_Z
        print(
            "  T=%.1f  overlap=%.4f | housing-to-lid gap %.2f | post-to-PCB gap %.2f | pins above lid %.2f"
            % (t, ovlt, hdr, HM_POST_Z0 - (HM_FRONT_Z + t), prot)
        )
        check(ovlt < 1e-6, "HMMD interference at PCB thickness %.1f" % t)
        check(prot > 3.0, "HMMD pins stick out < 3 mm at PCB thickness %.1f" % t)

    mid_lid = RIM_Z + LID_T / 2
    # ---- ESP32 cut-outs
    for name, (bx, by) in BUTTONS.items():
        check(not inside(lid, bx, by, mid_lid), "%s hole missing" % name)
        check(
            inside(lid, bx + BTN_HOLE_D / 2 + 0.3, by, mid_lid),
            "%s hole too big" % name,
        )
    zc = Z_PCB1 + USB_H / 2
    check(not inside(base, USB_CX, (OY1 + PCB_L) / 2 + 0.3, zc), "USB notch missing")
    check(
        inside(base, USB_CX + USB_NOTCH_W / 2 + 0.5, OY1 - WALL / 2, zc),
        "wall beside USB missing",
    )
    check(not inside(lid, USB_CX, OY1 + 0.4, mid_lid), "lid not cut back at USB")
    check(
        inside(lid, USB_CX, OY1 - 0.4, mid_lid), "lid plate missing over USB-end wall"
    )
    check(inside(lid, USB_CX, 11, mid_lid), "lid plate missing over the ESP32 centre")
    if ANT_NOTCH:  # external antenna exit: notch in the base wall, skirt opened, plate overhang kept above the cable
        zc_ant = Z_PCB1 + UFL_CABLE_Z
        ya, ys = OY0 + WALL / 2, OY0 - SKIRT_CLR - SKIRT_T / 2
        check(
            not inside(base, UFL_X, ya, zc_ant),
            "antenna cable notch missing in the wall",
        )
        check(
            inside(base, UFL_X + ANT_NOTCH_W / 2 + 0.5, ya, zc_ant),
            "wall beside the antenna notch missing",
        )
        check(
            inside(base, UFL_X, ya, ANT_NOTCH_Z0 - 0.3),
            "wall under the antenna notch missing",
        )
        check(
            not inside(lid, UFL_X, ys, RIM_Z - 1.0),
            "lid skirt not opened at the antenna exit",
        )
        check(
            inside(lid, UFL_X, ys, RIM_Z + 0.5),
            "lid plate overhang missing above the antenna exit",
        )
    for x in PIN_COL_X:
        for y in PIN_YS:
            check(
                not inside(lid, x, y, mid_lid),
                "no lid slot over ESP32 pin %.2f,%.2f" % (x, y),
            )
            check(
                not inside(base, x, y, Z_PCB0 - 1.0),
                "no tail pocket under ESP32 pin %.2f,%.2f" % (x, y),
            )
            check(
                inside(base, x, y, 0.5),
                "tail pocket opens through the underside at %.2f,%.2f" % (x, y),
            )

    # ---- closed underside: every sampled point 2 mm inside the outline is solid at z = 0.05
    holes = 0
    x = OX0 + 2.0
    while x <= OX1 - 2.0:
        y = OY0 + 2.0
        while y <= OY1 - 2.0:
            holes += 0 if inside(base, x, y, 0.05) else 1
            y += 1.0
        x += 1.0
    print("underside openings found in a 1 mm sample grid: %d" % holes)
    check(holes == 0, "underside has %d open sample points" % holes)

    # ---- HMMD: header slot, radome, ledges, keep-outs, posts
    for name, v in zip(HM_PIN_NAMES, HM_PIN_V):
        check(
            not inside(lid, HX + HM_PIN_X, HY + v, mid_lid),
            "no lid slot over HMMD pin %s" % name,
        )
    check(inside(lid, HX + 6.0, HY + 10.0, mid_lid), "lid plate missing over the HMMD")

    bad = 0
    n = 0
    for iu in range(3, 18, 2):
        for iv in range(3, 18, 2):
            u, v = iu + 0.5, iv + 0.5
            if any(
                u0 - 1 <= u <= u1 + 1 and v0 - 1 <= v <= v1 + 1
                for u0, u1, v0, v1 in HM_RAILS.values()
            ):
                continue
            n += 1
            if not (
                inside(base, HX + u, HY + v, RADOME_T - 0.05)
                and not inside(base, HX + u, HY + v, RADOME_T + 0.05)
            ):
                bad += 1
    print(
        "radome: %d sample points under the HMMD, %d off-thickness (floor %.2f mm, air gap H = %.2f mm)"
        % (n, bad, RADOME_T, HM_H)
    )
    check(bad == 0, "radome floor is not a uniform %.2f mm" % RADOME_T)
    pcb_zmin = hm_objs["PCB"].Shape.BoundBox.ZMin
    check(
        abs(pcb_zmin - HM_FRONT_Z) < 1e-6 and abs((pcb_zmin - RADOME_T) - HM_H) < 1e-6,
        "antenna-to-radome gap is not H",
    )

    for k, (u0, u1, v0, v1) in HM_RAILS.items():
        cu, cv = (u0 + u1) / 2, (v0 + v1) / 2
        check(inside(base, HX + cu, HY + cv, HM_FRONT_Z - 0.05), "ledge %s missing" % k)
        check(
            not inside(base, HX + cu, HY + cv, HM_FRONT_Z + 0.05),
            "ledge %s stands proud of the PCB face" % k,
        )
    for k, (u0, u1, v0, v1) in HM_PATCHES.items():
        m = HM_KEEPOUT
        kb = box(
            HX + u0 - m,
            HY + v0 - m,
            RADOME_T + 0.01,
            HX + u1 + m,
            HY + v1 + m,
            HM_FRONT_Z,
        )
        check(
            ovl(base, kb) < 1e-6,
            "case material within %.1f mm of antenna patch %s" % (m, k),
        )
    for u, v in HM_POSTS:
        check(
            inside(lid, HX + u, HY + v, RIM_Z - 0.5),
            "hold-down post at %.1f,%.1f missing" % (u, v),
        )

    # ---- snap fit
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
    check(base.isValid() and lid.isValid(), "invalid case geometry")

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
        "ESP32 platform top z=%.2f | HMMD antenna face z=%.2f | parting plane z=%.2f"
        % (Z_PCB0, HM_FRONT_Z, RIM_Z)
    )
    print(
        "RESULT:", "ALL CHECKS PASSED" if not fails else "FAILED -> " + "; ".join(fails)
    )
    return fails


doc, base, lid, board_objs, hmmd_objs = build()
fails = verify(doc, board_objs, hmmd_objs)
if App.GuiUp:
    import FreeCADGui as Gui

    Gui.ActiveDocument = Gui.getDocument(DOC_NAME)
    Gui.ActiveDocument.ActiveView.viewIsometric()
    Gui.SendMsgToActiveView("ViewFit")
if SAVE and not fails:
    doc.saveAs(SAVE_PATH)
    print("saved:", SAVE_PATH)
