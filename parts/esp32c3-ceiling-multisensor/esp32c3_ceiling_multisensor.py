"""ESP32-C3 SuperMini Plus (red) + DHT22 + MH-Z19B + Waveshare HMMD - round ceiling unit, HMMD fixed at 25 deg.

Run inside FreeCAD:
    exec(open("/Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor/esp32c3_ceiling_multisensor.py").read())
or headless:
    freecadcmd /Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor/esp32c3_ceiling_multisensor.py

Frame (mm): origin on the hole axis at the UNDERSIDE of the ceiling, +Z up into the ceiling. The trim (bezel) sits on
the room side (z = -BEZEL_T..0) and covers the edge of the hole; the housing fills the hole and the sensor tower on top
of it stands up into the open space above the ceiling. Edit a parameter and re-run; the model is only saved / exported
if every check passes.

 * Mounting: three swing clamps, like in-ceiling speakers. Push the unit into the hole and turn the three countersunk
   M4 screws in the trim clockwise: the nyloc nut in each arm drags the arm round until it hits its stop (arm now
   pointing outward over the ceiling), then the arm is pulled down onto the top of the ceiling. Any ceiling between
   CEIL_T_MIN and CEIL_T_MAX works. Undo: turn anticlockwise, the arms swing back in and the unit drops out.
 * HMMD: fixed in a plug whose flat face is the radome (same radome rules as the other case), tilted POD_TILT toward
   POD_LOOK; turn the whole unit in the hole to aim it. The plug drops straight down into a socket in the housing and
   pokes through the trim; a flare at its top sits on the socket and crush ribs hold it. Printed radome face down.
 * DHT22 module: its grille sits flush in a window of the trim, in a closed bay of its own, beside the HMMD plug.
 * MH-Z19B: hangs chamber-down over vents in the deck of the sensor tower, pins up; the bay frame keeps it in place.
   Room air reaches it through vent slots in the trim and the space under the deck (plenum).
 * ESP32-C3: stands on edge by the tower wall with its USB-C in a slot of the top lid, so a cable plugs in from above
   (the open space over the ceiling). The deck also caps the clamp channels and holds the top ends of the M4 screws.
 * Wiring: leads are soldered straight to the modules (no DuPont plugs), to keep the unit short.
"""

import math

import FreeCAD as App
import Part
from FreeCAD import Vector as V

DOC_NAME = "ESP32C3_Ceiling_Multisensor"
PART_DIR = "/Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor"
SAVE_PATH = PART_DIR + "/ESP32C3_Ceiling_Multisensor.FCStd"
STL_DIR = PART_DIR + "/stl"
SAVE = True  # save SAVE_PATH after building (only if every check passes)
# one STL per printed part, in print orientation (only if every check passes)
EXPORT_STL = True

# ------------------------------------------------------------------ ceiling (user spec)
HOLE_R = 35.0  # radius of the hole in the ceiling (70 mm across)
CEIL_T = 35.0  # ceiling thickness (open space above it); used for the reference model
CEIL_T_MIN, CEIL_T_MAX = 30.0, 40.0  # the clamps grip any thickness in this range

# ------------------------------------------------------------------ housing + sensor tower
BODY_R = HOLE_R - 1.0  # outer radius of everything that goes into the hole
WALL = 2.0
BEZEL_R, BEZEL_T = HOLE_R + 12.0, 3.0  # trim ring on the room side
DECK_T = 3.0  # tower floor: caps the clamp channels, carries the sensor bays
# M3 x 12 countersunk from the deck into columns of the housing (tower -> housing)
TOWER_SCREW_ANGLES, TOWER_SCREW_R, COL_R = (136.0, 256.0, 16.0), 28.3, 2.6
COL_Z0 = 22.0  # the columns start here, on 45 deg supports, clear of the HMMD plug and DHT22 bay
# room-air slots in the trim, by the wall (r0, r1) and the ribs between them
VENT_R0, VENT_R1, VENT_RIB = 29.75, 31.75, 2.5

# ------------------------------------------------------------------ swing clamps (M4 x 55 countersunk + M4 nyloc)
CLAMP_ANGLES = (90.0, 210.0, 330.0)
CLAMP_R = 28.0  # screw axis radius
# ISO 10642 countersunk screw; the length includes the head
M4_D, M4_BORE, M4_HEAD_D, M4_LEN = 4.0, 4.5, 8.0, 55.0
NUT4_AF, NUT4_H = 7.0, 5.0  # DIN 985
# arm: thickness, bar width, axis to tip, nut boss radius
ARM_T, ARM_W, ARM_L, ARM_BOSS_R = 7.0, 7.0, 15.0, 5.2
ARM_CLR = 0.8  # clamp channel clearance around the swinging arm
ARM_STOP_GAP = 0.3  # arm side to its stop face when pointing straight out
ARM_GRIP_MIN = 8.0  # the arms must reach this far over the ceiling top
PARK_GAP = 2.0  # parked (retracted) arm bottom above CEIL_T_MAX

# ------------------------------------------------------------------ HMMD plug (fixed tilt)
POD_TILT = 25.0  # radome face from horizontal (deg)
# direction the radar is tilted toward (deg); turn the unit in the hole to aim it
POD_LOOK = 180.0
POD_C = (0.0, -10.5)  # plug footprint centre (world x, y)
POD_WALL = 1.2  # plug wall round the HMMD pocket
POD_TOP = 14.0  # plug top above the face centre
POD_EDGE = 0.5  # high edge of the radome face below the trim face
POD_CR = 2.0  # footprint corner radius
# stop lip of the socket over the chamfered front top edge of the plug: width, thickness
LIP_W, LIP_T = 2.0, 1.5
POD_CLR = 0.2  # plug to socket
SOCK_W = 1.6  # socket wall
POD_RIB = 0.15  # crush rib interference, plug in its socket
M3_D, M3_BORE, M3_HEAD_D, M3_PILOT = 3.0, 3.4, 6.0, 2.6
# HMMD keeper: square tube + cap on the plug top
KEEPER_WALL, KEEPER_CLR, CAP_T = 1.0, 0.1, 2.0

# ------------------------------------------------------------------ ESP32-C3 SuperMini Plus (red V2) - as the other cases
# PCB size / row spacing: manufacturer datasheet. Component positions: pixel-measured on the to-scale ESPboards
# illustration (58.9 px/mm). USB-C: standard 16-pin receptacle. PCB thickness is NOT published.
PCB_W, PCB_L, PCB_R = 18.0, 22.52, 1.0
PCB_T = 1.2
PCB_T_MIN, PCB_T_MAX = 1.0, 1.6
PIN_COL_X = (1.38, PCB_W - 1.38)
PIN_COUNT, PIN_PITCH = 8, 2.54
PIN_Y_FIRST = PCB_L - 1.58
PIN_HOLE_D = 1.0
USB_CX, USB_W, USB_LEN, USB_H, USB_OVERHANG = PCB_W / 2, 8.94, 7.3, 3.26, 1.87
BUTTONS = {"BOOT": (5.96, 13.96), "RST": (12.05, 13.96)}
BTN_W, BTN_D, BTN_H = 4.0, 3.0, 2.0
HDR_W, HDR_H, HDR_TAIL, HDR_LONG = 2.54, 2.5, 3.0, 6.0
# leads soldered into the pin holes: joint below, insulated stub above (reference, wiring room)
WIRE_JOINT, WIRE_STUB = 1.0, 2.5
# the board stands on edge at ESP_ANGLE, its back face ESP_BACK_R from the axis
ESP_ANGLE, ESP_BACK_R = 90.0, 29.6
ESP_TOP_GAP = 0.3  # PCB end (USB side) to the lid underside
ESP_LIFT = 4.2  # antenna end of the board above the deck
LID_COLS = ((29.3, 60.0), (29.3, 200.0), (29.3, 320.0))  # top lid screws (r, deg)

# ------------------------------------------------------------------ Waveshare HMMD (S3KM1110, 24 GHz) - as the other case
HM_W, HM_L, HM_R = 20.0, 20.0, 1.4
HM_T = 1.0
HM_T_MIN, HM_T_MAX = 0.8, 1.6
HM_PIN_X = 1.5
HM_PIN_V = [18.38 - 2.54 * i for i in range(5)]
HM_PIN_NAMES = ("OT2", "RX", "TX", "GND", "3V3")
HM_PATCHES = {"A": (11.66, 17.56, 15.27, 17.73), "B": (2.43, 8.32, 2.17, 4.65)}
HM_KEEPOUT = 1.4
HM_FRONT_PARTS = {
    "chip": (9.96, 9.0, 5.0, 5.0, 0.9),
    "L2": (16.95, 8.0, 2.7, 2.4, 1.2),
    "U3": (15.5, 4.4, 2.6, 4.0, 0.8),
    "R14C15": (18.0, 5.5, 1.6, 1.6, 0.6),
    "R15": (16.1, 1.45, 1.6, 0.7, 0.5),
    "R16": (13.8, 1.45, 1.8, 0.7, 0.5),
}
HM_BACK_PARTS = {
    "U4": (9.5, 15.35, 3.0, 3.1, 0.9),
    "Y1": (14.95, 8.55, 3.2, 2.7, 0.8),
    "L1": (14.95, 4.3, 3.0, 3.0, 1.6),
    "C4": (14.95, 1.9, 2.3, 1.2, 1.0),
}
HM_REACH = 0.8
HM_RAILS = {
    "R": (HM_W - HM_REACH, HM_W, 2.0, 12.5),
    "T": (3.6, 10.0, HM_L - HM_REACH, HM_L),
    "L": (0.0, HM_REACH, 1.8, 6.5),
}
# <= 1/8 wavelength in the plastic for eps_r <= 3.7 (Waveshare radome guide)
RADOME_T = 0.8
# antenna face to the inside of the radome: half a wavelength in air (+-1.2 mm)
HM_H = 6.2
HM_CLR = 0.2

# ------------------------------------------------------------------ DHT22 / AM2302 module (black PCB, right-angle header)
# Sensor body: AM2302 datasheet. Module: retailer drawing (13 mm PCB, 43 mm overall incl. pins) + photos.
# bay direction and distance of its centre line from the axis (bay coordinates: z = 0 on the trim)
DHT_ANGLE, DHT_DIST = 90.0, 11.2
DHT_WIRE = 3.5  # room beyond the header pins for the soldered leads to turn up
DHT_BOSS_X = 6.0  # bay lid screws, on the wall side of the bay
DHT_BODY_W, DHT_BODY_L, DHT_BODY_T = 15.1, 25.1, 7.7
DHT_BODY_T_MIN, DHT_BODY_T_MAX = 7.4, 8.0
DHT_PCB_W, DHT_PCB_L, DHT_PCB_T = 13.0, 28.2, 1.6
DHT_OVERHANG = 9.0  # sensor body (hole tab) beyond the far end of the PCB
DHT_PIN_OUT = 6.5  # header pins beyond the near end of the PCB
DHT_LIP, DHT_CLR = 0.6, 0.25  # window lip under the grille; clearance around the body
DHT_BAY_TOP = 8.4  # bay lid underside

# ------------------------------------------------------------------ MH-Z19B (pins type), Winsen manual v1.7 drawing
MZ_ANGLE, MZ_DIST = 270.0, 4.0  # bay coordinates: z = 0 on the deck
MZ_CH_L, MZ_CH_W, MZ_CH_H = 26.53, 19.5, 6.5  # gold chamber, 8.1 mm overall with PCB
MZ_BUMP_H = 0.5  # round boss on the chamber top (8.6 mm overall)
MZ_PCB_L, MZ_PCB_W, MZ_PCB_T = 32.5, 19.7, 1.6
MZ_PIN_X = 29.5 / 2  # pin rows, centre to centre 29.5
MZ_PIN_Y0 = -MZ_CH_W / 2 + 2.70  # first pin of both rows
MZ_PIN_N = (5, 4)  # HD SR TX RX Vo | PWM AOT GND Vin
MZ_PIN_LEN, MZ_SPACER_H = 7.9, 2.5  # below the PCB in the datasheet orientation
MZ_GAP = 1.2  # chamber (bump) to the vented floor
MZ_CLR = 0.45

# ------------------------------------------------------------------ misc
# slots in the lid: room air rises from the plenum past the ESP32 into the space above (False over a cold attic)
INTERIOR_VENTS = True
WIRE_HOLE_D = 7.0  # extra hole in the lid for a power cable instead of USB-C (0 = none)
# slot in the deck (x0, y0, x1, y1, width): HMMD + DHT22 leads up into the tower, also a chimney
DECK_SLOT = (-11.0, 12.5, 11.0, 12.5, 2.4)
BAY_WALL = 1.6
BOSS_R = 3.25  # screw bosses (M3 into a 2.6 mm pilot)

# ------------------------------------------------------------------ derived
D2R = math.pi / 180.0
ARM_PARK_Z = CEIL_T_MAX + PARK_GAP  # parked arm bottom
# top of the housing = deck underside; the deck is also the top stop of the arms
HOUSING_Z = ARM_PARK_Z + ARM_T + 1.0
DECK_Z = HOUSING_Z + DECK_T  # deck top = z 0 of the bay coordinates
CH_Z0 = CEIL_T_MIN - 1.5  # the clamp channels start here, on 45 deg supports
TOP_Z = DECK_Z + ESP_LIFT + PCB_L + ESP_TOP_GAP  # tower top = lid underside
LID_T = 2.4
M4_BEARING = 2.5  # blind hole in the deck underside that takes the M4 screw end
# clamp channel, in clamp coordinates (x radial out, y anticlockwise seen from above)
NOTCH_X0 = -(ARM_W / 2 + ARM_CLR)
NOTCH_Y1 = ARM_W / 2 + ARM_STOP_GAP  # straight-out stop face
NOTCH_RAD = ARM_L + ARM_CLR
NOTCH_BOSS = ARM_BOSS_R + ARM_CLR
NUT4_POCKET = NUT4_H + 0.2

# HMMD frame (H): radome face at z = 0, HMMD toward +z, +x up the slope; the plug is printed in this frame.
# Plug frame (P): same origin (face centre), z vertical, +x the way the radar is tilted.
Z_POCKET = RADOME_T  # H: radome inner face
Z_HM_FRONT = Z_POCKET + HM_H  # H: HMMD antenna face
HM_X0, HM_Y0 = -HM_W / 2, -HM_L / 2  # H: HMMD plan origin
PKT = HM_W / 2 + HM_CLR  # half size of the square HMMD pocket
POD_HW = PKT + POD_WALL
S_T, C_T = math.sin(POD_TILT * D2R), math.cos(POD_TILT * D2R)
# P: plug footprint, back (low side) to front; the back reaches the top corner of the leaning pocket
POD_X0 = -PKT * C_T - (POD_TOP + PKT * S_T) / C_T * S_T - POD_WALL
POD_X1 = PKT * C_T - RADOME_T * S_T + POD_WALL
POD_MID = (POD_X0 + POD_X1) / 2
# world z of the face centre: the front (high) edge of the face sits POD_EDGE under the trim face
Z_FACE = -BEZEL_T - POD_EDGE - POD_X1 * S_T / C_T
KEEPER_Z0 = Z_HM_FRONT + HM_T_MAX + 0.1  # H: keeper bottom (on the HMMD back)

# DHT22 bay frame (x along the bay, y toward the axis); the bay is centred on x = 0
DHT_X_TAB = (
    DHT_OVERHANG + DHT_PCB_L + DHT_PIN_OUT + DHT_WIRE - 0.5
) / 2  # far end of the body
DHT_X_PCB0 = DHT_X_TAB - DHT_OVERHANG - DHT_PCB_L  # header end of the PCB
# inner x0, x1, half width: room for the header pins + the leads soldered to them
DHT_BAY = (DHT_X_PCB0 - DHT_PIN_OUT - DHT_WIRE, DHT_X_TAB + 0.5, DHT_BODY_W / 2 + 0.55)
DHT_Z_FACE = -BEZEL_T + DHT_LIP  # grille face

# MH-Z19B PCB lower face (chamber side, facing down) and frame underside
MZ_Z_PCB = MZ_GAP + MZ_BUMP_H + MZ_CH_H
MZ_BAY_TOP = MZ_Z_PCB + MZ_PCB_T + 0.6

ESP_Z0 = TOP_Z - ESP_TOP_GAP - PCB_L  # antenna end of the upright ESP32


# ------------------------------------------------------------------ helpers
def box(x0, y0, z0, x1, y1, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(x, y, z, r, h, axis=(0, 0, 1)):
    return Part.makeCylinder(r, h, V(x, y, z), V(*axis))


def cone(x, y, z, r0, r1, h):
    return Part.makeCone(r0, r1, h, V(x, y, z))


def hexagon(x, y, z, af, h, rot=0.0):
    rc = af / math.sqrt(3.0)
    pts = [
        V(
            x + rc * math.cos((a * 60 + rot) * D2R),
            y + rc * math.sin((a * 60 + rot) * D2R),
            z,
        )
        for a in range(7)
    ]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, h))


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


def slot(x0, y0, x1, y1, z0, h, w):
    """Rounded slot of width w from (x0, y0) to (x1, y1)."""
    d = V(x1 - x0, y1 - y0, 0)
    ang = math.degrees(math.atan2(d.y, d.x))
    s = box(0, -w / 2, 0, d.Length, w / 2, h).fuse(
        [cyl(0, 0, 0, w / 2, h), cyl(d.Length, 0, 0, w / 2, h)]
    )
    s.rotate(V(0, 0, 0), V(0, 0, 1), ang)
    s.translate(V(x0, y0, z0))
    return s


def solid(shape):
    """Unwrap a single-solid compound into a plain Solid."""
    return shape.Solids[0] if len(shape.Solids) == 1 else shape


def fuse_all(shapes):
    return shapes[0].fuse(shapes[1:]).removeSplitter() if len(shapes) > 1 else shapes[0]


def ovl(a, b):
    """Overlap volume, skipping the boolean when the bounding boxes are apart."""
    return a.common(b).Volume if a.BoundBox.intersect(b.BoundBox) else 0.0


def moved(shape, plc):
    s = shape.copy()
    s.transformShape(plc.toMatrix())
    return s


def rot_z(shape, deg):
    return moved(shape, App.Placement(V(0, 0, 0), App.Rotation(V(0, 0, 1), deg)))


def polar(r, deg):
    return V(r * math.cos(deg * D2R), r * math.sin(deg * D2R), 0)


def clamp_plc(angle):
    """Clamp coordinates -> world: origin on the screw axis, x radial out, y anticlockwise (seen from above)."""
    return App.Placement(polar(CLAMP_R, angle), App.Rotation(V(0, 0, 1), angle))


def bay_plc(angle, dist, z=0.0):
    """Bay coordinates -> world: x anticlockwise along the chord, y toward the axis, origin on the bay centre line
    at height z (the trim top for the DHT22, the deck top for the MH-Z19B)."""
    return App.Placement(
        polar(dist, angle) + V(0, 0, z), App.Rotation(V(0, 0, 1), angle + 90.0)
    )


def dht_plc():
    return bay_plc(DHT_ANGLE, DHT_DIST)


def mz_plc():
    return bay_plc(MZ_ANGLE, MZ_DIST, DECK_Z)


def esp_plc():
    """ESP32 coordinates (x across, y toward USB, z = back face to front face) -> world: upright, front facing in."""
    ur, ut = polar(1, ESP_ANGLE), polar(1, ESP_ANGLE + 90)
    ex, ey, ez = -ut, V(0, 0, 1), -ur
    t = ur * ESP_BACK_R + ut * (PCB_W / 2) + V(0, 0, ESP_Z0)
    m = App.Matrix(
        ex.x, ey.x, ez.x, t.x, ex.y, ey.y, ez.y, t.y, ex.z, ey.z, ez.z, t.z, 0, 0, 0, 1
    )
    return App.Placement(m)


def plug_plc():
    """Plug frame (P: face centre at the origin, z up, +x the way the radar is tilted) -> world."""
    rot = App.Rotation(V(0, 0, 1), POD_LOOK)
    o = V(POD_C[0], POD_C[1], Z_FACE) - rot.multVec(V(POD_MID, 0, 0))
    return App.Placement(o, rot)


def h_to_p():
    """HMMD frame (H: radome face at z = 0, HMMD toward +z) -> plug frame (P)."""
    return App.Placement(V(0, 0, 0), App.Rotation(V(0, 1, 0), -POD_TILT))


def hm_plc():
    """HMMD frame -> world."""
    return plug_plc().multiply(h_to_p())


# ------------------------------------------------------------------ ESP32 board model (reference only, its own frame)
PIN_YS = [PIN_Y_FIRST - i * PIN_PITCH for i in range(PIN_COUNT)]


def wire_stubs(x, ys, z_back, z_front):
    """Soldered leads at pin holes: joint below the board, insulated stub above (wiring room)."""
    return Part.makeCompound(
        [
            box(
                x - 0.6,
                y - 0.6,
                z_back - WIRE_JOINT,
                x + 0.6,
                y + 0.6,
                z_front + WIRE_STUB,
            )
            for y in ys
        ]
    )


def build_esp32(pcb_t=PCB_T):
    """Back face at z = 0, component side up; no headers, the leads are soldered into the pin holes."""
    parts = {}  # key -> (label, shape, rgb)
    z1 = pcb_t
    pcb = rbox(0, 0, 0, PCB_W, PCB_L, z1, PCB_R)
    for x in PIN_COL_X:
        for y in PIN_YS:
            pcb = pcb.cut(cyl(x, y, -0.1, PIN_HOLE_D / 2, pcb_t + 0.2))
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
        parts["Btn_" + name] = (
            "%s button" % name,
            body.fuse(cyl(bx, by, z1 + BTN_H - 0.4, 1.0, 0.4)),
            (0.55, 0.56, 0.6),
        )
    chip = box(9.03 - 2.5, 8.59 - 2.5, z1, 9.03 + 2.5, 8.59 + 2.5, z1 + 0.9)
    chip.rotate(V(9.03, 8.59, 0), V(0, 0, 1), 45)
    parts["Chip"] = ("ESP32-C3FN4 QFN32", chip, (0.10, 0.10, 0.11))
    for key, label, cx, cy, w, d, h, rgb in (
        ("LED_RGB", "WS2812 RGB LED", 14.26, 8.31, 2.0, 2.0, 0.84, (0.95, 0.95, 0.95)),
        ("UFL", "U.FL antenna socket", 13.82, 2.61, 3.0, 3.0, 1.25, (0.80, 0.65, 0.20)),
        (
            "Antenna",
            "Ceramic chip antenna",
            7.15,
            1.36,
            6.5,
            2.2,
            0.8,
            (0.92, 0.80, 0.76),
        ),
        (
            "Regulator",
            "Regulator SOT23-5",
            4.10,
            10.40,
            1.6,
            2.9,
            1.1,
            (0.20, 0.20, 0.22),
        ),
        ("Crystal", "Crystal 3225", 4.27, 5.77, 2.5, 3.2, 0.8, (0.70, 0.70, 0.72)),
        (
            "Diode",
            "Schottky diode SOD323",
            3.20,
            21.04,
            0.9,
            2.1,
            0.9,
            (0.15, 0.15, 0.16),
        ),
    ):
        parts[key] = (
            label,
            box(cx - w / 2, cy - d / 2, z1, cx + w / 2, cy + d / 2, z1 + h),
            rgb,
        )
    for tag, x in zip(("L", "R"), PIN_COL_X):
        parts["Wires_%s" % tag] = (
            "ESP32 soldered leads %s (wiring room)" % tag,
            wire_stubs(x, PIN_YS, 0, z1),
            (0.2, 0.2, 0.8),
        )
    return parts


# ------------------------------------------------------------------ HMMD model (reference only, HMMD frame)
def build_hmmd(t=HM_T):
    parts = {}
    zf, zb = Z_HM_FRONT, Z_HM_FRONT + t

    def P(u):
        return HM_X0 + u

    def Q(v):
        return HM_Y0 + v

    pcb = rbox(P(0), Q(0), zf, P(HM_W), Q(HM_L), zb, HM_R)
    for v in HM_PIN_V:
        pcb = pcb.cut(cyl(P(HM_PIN_X), Q(v), zf - 0.1, 0.45, t + 0.2))
    parts["PCB"] = ("HMMD PCB 20x20 (thickness %.1f)" % t, pcb, (0.10, 0.22, 0.55))
    for k, (u0, u1, v0, v1) in HM_PATCHES.items():
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
    # leads soldered into J2 from the back: the joints on the antenna side stay clear of the radome
    parts["Wires"] = (
        "HMMD J2 soldered leads (OT2 RX TX GND 3V3)",
        wire_stubs(P(HM_PIN_X), [Q(v) for v in HM_PIN_V], zf + 0.4, zb),
        (0.2, 0.2, 0.8),
    )
    return parts


# ------------------------------------------------------------------ DHT22 module model (reference only, DHT bay frame)
def build_dht(body_t=DHT_BODY_T):
    """Grille down in the trim window; PCB on top; right-angle header pointing along the bay."""
    parts = {}
    # grille face; back of the sensor = PCB front face
    zf, zb = DHT_Z_FACE, DHT_Z_FACE + body_t
    x1 = DHT_X_TAB
    x0 = x1 - DHT_BODY_L
    w = DHT_BODY_W / 2
    body = box(x0, -w, zf, x1, w, zb)
    body = body.cut(cyl(x1 - 3.5, 0, zf - 1, 1.5, body_t + 2))  # hole in the tab
    for i in range(4):  # grille, visual
        for j in range(4):
            body = body.cut(
                box(
                    x0 + 3.5 + 4.2 * i,
                    -5.3 + 3.2 * j,
                    zf - 0.1,
                    x0 + 5.0 + 4.2 * i,
                    -4.1 + 3.2 * j,
                    zf + 1.5,
                )
            )
    parts["Body"] = (
        "DHT22 / AM2302 sensor 15.1x25.1x%.1f" % body_t,
        body,
        (0.95, 0.95, 0.93),
    )
    xp = DHT_X_PCB0
    pcb = box(xp, -DHT_PCB_W / 2, zb, xp + DHT_PCB_L, DHT_PCB_W / 2, zb + DHT_PCB_T)
    pcb = pcb.cut(cyl(xp + 7.5, 0, zb - 0.1, 1.3, DHT_PCB_T + 0.2))
    parts["PCB"] = ("DHT22 module PCB 13x28.2 (black)", pcb, (0.08, 0.08, 0.09))
    zt = zb + DHT_PCB_T
    parts["Leads"] = (
        "DHT22 leads + solder joints",
        box(x0 - 1.8, -4.6, zb - 1.5, x0, 4.6, zt + 1.0),
        (0.8, 0.8, 0.8),
    )
    parts["Hdr"] = (
        "DHT22 right-angle header 1x3",
        box(xp, -3.81, zb - 2.5, xp + 2.54, 3.81, zb),
        (0.1, 0.1, 0.1),
    )
    zp = zb - 1.27
    pins = []
    for y in (-2.54, 0.0, 2.54):
        pins.append(
            box(xp - DHT_PIN_OUT, y - 0.32, zp - 0.32, xp + 1.6, y + 0.32, zp + 0.32)
        )
        pins.append(box(xp + 0.96, y - 0.32, zp, xp + 1.6, y + 0.32, zt + 0.8))
    parts["Pins"] = (
        "DHT22 header pins (- out +)",
        Part.makeCompound(pins),
        (0.85, 0.70, 0.25),
    )
    # leads soldered to the pin ends, turning up toward the wire notch in the bay lid
    x = xp - DHT_PIN_OUT
    parts["Wires"] = (
        "DHT22 soldered leads (wiring room)",
        box(x - 2.0, -3.81, zp - 1.0, x + 1.0, 3.81, DHT_BAY_TOP + 1.0),
        (0.2, 0.2, 0.8),
    )
    return parts


# ------------------------------------------------------------------ MH-Z19B model (reference only, MH-Z19B bay frame)
def build_mz():
    """Upside down: gold chamber hanging toward the vents, PCB on top, pins pointing up."""
    parts = {}
    zc0 = MZ_GAP + MZ_BUMP_H
    ch = rbox(-MZ_CH_L / 2, -MZ_CH_W / 2, zc0, MZ_CH_L / 2, MZ_CH_W / 2, MZ_Z_PCB, 1.0)
    ch = ch.fuse(cyl(6.5, -3.5, MZ_GAP, 3.0, MZ_BUMP_H + 0.01))
    parts["Chamber"] = (
        "MH-Z19B gold-plated optical chamber 26.5x19.5",
        ch,
        (0.85, 0.70, 0.30),
    )
    zt = MZ_Z_PCB + MZ_PCB_T
    parts["PCB"] = (
        "MH-Z19B PCB",
        box(-MZ_PCB_L / 2, -MZ_PCB_W / 2, MZ_Z_PCB, MZ_PCB_L / 2, MZ_PCB_W / 2, zt),
        (0.10, 0.45, 0.20),
    )
    for side, n, tag in ((-1, MZ_PIN_N[0], "A"), (1, MZ_PIN_N[1], "B")):
        x = side * MZ_PIN_X
        ys = [MZ_PIN_Y0 + PIN_PITCH * i for i in range(n)]
        parts["Spacer_" + tag] = (
            "MH-Z19B pin spacer %s" % tag,
            box(x - 1.27, ys[0] - 1.27, zt, x + 1.27, ys[-1] + 1.27, zt + MZ_SPACER_H),
            (0.1, 0.1, 0.1),
        )
        pins = [
            box(x - 0.32, y - 0.32, MZ_Z_PCB - 0.8, x + 0.32, y + 0.32, zt + MZ_PIN_LEN)
            for y in ys
        ]
        parts["Pins_" + tag] = (
            "MH-Z19B pins %s" % tag,
            Part.makeCompound(pins),
            (0.85, 0.70, 0.25),
        )
        parts["Wires_" + tag] = (
            "MH-Z19B soldered leads %s (wiring room)" % tag,
            wire_stubs(x, ys, zt + MZ_PIN_LEN - 1.5 + WIRE_JOINT, zt + MZ_PIN_LEN),
            (0.2, 0.2, 0.8),
        )
    return parts


# ------------------------------------------------------------------ case: building blocks
def notch_local(grow, z0, z1):
    """Clamp channel (clamp coordinates): everything the arm sweeps from tucked-in to straight out, plus clearance."""
    r = NOTCH_RAD + grow
    quad = box(NOTCH_X0 - grow, -r - 1, z0, r + 1, NOTCH_Y1 + grow, z1)
    return (
        cyl(0, 0, z0, r, z1 - z0)
        .common(quad)
        .fuse(cyl(0, 0, z0, NOTCH_BOSS + grow, z1 - z0))
    )


def cone_support(foot, r_in, z_top):
    """World: 45 deg support under a feature by the wall - its footprint foot(z0, z1), trimmed to a cone rising
    from the wall; r_in = the innermost radius of the footprint."""
    h = BODY_R - WALL - r_in + 1.0
    z0 = z_top - h
    keep = cyl(0, 0, z0, BODY_R, h).cut(
        cone(0, 0, z0, BODY_R - WALL + 0.01, BODY_R - WALL - h, h)
    )
    return foot(z0, z_top).common(keep)


def channel_support(p):
    return cone_support(
        lambda z0, z1: moved(notch_local(WALL, z0, z1), p),
        CLAMP_R - max(-NOTCH_X0, NOTCH_BOSS) - WALL,
        CH_Z0,
    )


def sector(r0, r1, z0, h, ang):
    """Ring sector from 0 to `ang` deg (anticlockwise)."""
    return Part.makeCylinder(r1, h, V(0, 0, z0), V(0, 0, 1), ang).cut(
        cyl(0, 0, z0 - 1, r0, h + 2)
    )


def vent_arcs():
    """Trim slots (start, end deg) between the M4 countersinks, with ribs (trim_vents() trims them further)."""
    blocked = sorted((a % 360, 12.0) for a in CLAMP_ANGLES)
    rib = math.degrees(VENT_RIB / VENT_R1)
    arcs = []
    for (a0, w0), (a1, w1) in zip(
        blocked, blocked[1:] + [(blocked[0][0] + 360, blocked[0][1])]
    ):
        s, e = a0 + w0, a1 - w1
        n = max(1, round((e - s) / 25.0))
        step = (e - s + rib) / n
        arcs += [(s + i * step, s + (i + 1) * step - rib) for i in range(n)]
    return arcs


def deck_slot():
    x0, y0, x1, y1, w = DECK_SLOT
    return slot(x0, y0, x1, y1, HOUSING_Z - 1, DECK_T + 2, w)


def rrect(x0, y0, x1, y1, r, z=0.0):
    """Rounded rectangle face in the plane at height z."""
    pts = [
        V(x0 + r, y0 + r, z),
        V(x1 - r, y0 + r, z),
        V(x1 - r, y1 - r, z),
        V(x0 + r, y1 - r, z),
    ]
    face = Part.Face(Part.makePolygon(pts + pts[:1]))
    return face.makeOffset2D(r) if r > 0 else face


def plug_outline(g, z0, z1):
    """Plug frame: the plug footprint grown by g, from z0 to z1."""
    return rrect(
        POD_X0 - g, -POD_HW - g, POD_X1 + g, POD_HW + g, POD_CR + g, z0
    ).extrude(V(0, 0, z1 - z0))


def halfspace(n, p0, size=200.0):
    """Big box on the side of the plane through p0 that the normal n points to."""
    return moved(
        box(0, -size, -size, size, size, size),
        App.Placement(p0, App.Rotation(V(1, 0, 0), n)),
    )


def plug_socket():
    """World: the socket round the plug (added to the housing) and the cavity it leaves (cut from the housing). The
    plug goes in from the room side; the 45 deg lip at the front stops it at its chamfered top edge."""
    g = POD_CLR + SOCK_W
    block = plug_outline(g, -Z_FACE - 0.01, POD_TOP + LIP_T)
    xs = POD_X1 + POD_CLR
    lip = box(
        xs - LIP_W,
        -POD_HW - g,
        POD_TOP - LIP_W - 2,
        xs + SOCK_W,
        POD_HW + g,
        POD_TOP + LIP_T,
    )
    k = POD_X1 + POD_TOP - LIP_W + POD_CLR * math.sqrt(2)
    lip = lip.common(halfspace(V(1, 0, 1), V(k, 0, 0)))
    cavity = plug_outline(POD_CLR, -60, POD_TOP + LIP_T + 1).cut(lip)
    return moved(block, plug_plc()), moved(cavity, plug_plc())


def trim_vents():
    """World: room-air slots through the trim, clear of the DHT22 bay and the plug socket."""
    x0, x1, hw = DHT_BAY
    t = BAY_WALL + 1.0
    keep_out = [
        moved(
            fuse_all(
                [box(x0 - t, -hw - t, -10, x1 + t, hw + t, 10)]
                + [cyl(bx, by, -10, BOSS_R + 1, 20) for bx, by in dht_bay()[3]]
            ),
            dht_plc(),
        ),
        moved(plug_outline(POD_CLR + SOCK_W + 1, -60, 60), plug_plc()),
    ]
    arcs = fuse_all(
        [
            rot_z(sector(VENT_R0, VENT_R1, -BEZEL_T - 1, BEZEL_T + 1.5, e - s), s)
            for s, e in vent_arcs()
        ]
    ).cut(fuse_all(keep_out))
    # drop slivers (shorter than about 6 mm)
    return [s for s in arcs.Solids if s.Volume > 6.0 * 2.0 * (BEZEL_T + 1.5)]


def dht_bay():
    """DHT22 bay (bay coordinates): walls + screw bosses, the cuts, the openings through the deck, boss positions."""
    x0, x1, hw = DHT_BAY
    t = BAY_WALL
    bosses = [(s * DHT_BOSS_X, -(hw + t + BOSS_R - 1.0)) for s in (-1, 1)]
    walls = fuse_all(
        [box(x0 - t, -hw - t, 0, x1 + t, hw + t, DHT_BAY_TOP)]
        + [cyl(bx, by, 0, BOSS_R, DHT_BAY_TOP) for bx, by in bosses]
    )
    xb0, c, w = DHT_X_TAB - DHT_BODY_L, DHT_CLR, DHT_BODY_W / 2
    cuts = [
        box(x0, -hw, 0, x1, hw, DHT_BAY_TOP + 1),
        box(
            xb0 - c, -w - c, DHT_Z_FACE, DHT_X_TAB + c, w + c, 0.5
        ),  # body pocket in the trim
    ] + [cyl(bx, by, DHT_BAY_TOP - 7, M3_PILOT / 2, 8) for bx, by in bosses]
    openings = [
        box(
            xb0 + 0.75,
            -w + 0.75,
            -BEZEL_T - 1,
            DHT_X_TAB - 0.75,
            w - 0.75,
            DHT_Z_FACE + 0.1,
        )
    ]  # grille window
    return walls, cuts, openings, bosses


def mz_bay():
    """MH-Z19B bay (bay coordinates): chamber pocket with vents, ledges for the PCB ends, screw bosses."""
    hx, hy, t = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR, BAY_WALL
    bosses = [(s * (hx + t + BOSS_R - 1.0), 0.0) for s in (-1, 1)]
    walls = fuse_all(
        [box(-hx - t, -hy - t, 0, hx + t, hy + t, MZ_BAY_TOP)]
        + [cyl(bx, by, 0, BOSS_R, MZ_BAY_TOP) for bx, by in bosses]
    )
    cx = MZ_CH_L / 2 + MZ_CLR
    cuts = [
        box(-cx, -hy, 0, cx, hy, MZ_Z_PCB + 0.01),
        box(-hx, -hy, MZ_Z_PCB, hx, hy, MZ_BAY_TOP + 1),
    ]
    for s in (-1, 1):  # reliefs in the ledges for the solder joints of the pins
        cuts.append(
            box(
                s * MZ_PIN_X - 1.0,
                -hy,
                MZ_Z_PCB - 1.2,
                s * MZ_PIN_X + 1.0,
                hy,
                MZ_Z_PCB + 0.1,
            )
        )
    # vents under the chamber (the gas inlet faces down)
    openings = [
        box(x - 0.8, -7.5, -DECK_T - 1, x + 0.8, 7.5, 0.1)
        for x in (-10.5 + 3.5 * i for i in range(7))
    ]
    cuts += [cyl(bx, by, MZ_BAY_TOP - 8, M3_PILOT / 2, 9) for bx, by in bosses]
    return walls, cuts, openings, bosses


def esp_socket():
    """ESP32 coordinates: column from the deck up to the antenna end of the board, with jaws that miss every part."""
    back = -(BODY_R - WALL + 0.5 - ESP_BACK_R)  # reaches into the outer wall
    t1 = PCB_T_MAX + 0.1
    return fuse_all(
        [
            box(-1.5, -ESP_LIFT - 0.5, back, PCB_W + 1.5, 0, 3.4),  # column + floor
            box(
                -1.5, 0, back, PCB_W + 1.5, 2.5, -0.1
            ),  # back jaw (below the first header tail)
            box(-1.5, 0, back, -0.3, 3.0, 2.2),  # side stops
            box(PCB_W + 0.3, 0, back, PCB_W + 1.5, 3.0, 2.2),
            box(
                0.0, 0, t1, 1.4, 1.5, t1 + 1.5
            ),  # front fingers at the two corners (clear of headers / antenna / U.FL)
            box(PCB_W - 1.4, 0, t1, PCB_W, 1.5, t1 + 1.5),
        ]
    )


def usb_center(t=None):
    t = (PCB_T_MIN + PCB_T_MAX) / 2 if t is None else t
    return esp_plc().multVec(V(USB_CX, PCB_L + USB_OVERHANG, t + USB_H / 2))


# ------------------------------------------------------------------ case parts
def build_body():
    bezel = cyl(0, 0, -BEZEL_T, BEZEL_R, BEZEL_T)
    bezel = fillet_where(bezel, 1.5, lambda a, b: abs(a.z + BEZEL_T) < 1e-6)
    bezel = fillet_where(
        bezel, 0.6, lambda a, b: abs(a.z) < 1e-6 and a.Length > BEZEL_R - 1
    )
    disc = cyl(0, 0, 0, BODY_R, HOUSING_Z)
    adds = [bezel, disc.cut(cyl(0, 0, -1, BODY_R - WALL, HOUSING_Z + 2))]
    cuts = []
    for a in CLAMP_ANGLES:
        p = clamp_plc(a)
        adds.append(moved(notch_local(WALL, CH_Z0, HOUSING_Z), p).common(disc))
        adds.append(channel_support(p).common(disc))
        adds.append(
            moved(cyl(0, 0, 0, M4_HEAD_D / 2 + 1.0, 4.0), p)
        )  # stiffens the trim
        cuts.append(moved(notch_local(0, CH_Z0, HOUSING_Z + 1), p))
    for a in TOWER_SCREW_ANGLES:  # columns + webs to the wall for the tower screws

        def foot(z0, z1, a=a):
            web = box(0, -COL_R, z0, BODY_R - WALL + 0.5 - TOWER_SCREW_R, COL_R, z1)
            return moved(
                cyl(0, 0, z0, COL_R, z1 - z0).fuse(web),
                App.Placement(polar(TOWER_SCREW_R, a), App.Rotation(V(0, 0, 1), a)),
            )

        p = polar(TOWER_SCREW_R, a)
        adds.append(foot(COL_Z0, HOUSING_Z))
        adds.append(cone_support(foot, TOWER_SCREW_R - COL_R, COL_Z0))
        cuts.append(cyl(p.x, p.y, HOUSING_Z - 12, M3_PILOT / 2, 13))
    # DHT22 bay on the trim, grille in a window of the trim
    walls, bcuts, bopen, _ = dht_bay()
    adds.append(moved(walls, dht_plc()))
    cuts += [moved(c, dht_plc()) for c in bcuts + bopen]
    # HMMD plug socket + its opening through the trim
    block, cavity = plug_socket()
    adds.append(block)
    cuts.append(cavity)
    cuts += trim_vents()  # room air for the MH-Z19B
    body = fuse_all(adds).cut(fuse_all(cuts))
    bores = []
    for (
        a
    ) in CLAMP_ANGLES:  # M4 bores through the trim and channel support + countersinks
        p = clamp_plc(a)
        bores.append(
            moved(cyl(0, 0, -BEZEL_T - 1, M4_BORE / 2, CH_Z0 + BEZEL_T + 2), p)
        )
        rc = M4_HEAD_D / 2 + 0.2
        bores.append(
            moved(cone(0, 0, -BEZEL_T - 0.01, rc, M4_BORE / 2, rc - M4_BORE / 2), p)
        )
    body = body.cut(fuse_all(bores))
    return solid(body.removeSplitter())


def chan_mask(grow=WALL + 0.5):
    """World: the clamp channels under the deck, grown; openings in the deck stay outside this."""
    return fuse_all(
        [
            moved(notch_local(grow, HOUSING_Z - 5, DECK_Z + 1), clamp_plc(a))
            for a in CLAMP_ANGLES
        ]
    )


def build_tower():
    """Deck (caps the clamp channels, M4 bearings, MH-Z19B bay) + wall up to the lid, ESP32 socket, lid columns."""
    z0 = HOUSING_Z
    shell = cyl(0, 0, z0, BODY_R, TOP_Z - z0).cut(
        cyl(0, 0, DECK_Z, BODY_R - WALL, TOP_Z)
    )
    walls, bcuts, bopen, _ = mz_bay()
    adds = [shell, moved(walls, mz_plc())]
    cuts = [moved(c, mz_plc()) for c in bcuts]
    openings = [moved(c, mz_plc()) for c in bopen] + [deck_slot()]
    adds.append(
        moved(esp_socket(), esp_plc()).common(cyl(0, 0, z0, BODY_R - 0.5, TOP_Z - z0))
    )
    for r, a in LID_COLS:
        p = polar(r, a)
        adds.append(cyl(p.x, p.y, DECK_Z - 0.01, 3.0, TOP_Z - DECK_Z + 0.01))
        cuts.append(cyl(p.x, p.y, TOP_Z - 10, M3_PILOT / 2, 11))
    for (
        a
    ) in TOWER_SCREW_ANGLES:  # countersunk from the deck top into the housing columns
        p = polar(TOWER_SCREW_R, a)
        rc = M3_HEAD_D / 2 + 0.2
        cuts.append(cyl(p.x, p.y, z0 - 1, M3_BORE / 2, DECK_T + 2))
        cuts.append(
            cone(
                p.x,
                p.y,
                DECK_Z - rc + M3_BORE / 2 + 0.01,
                M3_BORE / 2,
                rc,
                rc - M3_BORE / 2,
            )
        )
    for a in CLAMP_ANGLES:  # blind bearings for the M4 screw ends
        p = clamp_plc(a).Base
        cuts.append(cyl(p.x, p.y, z0 - 1, M4_D / 2 + 0.3, M4_BEARING + 1))
    tower = fuse_all(adds).cut(fuse_all(cuts))
    tower = tower.cut(fuse_all(openings).cut(chan_mask()))
    return solid(tower.removeSplitter())


def rail_box(u0, u1, v0, v1):
    """HMMD ledge (HMMD frame): from the pocket wall to the contact strip, radome up to the antenna face."""
    x0, x1, y0, y1 = HM_X0 + u0, HM_X0 + u1, HM_Y0 + v0, HM_Y0 + v1
    if u0 <= 0:
        x0 = -PKT - 0.5
    if u1 >= HM_W:
        x1 = PKT + 0.5
    if v0 <= 0:
        y0 = -PKT - 0.5
    if v1 >= HM_L:
        y1 = PKT + 0.5
    return box(x0, y0, Z_POCKET - 0.2, x1, y1, Z_HM_FRONT)


def build_plug(ribs=True):
    """HMMD frame (radome face on z = 0, the way it prints): vertical walls in the world, tilted radome face, 45 deg
    chamfer behind the face, square pocket with the HMMD ledges, open at the top for the keeper."""
    body = plug_outline(0, -30, POD_TOP)
    # front top edge: 45 deg chamfer that stops under the lip of the socket
    body = body.common(halfspace(V(-1, 0, -1), V(POD_X1 - LIP_W, 0, POD_TOP)))
    if ribs:  # crush ribs on the long sides, inside the socket once the plug is home
        h = POD_CLR + POD_RIB
        z0, z1 = -Z_FACE + 0.5, POD_TOP - LIP_W - 0.5
        for dx in (-6.0, 6.0):
            x = POD_MID + dx
            body = body.fuse(
                [
                    box(x - 0.6, POD_HW - 0.01, z0, x + 0.6, POD_HW + h, z1),
                    box(x - 0.6, -POD_HW - h, z0, x + 0.6, -POD_HW + 0.01, z1),
                ]
            )
    body = moved(body, h_to_p().inverse())
    body = body.common(box(-80, -80, 0, 80, 80, 80))  # above the radome face
    # behind the face: 45 deg up from the face edge (prints without support)
    body = body.common(halfspace(V(1, 0, 1), V(-POD_HW, 0, 0)))
    pocket = box(-PKT, -PKT, Z_POCKET, PKT, PKT, 60).cut(
        fuse_all([rail_box(*q) for q in HM_RAILS.values()])
    )
    return solid(body.cut(pocket).removeSplitter())


def build_keeper(ribs=True):
    """HMMD frame: square tube on the HMMD back (clear of every part, open over J2) + cap on the plug top, with a
    slot over J2 for the leads."""
    zb = KEEPER_Z0
    o = PKT - KEEPER_CLR
    i = o - KEEPER_WALL
    ph = h_to_p().inverse()
    tube = box(-o, -o, zb, o, o, 60).cut(box(-i, -i, zb - 1, i, i, 61))
    tube = tube.common(
        moved(box(-80, -80, -80, 80, 80, POD_TOP), ph)
    )  # to the plug top
    cap = plug_outline(-0.4, POD_TOP, POD_TOP + CAP_T).common(
        box(-80, -80, POD_TOP - 1, POD_X1 - LIP_W - 0.4, 80, POD_TOP + CAP_T + 1)
    )  # clear of the socket lip
    parts = [tube, moved(cap, ph)]
    if ribs:  # crush ribs: 0.15 mm interference, so the keeper stays in when the unit is upside down
        h = KEEPER_CLR + 0.15
        z0, z1 = zb + 1.5, zb + 5.0
        parts += [
            box(-o - h, -8.0, z0, -o + 0.01, -7.0, z1),
            box(1.5, o - 0.01, z0, 2.5, o + h, z1),
            box(1.5, -o - h, z0, 2.5, -o + 0.01, z1),
        ]
    # the J2 leads stay outside the keeper and leave through the cap
    xj = HM_X0 + HM_PIN_X + HDR_W / 2 + 0.3
    keeper = fuse_all(parts).cut(box(-o - 1, HM_Y0 + 6.0, zb - 1, xj, o + 1, 80))
    return solid(keeper.removeSplitter())


def build_arm():
    """Clamp coordinates, pointing straight out, clamping face at z = 0; nyloc nut drops in from the top."""
    tipx = ARM_L - ARM_W / 2
    arm = fuse_all(
        [
            cyl(0, 0, 0, ARM_BOSS_R, ARM_T),
            box(0, -ARM_W / 2, 0, tipx, ARM_W / 2, ARM_T),
            cyl(tipx, 0, 0, ARM_W / 2, ARM_T),
        ]
    )
    arm = arm.cut(cyl(0, 0, -1, M4_BORE / 2, ARM_T + 2)).cut(
        hexagon(0, 0, ARM_T - NUT4_POCKET, NUT4_AF + 0.3, NUT4_POCKET + 1)
    )
    return solid(arm.removeSplitter())


def arm_at(arm, clamp_angle, swing=0.0, z=CEIL_T):
    """swing 0 = straight out (clamping), -90 = tucked in; z = clamping face height."""
    return moved(
        arm,
        clamp_plc(clamp_angle).multiply(
            App.Placement(V(0, 0, z), App.Rotation(V(0, 0, 1), swing))
        ),
    )


def build_lid():
    z0 = TOP_Z
    lid = cyl(0, 0, z0, BODY_R, LID_T)
    lid = fillet_where(lid, 1.0, lambda a, b: abs(a.z - z0 - LID_T) < 1e-6)
    cuts = []
    for r, a in LID_COLS:
        p = polar(r, a)
        cuts.append(cyl(p.x, p.y, z0 - 1, M3_BORE / 2, LID_T + 2))
    c = usb_center()
    ut = polar(1, ESP_ANGLE + 90)
    # receptacle at any PCB thickness: rounded rectangle (round ends would catch its corners)
    hw = (USB_H + PCB_T_MAX - PCB_T_MIN + 0.4) / 2
    hl = (USB_W + 0.6) / 2
    hole = rbox(-hl, -hw, z0 - 1, hl, hw, z0 + LID_T + 1, 1.2)
    rot = App.Rotation(V(0, 0, 1), ESP_ANGLE + 90)
    cuts.append(moved(hole, App.Placement(V(c.x, c.y, 0), rot)))
    half = (14.0 - 8.5) / 2  # pocket for the plug overmould
    cuts.append(
        slot(
            c.x - ut.x * half,
            c.y - ut.y * half,
            c.x + ut.x * half,
            c.y + ut.y * half,
            z0 + LID_T - 1.3,
            2,
            8.5,
        )
    )
    if INTERIOR_VENTS:
        for da in (-32, -22, 22, 32):
            p0, p1 = polar(12.0, ESP_ANGLE + da), polar(24.0, ESP_ANGLE + da)
            cuts.append(slot(p0.x, p0.y, p1.x, p1.y, z0 - 1, LID_T + 2, 2.0))
    if WIRE_HOLE_D > 0:
        p = polar(17.0, ESP_ANGLE)
        cuts.append(cyl(p.x, p.y, z0 - 1, WIRE_HOLE_D / 2, LID_T + 2))
    return solid(lid.cut(fuse_all(cuts)).removeSplitter())


def build_dht_lid():
    x0, x1, hw = DHT_BAY
    t, z = BAY_WALL, DHT_BAY_TOP
    bosses = dht_bay()[3]
    lid = fuse_all(
        [box(x0 - t, -hw - t, z, x1 + t, hw + t, z + 2.0)]
        + [cyl(bx, by, z, BOSS_R, 2.0) for bx, by in bosses]
    )
    cuts = [cyl(bx, by, z - 1, M3_BORE / 2, 4) for bx, by in bosses] + [
        box(x0, -hw + 1.5, z - 1, x0 + 5.5, hw - 1.5, z + 3)
    ]  # wires
    # the bay corners run into the tower wall; the lid stops short of it (axis at y = DHT_DIST)
    lid = lid.common(cyl(0, DHT_DIST, z - 1, BODY_R - WALL - 0.3, 4))
    return solid(lid.cut(fuse_all(cuts)).removeSplitter())


def build_mz_frame():
    hx, hy, t, z = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR, BAY_WALL, MZ_BAY_TOP
    bosses = mz_bay()[3]
    frame = fuse_all(
        [box(-hx - t, -hy - t, z, hx + t, hy + t, z + 2.0)]
        + [cyl(bx, by, z, BOSS_R, 2.0) for bx, by in bosses]
    )
    cuts = [cyl(bx, by, z - 1, M3_BORE / 2, 4) for bx, by in bosses]
    cuts += [
        box(s * MZ_PIN_X - 2.1, -hy + 0.6, z - 1, s * MZ_PIN_X + 2.1, hy - 0.6, z + 3)
        for s in (-1, 1)
    ]
    return solid(frame.cut(fuse_all(cuts)).removeSplitter())


def build_ceiling(t=CEIL_T):
    s = BEZEL_R + 20
    return box(-s, -s, 0, s, s, t).cut(cyl(0, 0, -1, HOLE_R, t + 2))


def build_hardware(arm_z=CEIL_T):
    """Reference screws and nuts (not printed)."""
    parts = {}
    k4 = (M4_HEAD_D - M4_D) / 2
    for i, a in enumerate(CLAMP_ANGLES):
        p = clamp_plc(a)
        scr = cone(0, 0, -BEZEL_T, M4_HEAD_D / 2, M4_D / 2, k4).fuse(
            cyl(0, 0, -BEZEL_T + k4 - 0.01, M4_D / 2, M4_LEN - k4)
        )
        parts["M4_%d" % i] = (
            "M4 x %g countersunk (clamp %d)" % (M4_LEN, i + 1),
            moved(scr, p),
            (0.75, 0.75, 0.78),
        )
        nut = hexagon(0, 0, arm_z + ARM_T - NUT4_POCKET, NUT4_AF, NUT4_H).cut(
            cyl(0, 0, arm_z, M4_D / 2, 20)
        )
        parts["Nyloc_%d" % i] = (
            "M4 nyloc nut (clamp %d)" % (i + 1),
            moved(nut, p),
            (0.75, 0.75, 0.78),
        )
    k3 = (M3_HEAD_D - M3_D) / 2
    for i, a in enumerate(TOWER_SCREW_ANGLES):  # head flush with the deck top
        q = polar(TOWER_SCREW_R, a)
        # shank at the thread core: the thread cuts into the 2.6 mm pilot by design
        scr = cone(q.x, q.y, DECK_Z - k3, M3_D / 2, M3_HEAD_D / 2, k3).fuse(
            cyl(q.x, q.y, DECK_Z - 12, M3_PILOT / 2, 12 - k3 + 0.01)
        )
        parts["M3tower_%d" % i] = (
            "M3 x 12 countersunk (tower to housing %d)" % (i + 1),
            scr,
            (0.75, 0.75, 0.78),
        )
    return parts


# ------------------------------------------------------------------ document
SPECS = [
    (
        "Ceiling",
        "hole %g mm across (R %g), ceiling %g mm thick, open space above; clamps grip %g-%g mm"
        % (2 * HOLE_R, HOLE_R, CEIL_T, CEIL_T_MIN, CEIL_T_MAX),
        "user spec",
    ),
    (
        "Size",
        "trim %g mm across; housing + sensor tower %g mm across, %.1f mm from the trim face to the lid top"
        % (2 * BEZEL_R, 2 * BODY_R, BEZEL_T + TOP_Z + LID_T),
        "design result",
    ),
    (
        "Clamps",
        "3 swing arms on M4 x 55 countersunk screws (ISO 10642) with M4 nyloc nuts (DIN 985) - the nyloc friction swings the arm out",
        "design choice",
    ),
    (
        "Room air",
        "slots in the trim by the wall feed the space under the deck, where the MH-Z19B vents open; the DHT22 has its own window in the trim",
        "design choice",
    ),
    (
        "HMMD mount",
        "fixed plug, radome face tilted %g deg (toward %g deg); turn the unit in the hole to aim it; pushed in from the room side, crush ribs"
        % (POD_TILT, POD_LOOK),
        "user spec + design choice",
    ),
    (
        "Other screws",
        "3 x M3 x 12 countersunk tower -> housing, 2 x M3 x 8 per bay lid (DHT22, MH-Z19B), 3 x M3 x 10 for the top lid - all into 2.6 mm pilot holes",
        "design choice",
    ),
    (
        "Board",
        "ESP32-C3 SuperMini Plus (V2), red PCB, 22.52 x 18 mm; on edge, USB-C in the top lid; PCB 1.0-1.6 mm checked",
        "datasheet + illustration",
    ),
    (
        "Wiring",
        "no headers or DuPont plugs: leads soldered straight into the ESP32 and HMMD pin holes and onto the DHT22 / MH-Z19B pins",
        "user choice",
    ),
    (
        "HMMD",
        "Waveshare HMMD (S3KM1110, 24 GHz) 20 x 20 mm, J2 pads on the back; radome 0.8 mm, antenna gap 6.2 mm (half wavelength)",
        "Waveshare wiki + radome guide",
    ),
    (
        "HMMD thickness",
        "NOT published. Modelled 1.0; plug checked for 0.8-1.6 mm",
        "assumption - measure yours",
    ),
    (
        "DHT22",
        "AM2302 body 15.1 x 25.1 x 7.7 mm on a 13 x 28.2 mm black PCB, right-angle 3-pin header (- out +), 43 mm overall",
        "AM2302 datasheet, retailer drawing",
    ),
    (
        "DHT22 fit",
        "grille flush in a 13.6 x 23.6 mm window of the trim (0.6 mm lip), beside the HMMD plug; body 7.4-8.0 mm checked",
        "design choice",
    ),
    (
        "MH-Z19B",
        "pins type: chamber 26.53 x 19.5, PCB 32.5 long, 8.6 mm high, pin rows 29.5 apart; mounted chamber-down over 7 vents",
        "Winsen manual v1.7",
    ),
    (
        "MH-Z19B note",
        "the terminal (JST) version is 39.3 mm long and does NOT fit this bay; 4.5-5.5 V supply -> ESP32 5V pin",
        "Winsen manual v1.7",
    ),
    (
        "Wiring hint",
        "MH-Z19B Vin->5V, TX/RX->a UART; HMMD 3V3, TX/RX->second UART, OT2->GPIO; DHT22 +->3V3, out->GPIO (pull-up on module)",
        "datasheets",
    ),
    (
        "Print",
        "housing trim-down, tower deck-down, HMMD plug radome face-down (smooth bed), keeper cap-down, lid/arms/bay lids flat",
        "design choice",
    ),
]


def add_specs(doc):
    sh = doc.addObject("Spreadsheet::Sheet", "Specs")
    sh.Label = "Specs (documentation)"
    for col, head in zip("ABC", ("Property", "Value", "Source / confidence")):
        sh.set(col + "1", "'" + head)
    for i, row in enumerate(SPECS, start=2):
        for col, val in zip("ABC", row):
            sh.set("%s%d" % (col, i), "'" + val)
    sh.setColumnWidth("A", 120)
    sh.setColumnWidth("B", 760)
    sh.setColumnWidth("C", 220)


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


def module_parts():
    """Reference models in world coordinates: {group: {key: (label, shape, rgb)}}."""
    out = {}
    for grp, builder, plc in (
        ("ESP32", build_esp32, esp_plc()),
        ("HMMD", build_hmmd, hm_plc()),
        ("DHT22", build_dht, dht_plc()),
        ("MHZ19B", build_mz, mz_plc()),
    ):
        out[grp] = {
            k: (lab, moved(s, plc), rgb) for k, (lab, s, rgb) in builder().items()
        }
    out["Hardware"] = build_hardware()
    return out


def build():
    if DOC_NAME in App.listDocuments():
        App.closeDocument(DOC_NAME)
    doc = App.newDocument(DOC_NAME)
    arm = build_arm()
    case = {
        "Body": (
            "Housing: trim + HMMD plug socket + DHT22 bay + clamp channels",
            build_body(),
            (0.93, 0.93, 0.92),
            0,
        ),
        "Tower": (
            "Sensor tower: deck + MH-Z19B bay + ESP32 socket",
            build_tower(),
            (0.85, 0.88, 0.93),
            0,
        ),
        "Plug": (
            "HMMD plug (radome face tilted %g deg)" % POD_TILT,
            build_plug(),
            (0.97, 0.97, 0.97),
            0,
        ),
        "Keeper": ("HMMD keeper + cap", build_keeper(), (0.30, 0.60, 0.90), 0),
        "Lid": (
            "Top lid (USB-C slot, power cable hole)",
            build_lid(),
            (0.30, 0.60, 0.90),
            55,
        ),
        "DHT_Lid": (
            "DHT22 bay lid",
            moved(build_dht_lid(), dht_plc()),
            (0.30, 0.60, 0.90),
            0,
        ),
        "MZ_Frame": (
            "MH-Z19B bay frame",
            moved(build_mz_frame(), mz_plc()),
            (0.30, 0.60, 0.90),
            0,
        ),
    }
    for i, a in enumerate(CLAMP_ANGLES):
        case["Arm_%d" % i] = (
            "Clamp arm %d (clamping a %g mm ceiling)" % (i + 1, CEIL_T),
            arm_at(arm, a),
            (0.95, 0.55, 0.15),
            0,
        )
    world = dict(case)
    for k in ("Plug", "Keeper"):  # HMMD frame -> world
        lab, s, rgb, tr = case[k]
        world[k] = (lab, moved(s, hm_plc()), rgb, tr)
    for k, (lab, s, rgb, tr) in world.items():
        new_part(doc, "Case_" + k, lab, s, rgb, transparency=tr)
    refs = module_parts()
    for grp, parts in refs.items():
        g = doc.addObject("App::DocumentObjectGroup", grp + "_Reference")
        g.Label = grp + " - reference model"
        for k, (lab, s, rgb) in parts.items():
            new_part(doc, grp + "_" + k, lab, s, rgb, group=g)
    g = doc.addObject("App::DocumentObjectGroup", "Ceiling_Reference")
    g.Label = "Ceiling - reference (%g mm, hole %g mm)" % (CEIL_T, 2 * HOLE_R)
    new_part(
        doc,
        "Ceiling",
        "Ceiling slab",
        build_ceiling(),
        (0.80, 0.75, 0.65),
        group=g,
        transparency=75,
    )
    add_specs(doc)
    doc.recompute()
    return doc, {k: v[1] for k, v in world.items()}, case, arm


def max_radius(shape, n=36):
    return max(math.hypot(p.x, p.y) for e in shape.Edges for p in e.discretize(n))


def verify(world, arm):
    """Numeric checks: overlaps, thickness sweeps, clamp swing / reach / stops, HMMD plug fit, radome, openings."""
    fails = []

    def check(ok, msg):
        if not ok:
            fails.append(msg)

    def inside(shape, p):
        return shape.isInside(p, 1e-6, False)

    body, lid, tower = world["Body"], world["Lid"], world["Tower"]
    case = dict(world)
    # the crush ribs of the keeper and the plug are measured on their own
    case["Keeper"] = moved(build_keeper(ribs=False), hm_plc())
    case["Plug"] = moved(build_plug(ribs=False), hm_plc())
    names = list(case)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            v = ovl(case[a], case[b])
            check(v < 1e-4, "%s and %s overlap (%.4f mm3)" % (a, b, v))
    rib = ovl(world["Keeper"], case["Plug"])
    rib2 = ovl(world["Plug"], body)
    print(
        "crush ribs (by design): keeper in the plug %.2f mm3, plug in its socket %.2f mm3"
        % (rib, rib2)
    )
    check(0.5 < rib < 10, "keeper crush ribs missing or too big")
    check(0.5 < rib2 < 10, "plug crush ribs missing or too big")

    refs = module_parts()
    for grp, parts in refs.items():
        hits = 0
        for k, (lab, s, rgb) in parts.items():
            for cn, cs in case.items():
                v = ovl(cs, s)
                if v > 1e-4:
                    hits += 1
                    check(False, "%s %s intersects %s (%.4f mm3)" % (grp, k, cn, v))
        print(
            "%-9s %2d parts vs case: %s"
            % (grp, len(parts), "clear" if not hits else "%d HITS" % hits)
        )
    comp = {
        g: Part.makeCompound([s for _, s, _ in p.values()])
        for g, p in refs.items()
        if g != "Hardware"
    }
    gl = list(comp)
    for i, a in enumerate(gl):
        for b in gl[i + 1 :]:
            check(ovl(comp[a], comp[b]) < 1e-4, "modules %s and %s collide" % (a, b))
    ceil = build_ceiling()
    for cn, cs in case.items():
        check(ovl(ceil, cs) < 1e-4, "%s cuts into the ceiling" % cn)

    print("ESP32 PCB thickness sweep (upright, USB-C in the lid):")
    for t in (PCB_T_MIN, 1.2, 1.4, PCB_T_MAX):
        parts = {k: moved(s, esp_plc()) for k, (_, s, _) in build_esp32(t).items()}
        v = sum(ovl(tower, s) + ovl(lid, s) for s in parts.values())
        usb = parts["USB_C"]
        tip = usb.BoundBox.ZMax
        print(
            "  T=%.1f overlap=%.4f | USB-C to lid %.2f, to tower %.2f | receptacle tip %.2f below the lid top"
            % (
                t,
                v,
                usb.distToShape(lid)[0],
                usb.distToShape(tower)[0],
                TOP_Z + LID_T - tip,
            )
        )
        check(v < 1e-4, "ESP32 interference at PCB thickness %.1f" % t)
    plug0, keep0 = build_plug(ribs=False), build_keeper(ribs=False)
    print("HMMD PCB thickness sweep (in the plug):")
    for t in (HM_T_MIN, HM_T, 1.2, HM_T_MAX):
        v = sum(ovl(plug0, s) + ovl(keep0, s) for _, s, _ in build_hmmd(t).values())
        print(
            "  T=%.1f overlap=%.4f | board can lift %.2f mm before the keeper"
            % (t, v, HM_T_MAX + 0.1 - t)
        )
        check(v < 1e-4, "HMMD interference at PCB thickness %.1f" % t)
    dp = dht_plc()
    print("DHT22 body thickness sweep (grille in the trim window):")
    for bt in (DHT_BODY_T_MIN, DHT_BODY_T, DHT_BODY_T_MAX):
        parts = {k: moved(s, dp) for k, (_, s, _) in build_dht(bt).items()}
        v = sum(ovl(body, s) + ovl(world["DHT_Lid"], s) for s in parts.values())
        top = max(s.BoundBox.ZMax for k, s in parts.items() if k != "Wires")
        print(
            "  T=%.1f overlap=%.4f | module top %.2f above the trim, bay lid at %.2f"
            % (bt, v, top, DHT_BAY_TOP)
        )
        check(v < 1e-4, "DHT22 interference at body thickness %.1f" % bt)

    # ---- swing clamps
    for i, a in enumerate(CLAMP_ANGLES):
        near = body.common(
            moved(box(-16, -36, -BEZEL_T - 1, 36, 22, HOUSING_Z + 1), clamp_plc(a))
        )
        for z in (CEIL_T_MIN, CEIL_T, CEIL_T_MAX, ARM_PARK_Z):
            for sw in (-90, -60, -30, 0):
                s = arm_at(arm, a, sw, z)
                v = ovl(near, s) + ovl(tower, s)
                check(
                    v < 1e-4,
                    "arm %d hits the housing at swing %g, z %g (%.3f)"
                    % (i + 1, sw, z, v),
                )
        check(
            ovl(near, arm_at(arm, a, 6, CEIL_T)) > 1e-3,
            "arm %d has no stop when pointing out" % (i + 1),
        )
        check(
            ovl(near, arm_at(arm, a, -96, ARM_PARK_Z)) > 1e-3,
            "arm %d has no stop when tucked in" % (i + 1),
        )
    rt = max(
        max_radius(arm_at(arm, CLAMP_ANGLES[0], sw, ARM_PARK_Z)) for sw in (-90, -92)
    )
    ro = max_radius(arm_at(arm, CLAMP_ANGLES[0], 0, CEIL_T))
    end = -BEZEL_T + M4_LEN
    nut_top = ARM_PARK_Z + ARM_T - NUT4_POCKET + NUT4_H
    print(
        "clamp arm: tucked in %.2f from the axis (hole %.1f) | out %.2f -> grips %.1f mm of the ceiling top"
        % (rt, HOLE_R, ro, ro - HOLE_R)
    )
    print(
        "M4 screw end z=%.2f in the deck bearing (%.1f..%.1f); parked nyloc top %.2f; travel park->%g mm ceiling %.1f mm"
        % (end, HOUSING_Z, HOUSING_Z + M4_BEARING, nut_top, CEIL_T, ARM_PARK_Z - CEIL_T)
    )
    check(rt <= HOLE_R - 1.0, "tucked-in arms do not pass through the hole")
    check(
        ro - HOLE_R >= ARM_GRIP_MIN,
        "arms grip less than %g mm of the ceiling" % ARM_GRIP_MIN,
    )
    check(end >= nut_top + 1.0, "M4 screw too short for a parked arm")
    check(
        HOUSING_Z < end <= HOUSING_Z + M4_BEARING - 0.3,
        "M4 screw end not in the deck bearing",
    )
    for k, s in (("housing", body), ("tower", tower), ("lid", lid)):
        check(
            max_radius(s.common(cyl(0, 0, 0.1, 100, TOP_Z + LID_T))) <= HOLE_R - 0.9,
            "%s too big for the hole" % k,
        )

    # ---- HMMD plug: goes in from the room side, stops at the lip, radome face clear of the trim
    asm = moved(
        Part.makeCompound([plug0, keep0] + [s for _, s, _ in build_hmmd().values()]),
        hm_plc(),
    )
    for dz in (2.0, 6.0, 12.0, 20.0, 30.0):  # pulled straight out, down into the room
        v = ovl(body, moved(asm, App.Placement(V(0, 0, -dz), App.Rotation())))
        check(v < 1e-4, "plug cannot come out %g mm into the room (%.3f)" % (dz, v))
    stop = ovl(body, moved(asm, App.Placement(V(0, 0, 0.5), App.Rotation())))
    check(stop > 1e-3, "no stop above the plug")
    bb = moved(plug0, hm_plc()).BoundBox
    edge = Z_FACE + POD_X1 * S_T / C_T
    print(
        "HMMD plug: radome face %g deg toward %g deg, high edge %.2f under the trim face; sticks out %.1f mm"
        % (POD_TILT, POD_LOOK, -BEZEL_T - edge, -BEZEL_T - bb.ZMin)
    )
    check(edge <= -BEZEL_T - 0.3, "radome face edge not below the trim face")
    # field of view: rays from the face centre, 60 deg off the face normal (sideways and toward the look direction)
    look = polar(1, POD_LOOK)
    c = hm_plc().multVec(V(0, 0, 0))
    nrm = V(look.x * S_T, look.y * S_T, -C_T)  # outward face normal
    side = polar(1, POD_LOOK + 90)
    for k, d in (
        (
            "toward the tilt",
            look * math.sin((60 + POD_TILT) * D2R)
            + V(0, 0, -math.cos((60 + POD_TILT) * D2R)),
        ),
        ("sideways", nrm * math.cos(60 * D2R) + side * math.sin(60 * D2R)),
        ("other side", nrm * math.cos(60 * D2R) - side * math.sin(60 * D2R)),
    ):
        ray = Part.makeCylinder(0.3, 80, c + d * 1.0, d)
        check(ovl(body, ray) < 1e-4, "housing in the HMMD view %s" % k)

    bad = n = 0
    for iu in range(3, 18, 2):
        for iv in range(3, 18, 2):
            u, v = iu + 0.5, iv + 0.5
            if any(
                u0 - 1 <= u <= u1 + 1 and v0 - 1 <= v <= v1 + 1
                for u0, u1, v0, v1 in HM_RAILS.values()
            ):
                continue
            n += 1
            x, y = HM_X0 + u, HM_Y0 + v
            ok = inside(plug0, V(x, y, 0.05)) and inside(
                plug0, V(x, y, Z_POCKET - 0.05)
            )
            bad += 0 if ok and not inside(plug0, V(x, y, Z_POCKET + 0.05)) else 1
    print(
        "radome: %d sample points, %d off (floor %.2f mm, air gap %.2f mm)"
        % (n, bad, RADOME_T, HM_H)
    )
    check(bad == 0, "radome floor is not a uniform %.2f mm" % RADOME_T)
    for k, (u0, u1, v0, v1) in HM_RAILS.items():
        c = V(HM_X0 + (u0 + u1) / 2, HM_Y0 + (v0 + v1) / 2, Z_HM_FRONT)
        check(
            inside(plug0, c - V(0, 0, 0.05)) and not inside(plug0, c + V(0, 0, 0.05)),
            "ledge %s wrong" % k,
        )
    for k, (u0, u1, v0, v1) in HM_PATCHES.items():
        m = HM_KEEPOUT
        kb = box(
            HM_X0 + u0 - m,
            HM_Y0 + v0 - m,
            Z_POCKET + 0.01,
            HM_X0 + u1 + m,
            HM_Y0 + v1 + m,
            Z_HM_FRONT,
        )
        check(
            ovl(plug0, kb) < 1e-6,
            "plug material within %.1f mm of antenna patch %s" % (m, k),
        )

    # ---- openings
    w = DHT_BODY_W / 2
    xm = DHT_X_TAB - DHT_BODY_L / 2
    check(
        not inside(body, dp.multVec(V(xm, 0, -BEZEL_T + 0.3))),
        "DHT22 grille window missing",
    )
    check(
        inside(body, dp.multVec(V(xm, w - 0.4, -BEZEL_T + 0.3))),
        "DHT22 window lip missing",
    )
    check(
        not inside(tower, mz_plc().multVec(V(0, 0, -DECK_T + 0.5))),
        "MH-Z19B vents missing",
    )
    # deck openings over a clamp channel stay closed: how much is left open
    slab = cyl(0, 0, HOUSING_Z + 0.1, BODY_R, 0.3)
    full = fuse_all([moved(c, mz_plc()) for c in mz_bay()[2]]).common(slab)
    frac = full.cut(tower).Volume / full.Volume
    print("MH-Z19B vents: %.0f%% open to the plenum" % (100 * frac))
    check(frac > 0.6, "MH-Z19B vents mostly over a clamp channel")
    x0, y0, x1, y1, _ = DECK_SLOT
    check(
        not inside(tower, V((x0 + x1) / 2, (y0 + y1) / 2, HOUSING_Z + 1.5)),
        "deck slot missing",
    )
    vents = trim_vents()
    area = sum(s.Volume for s in vents) / (BEZEL_T + 1.5)
    print("trim vents: %d slots, %.0f mm2" % (len(vents), area))
    check(len(vents) >= 4 and area > 80, "too few trim vents")
    for s in vents[:1]:
        check(not inside(body, s.CenterOfMass), "trim vents missing")
    for a in TOWER_SCREW_ANGLES:
        p = polar(TOWER_SCREW_R, a)
        check(
            ovl(tower, cyl(p.x, p.y, DECK_Z + 0.05, M3_HEAD_D / 2 + 0.6, 10)) < 1e-4,
            "no screwdriver access to the tower screw at %g" % a,
        )
        check(
            inside(body, V(p.x, p.y, HOUSING_Z - 13))
            and not inside(body, V(p.x, p.y, HOUSING_Z - 1)),
            "no pilot hole for the tower screw at %g" % a,
        )
    u = usb_center()
    ut = polar(1, ESP_ANGLE + 90)
    check(not inside(lid, V(u.x, u.y, TOP_Z + 0.5)), "USB-C slot missing")
    check(
        inside(lid, u + ut * (USB_W / 2 + 1.2) + V(0, 0, TOP_Z + 0.5 - u.z)),
        "USB-C slot too long",
    )
    for a in CLAMP_ANGLES:
        p = clamp_plc(a).Base
        check(
            not inside(tower, V(p.x, p.y, HOUSING_Z + 1))
            and inside(tower, V(p.x, p.y, HOUSING_Z + M4_BEARING + 0.3)),
            "no blind deck bearing over clamp screw at %g" % a,
        )
        check(
            not inside(body, V(p.x, p.y, -BEZEL_T + 0.5)),
            "no countersink at clamp %g" % a,
        )

    for k, s in world.items():
        check(s.ShapeType == "Solid" and s.isValid(), "%s is not one valid solid" % k)
    print("printed parts (bounding box, mm):")
    for k in (
        "Body",
        "Tower",
        "Plug",
        "Keeper",
        "Lid",
        "DHT_Lid",
        "MZ_Frame",
        "Arm_0",
    ):
        bb = world[k].optimalBoundingBox()
        print(
            "  %-9s %6.1f x %6.1f x %5.1f  (%5.1f cm3)"
            % (k, bb.XLength, bb.YLength, bb.ZLength, world[k].Volume / 1000)
        )
    print(
        "RESULT:", "ALL CHECKS PASSED" if not fails else "FAILED -> " + "; ".join(fails)
    )
    return fails


def export_stl():
    """One STL per printed part, laid out for printing (only the arm needs printing three times)."""
    import os

    import MeshPart

    flip = App.Placement(V(0, 0, 0), App.Rotation(V(1, 0, 0), 180))
    parts = {
        "housing": build_body(),  # trim face down
        "sensor_tower": build_tower(),  # deck down
        "hmmd_plug": build_plug(),  # radome face down, smooth bed
        "hmmd_keeper": moved(moved(build_keeper(), h_to_p()), flip),  # cap down
        "top_lid": build_lid(),  # underside down
        "dht22_bay_lid": build_dht_lid(),
        "mhz19b_bay_frame": build_mz_frame(),
        "clamp_arm_x3": build_arm(),
    }
    os.makedirs(STL_DIR, exist_ok=True)
    for name, s in parts.items():
        bb = s.optimalBoundingBox()
        s = s.copy()
        s.translate(V(-(bb.XMin + bb.XMax) / 2, -(bb.YMin + bb.YMax) / 2, -bb.ZMin))
        mesh = MeshPart.meshFromShape(
            Shape=s, LinearDeflection=0.02, AngularDeflection=0.15, Relative=False
        )
        path = "%s/%s.stl" % (STL_DIR, name)
        mesh.write(path)
        print("stl: %s (%d facets)" % (path, mesh.CountFacets))


doc, world, case, arm = build()
fails = verify(world, arm)
if App.GuiUp:
    import FreeCADGui as Gui

    Gui.ActiveDocument = Gui.getDocument(DOC_NAME)
    Gui.ActiveDocument.ActiveView.viewIsometric()
    Gui.SendMsgToActiveView("ViewFit")
if not fails:
    if SAVE:
        doc.saveAs(SAVE_PATH)
        print("saved:", SAVE_PATH)
    if EXPORT_STL:
        export_stl()
