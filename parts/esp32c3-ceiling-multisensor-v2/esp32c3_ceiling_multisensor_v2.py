"""ESP32-C3 SuperMini Plus (red) + DHT22 + MH-Z19B + Waveshare HMMD - round ceiling unit, v2: printed parts only,
HMMD pod that tilts 0-35 deg.

Run inside FreeCAD:
    exec(open("/Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor-v2/esp32c3_ceiling_multisensor_v2.py").read())
or headless:
    freecadcmd /Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor-v2/esp32c3_ceiling_multisensor_v2.py

Same ceiling, size and modules as esp32c3-ceiling-multisensor (v1), but nothing is bought apart from the modules: no
screws, nuts, springs or glue. Frame (mm): origin on the hole axis at the UNDERSIDE of the ceiling, +Z up into the
ceiling. The hole goes through the lower layer of the ceiling (CEIL_T thick); above it is a gap, then the solid upper
ceiling at ROOF_Z, so the whole unit stays below ROOF_Z - TOP_CLEAR. The trim (bezel) sits on the room side
(z = -BEZEL_T..0) and covers the edge of the hole. Edit a parameter and re-run; the model is only saved / exported if
every check passes.

 * Mounting: three clamp arms, each printed in one piece with a spindle that runs down through the tower and the
   housing to a slotted end in a hole of the trim. Turn a spindle clockwise (seen from the room) with a flat
   screwdriver: the arm swings out level into the gap, over the lower layer, then a lug on the spindle runs down a
   helical groove in the housing column and pulls the arm down onto the top of the layer. Any lower layer between
   CEIL_T_MIN and CEIL_T_MAX works; the ramp is self-locking. Anticlockwise lifts the arm and swings it back in.
 * HMMD: a barrel-shaped pod on two trunnions tilts its radome face (same radome rules as the other case) from
   POD_TILT_MIN to POD_TILT_MAX toward POD_LOOK, with a click every POD_STEP: a leaf spring in the housing drops into
   grooves on the barrel. Turn the whole unit in the hole to aim it. The pod drops into its cradle from above and a
   yoke, held down by the tower deck, keeps the trunnions in their slots.
 * DHT22 module: its grille sits flush in a window of the trim, held by crush ribs, in a bay of its own.
 * MH-Z19B: hangs chamber-down over vents in the deck of the sensor tower, pins up. Its frame (with the ESP32 tray)
   fits round the pin spacers and sits on two pins, and the top lid presses it down through two posts. Room air
   reaches it through vent slots in the trim and the space under the deck (plenum). The plenum is closed off from the
   tower, which is open to the ceiling void: the leads go up through the deck one per hole, the chamber rests on a
   rim round the vents and the frame closes the bay, so the sensor gets room air only.
 * ESP32-C3: lies flat on the tray. Power: a 5 V cable runs in the gap and comes in through a slot at the top of the
   wall, soldered to the 5V and GND pins; the USB-C is only for flashing before assembly.
 * Lid: three pins with crush ribs go into the lid columns of the tower. The tower is held on the housing by the
   three clamp spindles.
 * Wiring: leads are soldered straight to the modules (no DuPont plugs), to keep the unit short.
"""

import math

import FreeCAD as App
import Part
from FreeCAD import Vector as V

DOC_NAME = "ESP32C3_Ceiling_Multisensor_v2"
PART_DIR = "/Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor-v2"
SAVE_PATH = PART_DIR + "/ESP32C3_Ceiling_Multisensor_v2.FCStd"
STL_DIR = PART_DIR + "/stl"
SAVE = True  # save SAVE_PATH after building (only if every check passes)
# one STL per printed part, in print orientation (only if every check passes)
EXPORT_STL = True

# ------------------------------------------------------------------ ceiling (user spec, measured) - as v1
HOLE_R = 35.0  # radius of the hole in the ceiling (70 mm across)
# the hole goes through the lower layer; above it is a gap, then the solid upper ceiling
CEIL_T = 30.0  # lower layer: the clamps grip its top, in the gap
ROOF_Z = 40.0  # underside of the upper ceiling, from the room-side surface
TOP_CLEAR = 2.0  # the whole unit stays this far below the upper ceiling
CEIL_T_MIN, CEIL_T_MAX = 28.5, 30.5  # the clamps grip any lower layer in this range

# ------------------------------------------------------------------ housing + sensor tower - as v1
BODY_R = HOLE_R - 1.0  # outer radius of everything that goes into the hole
WALL = 2.0
BEZEL_R, BEZEL_T = HOLE_R + 12.0, 3.0  # trim ring on the room side
HOUSING_Z = 13.0  # housing top = deck underside
DECK_T = 2.0  # tower floor: carries the MH-Z19B bay over the plenum
# room-air slots in the trim, by the wall (r0, r1) and the ribs between them
VENT_R0, VENT_R1, VENT_RIB = 29.75, 31.75, 2.5

# ------------------------------------------------------------------ clamp arms (printed in one piece with a spindle)
CLAMP_ANGLES = (90.0, 210.0, 330.0)
CLAMP_R = 26.5  # spindle axis radius
# arm: axis to tip, width, thickness, hub radius
ARM_L, ARM_W, ARM_T, ARM_HUB_R = 23.0, 7.0, 5.0, 5.0
# arm angle from radial (deg, + anticlockwise seen from above): tucked in, end of the level swing, end of the ramp
ARM_TUCK, ARM_DWELL_END, ARM_RAMP_END = -105.0, -55.0, 45.0
ARM_DROP = 2.8  # the ramp lowers the arm this much
ARM_GAP = 0.5  # parked arm bottom above CEIL_T_MAX
ARM_CLR = 0.8  # clearance round the swept arm
ARM_TOP_GAP = 0.3  # parked arm top to the lid underside
ARM_GRIP_MIN = 10.0  # v1 gripped 8 mm of the layer top: at least 25 % more, at every thickness in the range
SP_R, SP_CLR = 4.0, 0.2  # spindle radius, radial clearance in its bores
# lug on the spindle: protrusion, height (flat underside, 45 deg top), width, clearance in the groove
LUG_D, LUG_H, LUG_W, LUG_CLR = 1.5, 2.4, 2.4, 0.25
LUG_ANGLE = 30.0  # lug direction relative to the arm (deg): the groove faces the wall
LUG_Z = 8.0  # underside of the lug with the arm parked (in the housing column)
SP_END_Z = 0.2  # spindle end with the arm parked (in the trim hole, recessed at any ramp position)
SP_SLOT_W, SP_SLOT_D = 1.2, 3.0  # cross slot in the spindle end, for a flat screwdriver
COL_R_IN, COL_R_OUT = 5.6, 7.3  # housing column: radius on the axis side / groove side
TCOL_R = 7.0  # tower column round the spindle
GROOVE_STEP = 2.5  # deg between the lug positions that make up the groove

# ------------------------------------------------------------------ HMMD pod (tilting barrel)
# direction the radar is tilted toward (deg); turn the unit in the hole to aim it
POD_LOOK = 180.0
POD_TILT_MIN, POD_TILT_MAX, POD_STEP = (
    0.0,
    35.0,
    5.0,
)  # radome face from horizontal (deg), click every POD_STEP
POD_TILT_SHOW = 20.0  # tilt in the assembled document
POD_C = (0.0, -14.1)  # tilt axis centre (world x, y); the axis runs across POD_LOOK
POD_AXIS_Z = -0.3  # tilt axis height: the trunnions sit in the trim and the housing
POD_WALL = 1.2  # pod wall round the HMMD pocket
POD_H = 10.0  # tilt axis above the radome face
POD_TOP = 14.0  # pod top above the radome face
POD_CLR = 0.4  # barrel to the trim opening
TR_R, TR_L, TR_CLR = (
    1.75,
    2.2,
    0.15,
)  # trunnions: radius, length, clearance in their slots
END_GAP, END_T = 0.3, 2.0  # pod end face to the cradle end wall; end wall thickness
# detent: leaf contact (deg above horizontal, on the high side), groove depth, leaf preload into a groove
DET_PHI, DET_AMP, DET_PRESS = 10.0, 0.4, 0.3
LEAF_T, LEAF_Z0, LEAF_Z1 = (
    1.0,
    1.3,
    5.3,
)  # leaf spring thickness and world z range (free of the trim below)
YOKE_Z0, YOKE_T, YOKE_CLR = (
    8.8,
    1.8,
    0.15,
)  # yoke bar over the pod (world z), its thickness, fit in the slots
RADOME_T = (
    0.8  # <= 1/8 wavelength in the plastic for eps_r <= 3.7 (Waveshare radome guide)
)
HM_H = (
    6.2  # antenna face to the inside of the radome: half a wavelength in air (+-1.2 mm)
)
HM_CLR = 0.2
# HMMD keeper: square tube pushed in on the HMMD back, held by crush ribs
KEEPER_WALL, KEEPER_CLR = 1.0, 0.1

# ------------------------------------------------------------------ ESP32-C3 SuperMini Plus (red V2) - as v1
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
HDR_W = 2.54
# leads soldered into the pin holes: joint below, insulated stub above (reference, wiring room)
WIRE_JOINT, WIRE_STUB = 1.0, 2.5
# the board lies flat on a tray on the MH-Z19B frame, back face down, its length along the bay y
ESP_STANDOFF = 1.2  # rails under the back face: room for the solder joints of the leads
ESP_DY = 0.0  # board centre along the bay y (USB-C toward +y)

# ------------------------------------------------------------------ Waveshare HMMD (S3KM1110, 24 GHz) - as v1
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

# ------------------------------------------------------------------ DHT22 / AM2302 module (black PCB, right-angle header)
# Sensor body: AM2302 datasheet. Module: retailer drawing (13 mm PCB, 43 mm overall incl. pins) + photos.
# bay direction and distance of its centre line from the axis (bay coordinates: z = 0 on the trim)
DHT_ANGLE, DHT_DIST = 90.0, 9.6
DHT_WIRE = 3.5  # room beyond the header pins for the soldered leads to turn up
DHT_BODY_W, DHT_BODY_L, DHT_BODY_T = 15.1, 25.1, 7.7
DHT_BODY_T_MIN, DHT_BODY_T_MAX = 7.4, 8.0
DHT_PCB_W, DHT_PCB_L, DHT_PCB_T = 13.0, 28.2, 1.6
DHT_OVERHANG = 9.0  # sensor body (hole tab) beyond the far end of the PCB
DHT_PIN_OUT = 6.5  # header pins beyond the near end of the PCB
DHT_LIP, DHT_CLR = 0.6, 0.25  # window lip under the grille; clearance around the body
DHT_RIB = 0.15  # crush rib interference on the sensor body
DHT_BAY_TOP = 10.4  # bay wall top (open to the plenum; the leads turn up there)

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
MZ_FRAME_T = 2.0
# rim round the vents that the chamber rests on, so the sensor breathes only through the vents (0: no rim); inset
# from the chamber outline, since the chamber can shift MZ_CLR in its pocket; the ledges under the PCB ends sit
# MZ_LEDGE_DROP lower, so the chamber rests on the rim
MZ_RIM_W, MZ_RIM_IN, MZ_LEDGE_DROP = 1.0, 0.8, 0.15
# frame: locating pins on two bosses (bay x, diagonally opposite) and posts up to the lid (bay x, y)
MZ_BOSS_X, MZ_PIN_D, MZ_PIN_H = 14.5, 2.4, 2.0
MZ_POSTS, MZ_POST_R = ((17.0, 6.5), (-17.0, 6.5)), 1.8

# ------------------------------------------------------------------ lid, cable, deck openings
LID_T = 1.6
# lid columns (angle, radius) between the arm sweeps; pins with crush ribs on the lid
LID_COL_ANGLES, LID_COL_DIST, LID_COL_R = (130.0, 250.0, 10.0), 28.5, 3.2
LID_PIN_D, LID_PIN_L, LID_RIB = 3.0, 4.5, 0.15
# power: a 5 V cable runs in the gap and comes in through a slot in the wall and the lid rim
CABLE_ANGLE, CABLE_W = 152.0, 6.0
CABLE_Z0 = CEIL_T_MAX + 0.5  # slot bottom, above the lower layer of the ceiling
# the leads go up through the deck one per hole, so that the space under the deck stays closed off from the tower
# (and the ceiling void behind it): rows of (first hole x, y), (step x, y), number of leads - the DHT22 leads over its
# header pins, the HMMD leads beside the MH-Z19B
LEAD_D, LEAD_CLR = (
    1.2,
    0.25,
)  # outer diameter of a lead (measure yours), clearance in its hole
LEAD_ROWS = (((20.85, 7.06), (0.0, 2.54), 3), ((3.0, -18.8), (2.2, 0.0), 5))
MZ_SPACER_CLR = 0.2  # frame round the MH-Z19B pin spacers: the frame closes the bay off from the tower
BAY_WALL = 1.6
BOSS_R = 3.25

# ------------------------------------------------------------------ derived
D2R = math.pi / 180.0
ARM_PARK_Z = CEIL_T_MAX + ARM_GAP  # parked arm bottom
DECK_Z = HOUSING_Z + DECK_T  # deck top = z 0 of the MH-Z19B bay coordinates
# tower top = lid underside, just over the parked arms
TOP_Z = ARM_PARK_Z + ARM_T + ARM_TOP_GAP
SWEEP_Z0 = ARM_PARK_Z - ARM_DROP - ARM_CLR  # the arms sweep from here up to the lid
SP_BORE = SP_R + SP_CLR
LUG_R1 = SP_R + LUG_D + LUG_CLR  # groove radius
RAMP_SPAN = ARM_RAMP_END - ARM_DWELL_END

# HMMD frame (H): radome face at z = 0, HMMD toward +z, +x up the slope (the high edge, toward POD_LOOK); the pod is
# printed in this frame. Cradle frame (C): origin on the tilt axis, z vertical, +x toward POD_LOOK, y along the axis.
Z_POCKET = RADOME_T  # H: radome inner face
Z_HM_FRONT = Z_POCKET + HM_H  # H: HMMD antenna face
HM_X0, HM_Y0 = -HM_W / 2, -HM_L / 2  # H: HMMD plan origin
PKT = HM_W / 2 + HM_CLR  # half size of the square HMMD pocket
POD_HW = (
    PKT + POD_WALL
)  # half width of the radome face, half length of the pod along the axis
POD_RB = math.hypot(POD_HW, POD_H)  # barrel radius: the face edges lie on it
KEEPER_Z0 = Z_HM_FRONT + HM_T_MAX + 0.1  # H: keeper bottom (on the HMMD back)
END_Y0 = POD_HW + END_GAP  # C: end wall inner face
END_X = POD_RB + LEAF_T + 1.0  # C: end wall half width (carries the leaf)
POD_TILTS = [
    POD_TILT_MIN + POD_STEP * i
    for i in range(int(round((POD_TILT_MAX - POD_TILT_MIN) / POD_STEP)) + 1)
]

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

ESP_Z0 = MZ_BAY_TOP + MZ_FRAME_T + ESP_STANDOFF  # bay coordinates: ESP32 back face


# ------------------------------------------------------------------ helpers
def box(x0, y0, z0, x1, y1, z1):
    """Box between two corners, in any order."""
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
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


def prism(pts, y0, y1):
    """Polygon given in the x-z plane, extruded along y from y0 to y1."""
    face = Part.Face(Part.makePolygon([V(x, y0, z) for x, z in pts + pts[:1]]))
    return face.extrude(V(0, y1 - y0, 0))


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


def crush_pin(x, y, z0, d, h, rib, n=3):
    """Round pin (tip at z0) for a hole of d + 0.2, with n vertical crush ribs that bite `rib` into the hole wall."""
    pin = cyl(x, y, z0, d / 2, h)
    ribs = [
        moved(
            box(d / 2 - 0.3, -0.4, z0 + 0.8, d / 2 + 0.1 + rib, 0.4, z0 + h),
            App.Placement(V(x, y, 0), App.Rotation(V(0, 0, 1), 360.0 * i / n)),
        )
        for i in range(n)
    ]
    return fuse_all([pin] + ribs)


# ------------------------------------------------------------------ frames
def clamp_plc(angle):
    """Clamp coordinates -> world: origin on the spindle axis, x radial out, y anticlockwise (seen from above)."""
    return App.Placement(polar(CLAMP_R, angle), App.Rotation(V(0, 0, 1), angle))


def arm_drop(psi):
    """How far the arm is below its parked height at arm angle psi (level swing, then the ramp)."""
    if psi <= ARM_DWELL_END:
        return 0.0
    return ARM_DROP * min(1.0, (psi - ARM_DWELL_END) / RAMP_SPAN)


def arm_psi(drop):
    """Arm angle at which the ramp has lowered the arm by `drop`."""
    return ARM_DWELL_END + RAMP_SPAN * drop / ARM_DROP


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
    """ESP32 coordinates (x across, y toward USB, z = back face to front face) -> world: flat on the tray of the
    MH-Z19B frame, component side up, USB-C toward the bay +y."""
    return mz_plc().multiply(
        App.Placement(V(-PCB_W / 2, ESP_DY - PCB_L / 2, ESP_Z0), App.Rotation())
    )


def lid_cols():
    """World xy of the lid columns."""
    return [polar(LID_COL_DIST, a) for a in LID_COL_ANGLES]


def cradle_plc():
    """Cradle frame (C: origin on the tilt axis, z up, +x toward POD_LOOK, y along the axis) -> world."""
    return App.Placement(
        V(POD_C[0], POD_C[1], POD_AXIS_Z), App.Rotation(V(0, 0, 1), POD_LOOK)
    )


def h_to_c(tilt):
    """HMMD frame (H) -> cradle frame (C), with the radome face tilted `tilt` deg (high edge toward +x)."""
    return App.Placement(V(0, 0, 0), App.Rotation(V(0, 1, 0), -tilt)).multiply(
        App.Placement(V(0, 0, -POD_H), App.Rotation())
    )


def hm_plc(tilt=POD_TILT_SHOW):
    """HMMD frame -> world."""
    return cradle_plc().multiply(h_to_c(tilt))


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
    # leads soldered to the pin ends, turning up toward the slot in the deck
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


# ------------------------------------------------------------------ clamp arms: building blocks
def lug_solid(grow=0.0, z0=None, z1=None):
    """Spindle coordinates (axis on z, lug along +x, z = 0 on the lug underside): the lug, flat underside and 45 deg
    top (prints without support with the arm on the bed), grown by `grow`. With z0 / z1 given: a straight-sided
    channel of the lug's width from z0 to z1 instead (the way in for the lug)."""
    c = grow
    r0, r1 = SP_R - 0.8, SP_R + LUG_D + c
    if z0 is not None:
        pts = [(r0, z0), (r1, z0), (r1, z1), (r0, z1)]
    else:
        pts = [
            (r0, -c),
            (r1, -c),
            (r1, LUG_H - LUG_D + c * (math.sqrt(2) - 1)),
            (r0, LUG_H - (r0 - SP_R) + c * math.sqrt(2)),
        ]
    w = LUG_W / 2 + c
    return prism(pts, -w, w)


def lug_at(shape, psi, z=None):
    """Spindle coordinates -> clamp coordinates: the lug with the arm at angle psi (height from the groove)."""
    z = LUG_Z - arm_drop(psi) if z is None else z
    return moved(
        shape, App.Placement(V(0, 0, z), App.Rotation(V(0, 0, 1), psi + LUG_ANGLE))
    )


def groove():
    """Clamp coordinates: the groove in the housing column - everything the lug (plus clearance) passes through
    from tucked in to the end of the ramp - and the channel the lug comes down when the arm is put in."""
    lug = lug_solid(LUG_CLR)
    face = Part.Face(
        Part.makePolygon(
            [
                V(x, 0, z)
                for x, z in [
                    (SP_R - 0.8, -LUG_CLR),
                    (LUG_R1, -LUG_CLR),
                    (LUG_R1, LUG_H - LUG_D + LUG_CLR * (math.sqrt(2) - 1)),
                    (SP_R - 0.8, LUG_H + 0.8 + LUG_CLR * math.sqrt(2)),
                    (SP_R - 0.8, -LUG_CLR),
                ]
            ]
        )
    )
    a0, a1 = ARM_TUCK + LUG_ANGLE, ARM_DWELL_END + LUG_ANGLE
    dwell = face.revolve(V(0, 0, 0), V(0, 0, 1), a1 - a0)
    parts = [moved(dwell, App.Placement(V(0, 0, LUG_Z), App.Rotation(V(0, 0, 1), a0)))]
    n = int(math.ceil((ARM_RAMP_END - ARM_DWELL_END) / GROOVE_STEP))
    psis = [ARM_TUCK, ARM_DWELL_END] + [
        ARM_DWELL_END + (ARM_RAMP_END - ARM_DWELL_END) * i / n for i in range(1, n + 1)
    ]
    parts += [lug_at(lug, p) for p in psis]
    entry = lug_solid(LUG_CLR, -LUG_CLR, HOUSING_Z + 1.0 - LUG_Z)
    parts.append(lug_at(entry, ARM_TUCK))
    return fuse_all(parts)


def lug_channel(z0, z1):
    """Clamp coordinates: straight channel for the lug (tucked-in angle) through the deck and the tower column."""
    return lug_at(lug_solid(LUG_CLR, 0, z1 - z0), ARM_TUCK, z0)


def column(r_in, r_out, z0, z1):
    """Clamp coordinates: column round the spindle, r_in on the axis side, r_out on the groove (wall) side."""
    big = cyl(0, 0, z0, r_out, z1 - z0).common(box(-0.5, -20, z0 - 1, 20, 20, z1 + 1))
    return cyl(0, 0, z0, r_in, z1 - z0).fuse(big)


def arm_outline(grow, z0, z1):
    """Arm coordinates (spindle axis on z, arm along +x): plan of the arm - hub, bar, round tip - grown by `grow`."""
    tipx = ARM_L - ARM_W / 2
    r = ARM_W / 2 + grow
    return fuse_all(
        [
            cyl(0, 0, z0, ARM_HUB_R + grow, z1 - z0),
            box(0, -r, z0, tipx, r, z1),
            cyl(tipx, 0, z0, r, z1 - z0),
        ]
    )


def arm_sweep(grow, z0, z1):
    """Clamp coordinates: everything the arm sweeps from tucked in to the end of the ramp, grown by `grow`."""
    pie = Part.makeCylinder(
        ARM_L + grow, z1 - z0, V(0, 0, z0), V(0, 0, 1), ARM_RAMP_END - ARM_TUCK
    )
    return fuse_all(
        [rot_z(pie, ARM_TUCK)]
        + [rot_z(arm_outline(grow, z0, z1), a) for a in (ARM_TUCK, ARM_RAMP_END)]
    )


def arm_plc(clamp_angle, psi, drop=None):
    """Arm coordinates -> world, with the arm at angle psi (drop: below the parked height, from the ramp)."""
    drop = arm_drop(psi) if drop is None else drop
    return clamp_plc(clamp_angle).multiply(
        App.Placement(V(0, 0, ARM_PARK_Z - drop), App.Rotation(V(0, 0, 1), psi))
    )


def arm_at(arm, clamp_angle, psi, drop=None):
    return moved(arm, arm_plc(clamp_angle, psi, drop))


# ------------------------------------------------------------------ HMMD pod: building blocks
def pod_barrel(g=0.0):
    """HMMD frame: barrel of radius POD_RB + g round the tilt axis, between the radome face and the pod top."""
    c = Part.makeCylinder(POD_RB + g, 2 * POD_HW, V(0, -POD_HW, POD_H), V(0, 1, 0))
    return c.common(box(-50, -POD_HW - 1, -g, 50, POD_HW + 1, POD_TOP + g))


def polar_xz(r, deg, t=0.0):
    """(x, z) round the tilt axis: radius r at `deg` from +x toward +z, offset t along the tangent."""
    c, s = math.cos(deg * D2R), math.sin(deg * D2R)
    return (r * c - t * s, r * s + t * c)


def detent_grooves():
    """Cradle coordinates round the axis (shift to the HMMD frame with +POD_H): V grooves on the barrel, one per
    click; the leaf touches the barrel at DET_PHI, so the groove for tilt t sits at DET_PHI - t on the pod."""
    out = []
    for t in POD_TILTS:
        a = DET_PHI - t
        pts = [
            polar_xz(POD_RB - DET_AMP, a),
            polar_xz(POD_RB + 0.6, a, DET_AMP + 0.6),
            polar_xz(POD_RB + 0.6, a, -DET_AMP - 0.6),
        ]
        out.append(prism(pts, -POD_HW - 1, POD_HW + 1))
    return out


def leaf_x0():
    """Cradle coordinates: inner face of the leaf, clear of the barrel over the whole leaf height."""
    z = LEAF_Z0 - POD_AXIS_Z
    return math.sqrt(POD_RB**2 - z * z) + 0.25


def cradle():
    """Cradle coordinates: end walls with the trunnion slots, the leaf spring between them (on the high side) and
    its bump; and the cuts: barrel opening through the trim, trunnion slots (down into the trim)."""
    z_floor = -POD_AXIS_Z  # trim top
    # the walls stop at the yoke bar, clear of the MH-Z19B vents under the deck; the yoke legs go on up to the deck
    z_top = YOKE_Z0 + YOKE_T - POD_AXIS_Z
    walls = [
        box(-END_X, s * END_Y0, z_floor - 0.01, END_X, s * (END_Y0 + END_T), z_top)
        for s in (-1, 1)
    ]
    x0 = leaf_x0()
    leaf = box(
        x0,
        -END_Y0 - 0.01,
        LEAF_Z0 - POD_AXIS_Z,
        x0 + LEAF_T,
        END_Y0 + 0.01,
        LEAF_Z1 - POD_AXIS_Z,
    )
    # V ridge along the axis, symmetric about the contact line like the grooves; its base is buried in the leaf
    e = 0.7
    bump = prism(
        [
            polar_xz(POD_RB - DET_PRESS, DET_PHI),
            polar_xz(POD_RB + e, DET_PHI, DET_PRESS + e),
            polar_xz(POD_RB + e, DET_PHI, -DET_PRESS - e),
        ],
        -POD_HW,
        POD_HW,
    )
    rs = TR_R + TR_CLR
    cuts = [
        Part.makeCylinder(
            POD_RB + POD_CLR, 2 * END_Y0, V(0, -END_Y0, 0), V(0, 1, 0)
        ).common(box(-50, -END_Y0, -20, 50, END_Y0, z_floor + 0.5))
    ]
    for s in (-1, 1):
        y0, y1 = sorted((s * (END_Y0 - 0.5), s * (END_Y0 + END_T + 0.5)))
        cuts.append(box(-rs, y0, 0, rs, y1, z_top + 1))
        cuts.append(Part.makeCylinder(rs, y1 - y0, V(0, y0, 0), V(0, 1, 0)))
    return walls + [leaf], bump, cuts


def trunnions():
    """HMMD frame: the two trunnions on the end faces of the pod."""
    return [
        Part.makeCylinder(
            TR_R, TR_L + 0.01, V(0, s * (POD_HW - 0.01), POD_H), V(0, s, 0)
        )
        for s in (-1, 1)
    ]


def build_yoke():
    """Cradle coordinates: two legs that fill the trunnion slots above the trunnions, joined by a bar over the pod;
    the deck holds it down. Printed lying on its side."""
    w = TR_R + TR_CLR - YOKE_CLR
    z0, z1 = TR_R + 0.1, HOUSING_Z - 0.2 - POD_AXIS_Z
    zb0, zb1 = YOKE_Z0 - POD_AXIS_Z, YOKE_Z0 + YOKE_T - POD_AXIS_Z
    legs = [
        box(-w, s * END_Y0, z0, w, s * (END_Y0 + END_T - 0.05), z1) for s in (-1, 1)
    ]
    bar = box(-w, -END_Y0 - 0.01, zb0, w, END_Y0 + 0.01, zb1)
    return solid(fuse_all(legs + [bar]))


# ------------------------------------------------------------------ bays, vents, openings
def sector(r0, r1, z0, h, ang):
    """Ring sector from 0 to `ang` deg (anticlockwise)."""
    return Part.makeCylinder(r1, h, V(0, 0, z0), V(0, 0, 1), ang).cut(
        cyl(0, 0, z0 - 1, r0, h + 2)
    )


def vent_arcs():
    """Trim slots (start, end deg) between the clamp columns, with ribs (trim_vents() trims them further)."""
    blocked = sorted([(a % 360, 18.0) for a in CLAMP_ANGLES])
    rib = math.degrees(VENT_RIB / VENT_R1)
    arcs = []
    for (a0, w0), (a1, w1) in zip(
        blocked, blocked[1:] + [(blocked[0][0] + 360, blocked[0][1])]
    ):
        s, e = a0 + w0, a1 - w1
        if e - s < 10.0:  # too short to print cleanly
            continue
        n = max(1, round((e - s) / 25.0))
        step = (e - s + rib) / n
        arcs += [(s + i * step, s + (i + 1) * step - rib) for i in range(n)]
    return arcs


def dht_bay():
    """DHT22 bay (bay coordinates): walls, the cuts (bay + body pocket in the trim), the grille window, and the crush
    ribs that hold the sensor body in its pocket."""
    x0, x1, hw = DHT_BAY
    t = BAY_WALL
    walls = box(x0 - t, -hw - t, 0, x1 + t, hw + t, DHT_BAY_TOP)
    xb0, c, w = DHT_X_TAB - DHT_BODY_L, DHT_CLR, DHT_BODY_W / 2
    cuts = [
        box(x0, -hw, 0, x1, hw, DHT_BAY_TOP + 1),
        box(xb0 - c, -w - c, DHT_Z_FACE, DHT_X_TAB + c, w + c, 0.5),  # body pocket
    ]
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
    ribs = [
        box(
            x - 0.6,
            s * (w - DHT_RIB),
            DHT_Z_FACE + 0.3,
            x + 0.6,
            s * (w + c + 0.01),
            -0.2,
        )
        for x in (xb0 + 6.0, DHT_X_TAB - 6.0)
        for s in (-1, 1)
    ]
    return walls, cuts, openings, ribs


def mz_bay():
    """MH-Z19B bay (bay coordinates): chamber pocket with vents, ledges for the PCB ends, bosses for the frame pins."""
    hx, hy, t = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR, BAY_WALL
    bosses = [(s * MZ_BOSS_X, s * (hy + t + BOSS_R - 1.0)) for s in (-1, 1)]
    walls = fuse_all(
        [box(-hx - t, -hy - t, 0, hx + t, hy + t, MZ_BAY_TOP)]
        + [cyl(bx, by, 0, BOSS_R, MZ_BAY_TOP) for bx, by in bosses]
        + [
            cyl(bx, by, MZ_BAY_TOP - 0.01, MZ_PIN_D / 2, MZ_PIN_H + 0.01)
            for bx, by in bosses
        ]
    )
    cx = MZ_CH_L / 2 + MZ_CLR
    drop = MZ_LEDGE_DROP if MZ_RIM_W > 0 else 0.0
    cuts = [
        box(-cx, -hy, 0, cx, hy, MZ_Z_PCB + 0.01),
        box(-hx, -hy, MZ_Z_PCB - drop, hx, hy, MZ_BAY_TOP + 1),
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
    return walls, cuts, openings, bosses


def mz_rim():
    """Bay coordinates: rim on the deck round the vents, up to the chamber face; the chamber rests on it, so the
    sensor takes in air only through the vents, not from the rest of the bay. None with MZ_RIM_W = 0."""
    if MZ_RIM_W <= 0:
        return None
    ox, oy, w = MZ_CH_L / 2 - MZ_RIM_IN, MZ_CH_W / 2 - MZ_RIM_IN, MZ_RIM_W
    z1 = MZ_GAP + MZ_BUMP_H  # chamber face
    return rbox(-ox, -oy, -0.01, ox, oy, z1, 0.6).cut(
        rbox(-ox + w, -oy + w, -1, ox - w, oy - w, z1 + 1, 0.3)
    )


def esp_tray():
    """Bay coordinates, on top of the MH-Z19B frame: two rails under the ESP32 back face (clear of the solder joints
    along the pin rows) and a low rim round the board, below its top face at any PCB thickness."""
    x0, y0 = -PCB_W / 2, ESP_DY - PCB_L / 2
    z0, zb = MZ_BAY_TOP + MZ_FRAME_T - 0.01, ESP_Z0
    c, t = 0.3, 1.2
    rim = rbox(
        x0 - c - t,
        y0 - c - t,
        z0,
        x0 + PCB_W + c + t,
        y0 + PCB_L + c + t,
        zb + 0.8,
        1.5,
    ).cut(rbox(x0 - c, y0 - c, z0 - 1, x0 + PCB_W + c, y0 + PCB_L + c, zb + 2, 1.0))
    rails = [
        box(x0 + u0, y0 + 0.5, z0, x0 + u1, y0 + PCB_L - 0.5, zb)
        for u0, u1 in ((4.0, 6.0), (12.0, 14.0))
    ]
    return fuse_all([rim] + rails)


def tray_plate():
    """Bay coordinates: the part of the frame plate under the ESP32 tray."""
    x0, y0, g = -PCB_W / 2 - 1.5, ESP_DY - PCB_L / 2 - 1.5, 3.0
    z = MZ_BAY_TOP
    return box(x0, y0, z, -x0, y0 + PCB_L + g, z + MZ_FRAME_T)


def lead_holes():
    """World xy of the holes for the leads through the deck, one per lead."""
    return [
        V(x + i * dx, y + i * dy, 0)
        for (x, y), (dx, dy), n in LEAD_ROWS
        for i in range(n)
    ]


def deck_holes():
    r = (LEAD_D + LEAD_CLR) / 2
    return [cyl(q.x, q.y, HOUSING_Z - 1, r, DECK_T + 2) for q in lead_holes()]


def cable_slot():
    """World: slot for the 5 V cable through the wall and the lid rim, from CABLE_Z0 up, deep enough that the cable
    stays inside the hole while the unit goes in."""
    s = box(
        BODY_R - CABLE_W,
        -CABLE_W / 2,
        CABLE_Z0,
        BODY_R + 1,
        CABLE_W / 2,
        TOP_Z + LID_T + 1,
    )
    return rot_z(s, CABLE_ANGLE)


def trim_vents():
    """World: room-air slots through the trim, clear of the DHT22 bay, the pod cradle and the clamp columns."""
    x0, x1, hw = DHT_BAY
    t = BAY_WALL + 1.0
    g = END_Y0 + END_T + 1.0
    keep_out = [
        moved(box(x0 - t, -hw - t, -10, x1 + t, hw + t, 10), dht_plc()),
        moved(box(-END_X - 1.0, -g, -10, END_X + 1.0, g, 10), cradle_plc()),
    ] + [moved(cyl(0, 0, -10, COL_R_OUT + 1.0, 20), clamp_plc(a)) for a in CLAMP_ANGLES]
    arcs = fuse_all(
        [
            rot_z(sector(VENT_R0, VENT_R1, -BEZEL_T - 1, BEZEL_T + 1.5, e - s), s)
            for s, e in vent_arcs()
        ]
    ).cut(fuse_all(keep_out))
    # drop slivers (shorter than about 6 mm)
    return [s for s in arcs.Solids if s.Volume > 6.0 * 2.0 * (BEZEL_T + 1.5)]


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


# ------------------------------------------------------------------ case parts
def build_body(ribs=True, bump=True):
    """Housing: trim, wall, clamp columns with the spindle grooves, DHT22 bay, pod cradle. `ribs` / `bump`: the DHT22
    crush ribs and the detent bump of the leaf, which touch other parts by design."""
    bezel = cyl(0, 0, -BEZEL_T, BEZEL_R, BEZEL_T)
    bezel = fillet_where(bezel, 1.5, lambda a, b: abs(a.z + BEZEL_T) < 1e-6)
    bezel = fillet_where(
        bezel, 0.6, lambda a, b: abs(a.z) < 1e-6 and a.Length > BEZEL_R - 1
    )
    disc = cyl(0, 0, 0, BODY_R, HOUSING_Z)
    adds = [bezel, disc.cut(cyl(0, 0, -1, BODY_R - WALL, HOUSING_Z + 2))]
    adds += [
        moved(column(COL_R_IN, COL_R_OUT, 0, HOUSING_Z), clamp_plc(a))
        for a in CLAMP_ANGLES
    ]
    walls, bcuts, bopen, bribs = dht_bay()
    adds.append(moved(walls, dht_plc()))
    cuts = [moved(c, dht_plc()) for c in bcuts + bopen]
    cw, cbump, ccuts = cradle()
    adds += [moved(s, cradle_plc()) for s in cw]
    cuts += [moved(c, cradle_plc()) for c in ccuts]
    cuts += trim_vents()
    body = fuse_all(adds).cut(fuse_all(cuts))
    gr = groove()
    bores = []
    for a in (
        CLAMP_ANGLES
    ):  # spindle bores through the trim and the columns, with their grooves
        p = clamp_plc(a)
        bores.append(
            moved(cyl(0, 0, -BEZEL_T - 1, SP_BORE, HOUSING_Z + BEZEL_T + 2), p)
        )
        bores.append(moved(gr, p))
    body = body.cut(fuse_all(bores))
    extras = [moved(r, dht_plc()) for r in bribs] if ribs else []
    if bump:
        extras.append(moved(cbump, cradle_plc()))
    if extras:
        body = body.fuse(extras)
    return solid(body.removeSplitter())


def build_tower():
    """Deck (MH-Z19B bay over the plenum) + wall up to the lid. Per clamp: a column round the spindle under the arm
    and the arm sweep cut out of the wall; lid columns with the pin holes; power cable slot at the top of the wall."""
    z0 = HOUSING_Z
    shell = cyl(0, 0, z0, BODY_R, TOP_Z - z0).cut(
        cyl(0, 0, DECK_Z, BODY_R - WALL, TOP_Z)
    )
    walls, bcuts, bopen, _ = mz_bay()
    adds = [shell, moved(walls, mz_plc())]
    cuts = [moved(c, mz_plc()) for c in bcuts]
    openings = [moved(c, mz_plc()) for c in bopen] + deck_holes()
    for a in CLAMP_ANGLES:
        p = clamp_plc(a)
        adds.append(moved(column(COL_R_IN, TCOL_R, DECK_Z - 0.01, SWEEP_Z0), p))
        cuts.append(moved(arm_sweep(ARM_CLR, SWEEP_Z0, TOP_Z + 1), p))
        cuts.append(moved(cyl(0, 0, z0 - 1, SP_BORE, SWEEP_Z0 + 2 - z0), p))
        cuts.append(moved(lug_channel(z0 - 1, SWEEP_Z0 + 1), p))
    for q in lid_cols():
        adds.append(cyl(q.x, q.y, DECK_Z - 0.01, LID_COL_R, TOP_Z - DECK_Z + 0.01))
        cuts.append(
            cyl(q.x, q.y, TOP_Z - LID_PIN_L - 0.5, LID_PIN_D / 2 + 0.1, LID_PIN_L + 1.5)
        )
    cuts.append(cable_slot())
    tower = fuse_all(adds).cut(fuse_all(cuts))
    rim = mz_rim()
    if rim is not None:
        tower = tower.fuse(moved(rim, mz_plc()))
    tower = tower.cut(fuse_all(openings))
    return solid(tower.removeSplitter())


def build_pod(grooves=True):
    """HMMD frame (radome face on z = 0, the way it prints): barrel with the trunnions, the detent grooves on the
    high side, square pocket with the HMMD ledges, open at the top for the keeper."""
    body = pod_barrel().fuse(trunnions())
    if grooves:
        lift = App.Placement(V(0, 0, POD_H), App.Rotation())
        body = body.cut(fuse_all([moved(g, lift) for g in detent_grooves()]))
    pocket = box(-PKT, -PKT, Z_POCKET, PKT, PKT, 60).cut(
        fuse_all([rail_box(*q) for q in HM_RAILS.values()])
    )
    return solid(body.cut(pocket).removeSplitter())


def build_keeper(ribs=True):
    """HMMD frame: square tube pushed into the pocket onto the HMMD back (clear of every part, open over J2), top
    flush with the pod; crush ribs hold it."""
    zb = KEEPER_Z0
    o = PKT - KEEPER_CLR
    i = o - KEEPER_WALL
    parts = [box(-o, -o, zb, o, o, POD_TOP).cut(box(-i, -i, zb - 1, i, i, POD_TOP + 1))]
    if ribs:  # crush ribs: 0.15 mm interference, so the keeper stays in at any tilt
        h = KEEPER_CLR + 0.15
        z0, z1 = zb + 1.5, POD_TOP - 0.5
        parts += [
            box(-o - h, -8.0, z0, -o + 0.01, -7.0, z1),
            box(1.5, o - 0.01, z0, 2.5, o + h, z1),
            box(1.5, -o - h, z0, 2.5, -o + 0.01, z1),
        ]
    # the J2 leads stay outside the keeper
    xj = HM_X0 + HM_PIN_X + HDR_W / 2 + 0.3
    keeper = fuse_all(parts).cut(box(-o - 1, HM_Y0 + 6.0, zb - 1, xj, o + 1, 80))
    return solid(keeper.removeSplitter())


def build_arm():
    """Arm coordinates (spindle axis on z, arm along +x, z = 0 on the arm underside): arm, spindle down to the trim,
    the lug, and a cross slot in the spindle end. Printed arm top down."""
    z_end = SP_END_Z - ARM_PARK_Z
    lug = moved(
        lug_solid(),
        App.Placement(V(0, 0, LUG_Z - ARM_PARK_Z), App.Rotation(V(0, 0, 1), LUG_ANGLE)),
    )
    arm = fuse_all(
        [arm_outline(0, 0, ARM_T), cyl(0, 0, z_end, SP_R, -z_end + 0.01), lug]
    )
    w, d = SP_SLOT_W / 2, SP_R + 1
    cross = box(-d, -w, z_end - 1, d, w, z_end + SP_SLOT_D).fuse(
        box(-w, -d, z_end - 1, w, d, z_end + SP_SLOT_D)
    )
    return solid(arm.cut(cross).removeSplitter())


def build_lid(ribs=True):
    """Closes the tower: pins with crush ribs into the lid columns, notch for the power cable."""
    z0 = TOP_Z
    lid = cyl(0, 0, z0, BODY_R, LID_T)
    pins = [
        crush_pin(q.x, q.y, z0 - LID_PIN_L, LID_PIN_D, LID_PIN_L + 0.01, LID_RIB)
        if ribs
        else cyl(q.x, q.y, z0 - LID_PIN_L, LID_PIN_D / 2, LID_PIN_L + 0.01)
        for q in lid_cols()
    ]
    return solid(lid.fuse(pins).cut(cable_slot()).removeSplitter())


def build_mz_frame():
    """Bay coordinates: frame over the MH-Z19B (openings that fit round its pin spacers, so it closes the bay off
    from the tower; holes for the locating pins), the ESP32 tray and two posts that the lid presses down; clear of
    the spindle columns."""
    hx, hy, t, z = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR, BAY_WALL, MZ_BAY_TOP
    zt = z + MZ_FRAME_T
    bosses = mz_bay()[3]
    post_h = TOP_Z - 0.1 - DECK_Z - z
    frame = fuse_all(
        [box(-hx - t, -hy - t, z, hx + t, hy + t, zt), tray_plate(), esp_tray()]
        + [cyl(bx, by, z, BOSS_R, MZ_FRAME_T) for bx, by in bosses]
        + [cyl(px, py, z, MZ_POST_R, post_h) for px, py in MZ_POSTS]
    )
    cuts = [
        cyl(bx, by, z - 1, MZ_PIN_D / 2 + 0.15, MZ_FRAME_T + 2) for bx, by in bosses
    ]
    c, h = MZ_SPACER_CLR, PIN_PITCH / 2
    for s, n in zip((-1, 1), MZ_PIN_N):  # the leads on the pin ends go up through these
        x, y1 = s * MZ_PIN_X, MZ_PIN_Y0 + PIN_PITCH * (n - 1)
        cuts.append(
            box(x - h - c, MZ_PIN_Y0 - h - c, z - 1, x + h + c, y1 + h + c, zt + 3)
        )
    inv = mz_plc().inverse()
    cuts += [
        moved(
            column(COL_R_IN + 0.3, TCOL_R + 0.3, DECK_Z - 1, TOP_Z + 1),
            inv.multiply(clamp_plc(a)),
        )
        for a in CLAMP_ANGLES
    ]
    return solid(frame.cut(fuse_all(cuts)).removeSplitter())


def build_ceiling(t=CEIL_T):
    """Lower layer of thickness t with the hole + the solid upper ceiling above the gap."""
    s = BEZEL_R + 20
    lower = box(-s, -s, 0, s, s, t).cut(cyl(0, 0, -1, HOLE_R, t + 2))
    return lower.fuse(box(-s, -s, ROOF_Z, s, s, ROOF_Z + 10))


# ------------------------------------------------------------------ document
SPECS = [
    (
        "Ceiling",
        "hole %g mm across through a %g mm lower layer, then a gap, solid upper ceiling %g mm up; clamps grip %g-%g mm"
        % (2 * HOLE_R, CEIL_T, ROOF_Z, CEIL_T_MIN, CEIL_T_MAX),
        "user spec (measured)",
    ),
    (
        "Size",
        "trim %g mm across; housing + sensor tower %g mm across, %.1f mm above the ceiling surface (%.1f from the trim face), %.1f mm under the upper ceiling"
        % (
            2 * BEZEL_R,
            2 * BODY_R,
            TOP_Z + LID_T,
            BEZEL_T + TOP_Z + LID_T,
            ROOF_Z - TOP_Z - LID_T,
        ),
        "design result",
    ),
    (
        "Hardware",
        "none: every part is printed; nothing but the four modules, their leads and the 5 V cable goes in",
        "user spec",
    ),
    (
        "Clamps",
        "3 arms printed in one piece with their spindles, reaching %g mm from the axis (%g mm past the hole edge, v1: 8). The first %g deg swing the arm out level, the next %g deg run a lug down a helical groove (%.1f mm) that pulls it onto the layer top"
        % (
            CLAMP_R + ARM_L,
            CLAMP_R + ARM_L - HOLE_R,
            ARM_DWELL_END - ARM_TUCK,
            ARM_RAMP_END - ARM_DWELL_END,
            ARM_DROP,
        ),
        "user spec + design choice",
    ),
    (
        "Joints",
        "tower held on the housing by the 3 spindles; lid: 3 pins with crush ribs; MH-Z19B frame: 2 locating pins, pressed down by the lid; DHT22 and HMMD keeper: crush ribs; HMMD pod: trunnions under a yoke held by the deck",
        "design choice",
    ),
    (
        "Power",
        "5 V cable through the slot at the top of the wall (in the gap), soldered to the ESP32 5V and GND pins",
        "user choice",
    ),
    (
        "Room air",
        "slots in the trim by the wall feed the space under the deck, where the MH-Z19B vents open; that space is closed off from the tower (one deck hole per lead, frame tight round the MH-Z19B pin spacers) and the chamber rests on a rim round its vents, so the sensor gets room air and not ceiling-void air; the DHT22 has its own window in the trim",
        "design choice",
    ),
    (
        "HMMD mount",
        "barrel pod on two trunnions, radome face tilts %g-%g deg toward %g deg with a click every %g deg (leaf spring + grooves); turn the unit in the hole to aim it"
        % (POD_TILT_MIN, POD_TILT_MAX, POD_LOOK, POD_STEP),
        "user spec + design choice",
    ),
    (
        "Board",
        "ESP32-C3 SuperMini Plus (V2), red PCB, 22.52 x 18 mm; flat on a tray on the MH-Z19B frame, USB-C only for flashing; PCB 1.0-1.6 mm checked",
        "datasheet + illustration",
    ),
    (
        "Wiring",
        "no headers or DuPont plugs: leads soldered straight into the ESP32 and HMMD pin holes and onto the DHT22 / MH-Z19B pins; the DHT22 and HMMD leads go up through the deck one per hole, sized for %g mm leads"
        % LEAD_D,
        "user choice",
    ),
    (
        "HMMD",
        "Waveshare HMMD (S3KM1110, 24 GHz) 20 x 20 mm, J2 pads on the back; radome 0.8 mm, antenna gap 6.2 mm (half wavelength)",
        "Waveshare wiki + radome guide",
    ),
    (
        "HMMD thickness",
        "NOT published. Modelled 1.0; pod checked for 0.8-1.6 mm",
        "assumption - measure yours",
    ),
    (
        "DHT22",
        "AM2302 body 15.1 x 25.1 x 7.7 mm on a 13 x 28.2 mm black PCB, right-angle 3-pin header (- out +), 43 mm overall",
        "AM2302 datasheet, retailer drawing",
    ),
    (
        "DHT22 fit",
        "grille flush in a 13.6 x 23.6 mm window of the trim (0.6 mm lip), held by 4 crush ribs; body 7.4-8.0 mm checked",
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
        "housing trim-down, tower deck-down, HMMD pod radome face-down (smooth bed), arms arm-top-down, lid top-down, yoke on its side, keeper and frame as exported; no supports",
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


def module_parts(tilt=POD_TILT_SHOW):
    """Reference models in world coordinates: {group: {key: (label, shape, rgb)}}."""
    out = {}
    for grp, builder, plc in (
        ("ESP32", build_esp32, esp_plc()),
        ("HMMD", build_hmmd, hm_plc(tilt)),
        ("DHT22", build_dht, dht_plc()),
        ("MHZ19B", build_mz, mz_plc()),
    ):
        out[grp] = {
            k: (lab, moved(s, plc), rgb) for k, (lab, s, rgb) in builder().items()
        }
    return out


def build():
    if DOC_NAME in App.listDocuments():
        App.closeDocument(DOC_NAME)
    doc = App.newDocument(DOC_NAME)
    arm = build_arm()
    psi = arm_psi(ARM_PARK_Z - CEIL_T)  # clamping the nominal layer
    hp = hm_plc()
    world = {
        "Body": (
            "Housing: trim + spindle columns + DHT22 bay + HMMD pod cradle",
            build_body(),
            (0.93, 0.93, 0.92),
            0,
        ),
        "Tower": (
            "Sensor tower: deck + MH-Z19B bay + spindle columns + lid columns",
            build_tower(),
            (0.85, 0.88, 0.93),
            0,
        ),
        "Pod": (
            "HMMD pod (radome face tilted %g deg, %g-%g)"
            % (POD_TILT_SHOW, POD_TILT_MIN, POD_TILT_MAX),
            moved(build_pod(), hp),
            (0.97, 0.97, 0.97),
            0,
        ),
        "Keeper": ("HMMD keeper", moved(build_keeper(), hp), (0.30, 0.60, 0.90), 0),
        "Yoke": (
            "Pod yoke (holds the trunnions, held by the deck)",
            moved(build_yoke(), cradle_plc()),
            (0.30, 0.60, 0.90),
            0,
        ),
        "Lid": (
            "Top lid (pins with crush ribs, power cable notch)",
            build_lid(),
            (0.30, 0.60, 0.90),
            55,
        ),
        "MZ_Frame": (
            "MH-Z19B bay frame + ESP32 tray",
            moved(build_mz_frame(), mz_plc()),
            (0.30, 0.60, 0.90),
            0,
        ),
    }
    for i, a in enumerate(CLAMP_ANGLES):
        world["Arm_%d" % i] = (
            "Clamp arm + spindle %d (clamping a %g mm layer)" % (i + 1, CEIL_T),
            arm_at(arm, a, psi),
            (0.95, 0.55, 0.15),
            0,
        )
    for k, (lab, s, rgb, tr) in world.items():
        new_part(doc, "Case_" + k, lab, s, rgb, transparency=tr)
    refs = module_parts()
    for grp, parts in refs.items():
        g = doc.addObject("App::DocumentObjectGroup", grp + "_Reference")
        g.Label = grp + " - reference model"
        for k, (lab, s, rgb) in parts.items():
            new_part(doc, grp + "_" + k, lab, s, rgb, group=g)
    g = doc.addObject("App::DocumentObjectGroup", "Ceiling_Reference")
    g.Label = (
        "Ceiling - reference (%g mm layer, hole %g mm, upper ceiling at %g mm)"
        % (CEIL_T, 2 * HOLE_R, ROOF_Z)
    )
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
    return doc, {k: v[1] for k, v in world.items()}, arm


def max_radius(shape, n=36):
    return max(math.hypot(p.x, p.y) for e in shape.Edges for p in e.discretize(n))


def verify(world, arm):
    """Numeric checks: overlaps, thickness sweeps, clamp swing / grip / groove, HMMD pod tilt and clicks, radome,
    openings."""
    fails = []

    def check(ok, msg):
        if not ok:
            fails.append(msg)

    def inside(shape, p):
        return shape.isInside(p, 1e-6, False)

    body, lid, tower = world["Body"], world["Lid"], world["Tower"]
    hp = hm_plc()
    # the crush ribs and the detent bump touch other parts by design: they are measured on their own
    body0 = build_body(ribs=False, bump=False)
    case = dict(world)
    case["Body"] = body0
    case["Keeper"] = moved(build_keeper(ribs=False), hp)
    case["Lid"] = build_lid(ribs=False)
    names = list(case)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            v = ovl(case[a], case[b])
            check(v < 1e-4, "%s and %s overlap (%.4f mm3)" % (a, b, v))
    rib_k = ovl(world["Keeper"], world["Pod"])
    rib_l = ovl(lid, tower)
    rib_d = ovl(body, moved(build_dht()["Body"][1], dht_plc()))
    print(
        "crush ribs (by design): keeper in the pod %.2f mm3, lid pins %.2f mm3, DHT22 body %.2f mm3"
        % (rib_k, rib_l, rib_d)
    )
    check(0.5 < rib_k < 10, "keeper crush ribs missing or too big")
    check(0.5 < rib_l < 10, "lid pin crush ribs missing or too big")
    check(0.5 < rib_d < 10, "DHT22 crush ribs missing or too big")

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
        g: Part.makeCompound([s for _, s, _ in p.values()]) for g, p in refs.items()
    }
    gl = list(comp)
    for i, a in enumerate(gl):
        for b in gl[i + 1 :]:
            check(ovl(comp[a], comp[b]) < 1e-4, "modules %s and %s collide" % (a, b))
    ceil = build_ceiling()
    for cn, cs in case.items():
        check(ovl(ceil, cs) < 1e-4, "%s cuts into the ceiling" % cn)
    # nothing may come closer than TOP_CLEAR to the upper ceiling, parked arms included
    top = max(
        [s.BoundBox.ZMax for s in case.values()]
        + [s.BoundBox.ZMax for p in refs.values() for _, s, _ in p.values()]
        + [arm_at(arm, CLAMP_ANGLES[0], ARM_TUCK).BoundBox.ZMax]
    )
    print(
        "unit top z=%.2f (lid top %.2f), upper ceiling at %g: %.2f mm clear"
        % (top, TOP_Z + LID_T, ROOF_Z, ROOF_Z - top)
    )
    check(
        top <= ROOF_Z - TOP_CLEAR + 1e-6,
        "unit comes within %g mm of the upper ceiling" % TOP_CLEAR,
    )

    print("ESP32 PCB thickness sweep (flat on the tray of the MH-Z19B frame):")
    frame = world["MZ_Frame"]
    for t in (PCB_T_MIN, 1.2, 1.4, PCB_T_MAX):
        parts = {k: moved(s, esp_plc()) for k, (_, s, _) in build_esp32(t).items()}
        v = sum(
            ovl(frame, s) + ovl(case["Lid"], s) + ovl(tower, s) for s in parts.values()
        )
        zt = max(s.BoundBox.ZMax for s in parts.values())
        print(
            "  T=%.1f overlap=%.4f | top of the board + leads %.2f, lid underside %.2f"
            % (t, v, zt, TOP_Z)
        )
        check(v < 1e-4, "ESP32 interference at PCB thickness %.1f" % t)
    pod0, keep0 = build_pod(), build_keeper(ribs=False)
    print("HMMD PCB thickness sweep (in the pod):")
    for t in (HM_T_MIN, HM_T, 1.2, HM_T_MAX):
        v = sum(ovl(pod0, s) + ovl(keep0, s) for _, s, _ in build_hmmd(t).values())
        print(
            "  T=%.1f overlap=%.4f | board can lift %.2f mm before the keeper"
            % (t, v, HM_T_MAX + 0.1 - t)
        )
        check(v < 1e-4, "HMMD interference at PCB thickness %.1f" % t)
    dp = dht_plc()
    print("DHT22 body thickness sweep (grille in the trim window):")
    for bt in (DHT_BODY_T_MIN, DHT_BODY_T, DHT_BODY_T_MAX):
        parts = {k: moved(s, dp) for k, (_, s, _) in build_dht(bt).items()}
        v = sum(ovl(body0, s) + ovl(tower, s) for s in parts.values())
        top = max(s.BoundBox.ZMax for k, s in parts.items() if k != "Wires")
        print(
            "  T=%.1f overlap=%.4f | module top %.2f above the trim, deck at %.2f"
            % (bt, v, top, HOUSING_Z)
        )
        check(v < 1e-4, "DHT22 interference at body thickness %.1f" % bt)

    # ---- clamp arms: swing, groove, grip
    n = int(round((ARM_RAMP_END - ARM_TUCK) / 5.0))
    psis = [ARM_TUCK + (ARM_RAMP_END - ARM_TUCK) * i / n for i in range(n + 1)]
    sweep = sorted(
        set(psis + [p + GROOVE_STEP / 2 for p in psis[:-1]] + [ARM_DWELL_END])
    )
    layer_max = build_ceiling(CEIL_T_MAX)
    psi_nom = arm_psi(ARM_PARK_Z - CEIL_T)
    for i, a in enumerate(CLAMP_ANGLES):
        loc = moved(cyl(0, 0, -BEZEL_T - 1, ARM_L + 12, ROOF_Z + 1), clamp_plc(a))
        near = [
            s.common(loc)
            for s in (body0, tower, case["Lid"], world["MZ_Frame"], world["Yoke"])
        ]
        others = [
            arm_at(arm, b, p)
            for b in CLAMP_ANGLES
            if b != a
            for p in (ARM_TUCK, psi_nom)
        ]

        def hit(s):
            return sum(ovl(q, s) for q in near)

        for p in sweep:
            s = arm_at(arm, a, p)
            v = hit(s)
            check(v < 1e-4, "arm %d hits the case at %.1f deg (%.4f)" % (i + 1, p, v))
            v = sum(ovl(o, s) for o in others)
            check(v < 1e-4, "arm %d hits another arm at %.1f deg" % (i + 1, p))
            if p <= ARM_DWELL_END:
                v = ovl(layer_max, s)
                check(
                    v < 1e-4,
                    "arm %d cannot swing out over a %g mm layer (%.1f deg)"
                    % (i + 1, CEIL_T_MAX, p),
                )
        # the lug holds the spindle: no axial play in the groove (only the lid stops it at the way in)
        for p in (ARM_TUCK + 10.0, -80.0, ARM_DWELL_END, 0.0, ARM_RAMP_END):
            for dz in (0.6, -0.6):
                v = ovl(body0, arm_at(arm, a, p, arm_drop(p) - dz))
                check(
                    v > 1e-3,
                    "arm %d spindle not held in its groove at %g deg (%+g mm)"
                    % (i + 1, p, dz),
                )
        v = ovl(case["Lid"], arm_at(arm, a, ARM_TUCK, -ARM_TOP_GAP - 0.3))
        check(v > 1e-3, "lid does not keep arm %d from lifting out" % (i + 1))
    print("clamp grip on the top of the layer (v1: 8.0 mm at any thickness):")
    a = CLAMP_ANGLES[0]
    for t in (CEIL_T_MIN, 29.0, 29.5, CEIL_T, CEIL_T_MAX):
        d = ARM_PARK_Z - t
        p = arm_psi(d)
        s = arm_at(arm, a, p, d)
        layer = build_ceiling(t)
        grip = max_radius(s) - HOLE_R
        foot = (
            s.common(box(-99, -99, t, 99, 99, t + 0.2))
            .cut(cyl(0, 0, t - 1, HOLE_R, 3))
            .Volume
            / 0.2
        )
        print(
            "  layer %.1f mm: arm at %+6.1f deg, %.1f mm below parked, grips %.1f mm past the hole edge, %.0f mm2 on the layer"
            % (t, p, d, grip, foot)
        )
        check(ovl(layer, s) < 1e-4, "arm cuts into a %g mm layer" % t)
        check(
            ovl(layer, arm_at(arm, a, p, d + 0.2)) > 1e-3,
            "arm does not reach a %g mm layer" % t,
        )
        check(
            grip >= ARM_GRIP_MIN, "arm grips only %.1f mm of a %g mm layer" % (grip, t)
        )
        check(foot >= 40.0, "arm rests on only %.0f mm2 of a %g mm layer" % (foot, t))
    rt = max_radius(arm_at(arm, a, ARM_TUCK))
    r_m = SP_R + LUG_D / 2
    slope = math.degrees(math.atan(ARM_DROP / (r_m * RAMP_SPAN * D2R)))
    lock = math.degrees(
        math.atan(0.3 * math.sqrt(2))
    )  # 45 deg flank, mu ~0.3 printed on printed
    print(
        "clamp arm: tucked in %.2f from the axis (hole %.1f) | reach %.1f, %.0f %% more grip than v1 | ramp %.1f mm over %g deg = %.1f deg at the lug (holds up to %.1f)"
        % (
            rt,
            HOLE_R,
            CLAMP_R + ARM_L,
            100 * ((CLAMP_R + ARM_L - HOLE_R) / 8.0 - 1),
            ARM_DROP,
            RAMP_SPAN,
            slope,
            lock,
        )
    )
    print(
        "spindle end %.2f below the trim face parked, %.2f at the end of the ramp"
        % (SP_END_Z + BEZEL_T, SP_END_Z - ARM_DROP + BEZEL_T)
    )
    # assembly: with the lid off, each arm goes in from above, tucked in, its lug down the channels to the groove
    for i, a in enumerate(CLAMP_ANGLES):
        for dz in (1.0, 3.0, 6.0, 10.0, 16.0, 24.0):
            s = arm_at(arm, a, ARM_TUCK, -dz)
            v = ovl(body0, s) + ovl(tower, s)
            check(
                v < 1e-4,
                "arm %d cannot be put in from above (%g mm up: %.4f)" % (i + 1, dz, v),
            )
    check(rt <= HOLE_R - 1.0, "tucked-in arms do not pass through the hole")
    check(slope <= lock - 1.0, "groove ramp too steep to hold the clamp")
    check(SP_END_Z - ARM_DROP >= -BEZEL_T + 0.3, "spindle end comes out of the trim")
    for k, s in (("housing", body), ("tower", tower), ("lid", lid)):
        check(
            max_radius(s.common(cyl(0, 0, 0.1, 100, TOP_Z + LID_T))) <= HOLE_R - 0.9,
            "%s too big for the hole" % k,
        )

    # ---- HMMD pod: tilt range, clicks, retention, field of view
    hm0 = Part.makeCompound([s for _, s, _ in build_hmmd().values()])
    asm = Part.makeCompound([pod0, keep0, hm0])
    loc = moved(box(-25, -20, -20, 25, 20, 20), cradle_plc())
    near = [s.common(loc) for s in (body0, tower, world["Yoke"])]
    tilts = sorted(POD_TILTS + [t + POD_STEP / 2 for t in POD_TILTS[:-1]])
    for t in tilts:
        v = sum(ovl(q, moved(asm, hm_plc(t))) for q in near)
        check(v < 1e-4, "HMMD pod hits the case at %g deg (%.4f)" % (t, v))
    bump = moved(cradle()[1], cradle_plc())
    for t in POD_TILTS:
        check(
            ovl(bump, moved(pod0, hm_plc(t))) < 1e-4,
            "detent bump not in its groove at %g deg" % t,
        )
    mid = [ovl(bump, moved(pod0, hm_plc(t + POD_STEP / 2))) for t in POD_TILTS[:-1]]
    print(
        "pod clicks: bump clear in its groove at %s deg, %.2f-%.2f mm3 into the barrel between them"
        % ("/".join("%g" % t for t in POD_TILTS), min(mid), max(mid))
    )
    check(min(mid) > 1e-3, "detent bump does not press on the barrel between clicks")
    # the leaf is held only at its ends: free behind it and under it, so it can spring back
    x0 = leaf_x0()
    zc0, zc1 = LEAF_Z0 - POD_AXIS_Z, LEAF_Z1 - POD_AXIS_Z
    behind = box(
        x0 + LEAF_T + 0.05, -POD_HW + 1, zc0, x0 + LEAF_T + 0.6, POD_HW - 1, zc1
    )
    under = box(x0, -POD_HW + 1, -POD_AXIS_Z + 0.2, x0 + LEAF_T, POD_HW - 1, zc0 - 0.2)
    for k, s in (("behind", behind), ("under", under)):
        check(ovl(body, moved(s, cradle_plc())) < 1e-4, "no free room %s the leaf" % k)
    span = 2 * END_Y0
    print(
        "leaf spring: %.1f mm long, %.1f thick, bends %.2f mm per click (%.2f %% strain), free under it %.1f mm"
        % (span, LEAF_T, DET_PRESS, 100 * 12 * DET_PRESS * LEAF_T / span**2, LEAF_Z0)
    )
    # assembly: the pod drops into its cradle from above (the leaf gives way to the barrel)
    worst = 0.0
    for dz in (1.0, 3.0, 6.0, 10.0, 15.0):
        plc = App.Placement(V(0, 0, dz), App.Rotation()).multiply(hm_plc(POD_TILT_MIN))
        s = moved(asm, plc)
        check(ovl(body0, s) < 1e-4, "pod cannot be put in from above (%g mm up)" % dz)
        worst = max(worst, ovl(bump, s))
    print(
        "pod goes in from above; the barrel pushes the leaf bump aside (%.2f mm3)"
        % worst
    )
    rot = cradle_plc().Rotation
    for t in (POD_TILT_MIN, POD_TILT_MAX):
        for d in (
            V(0, 0, 0.3),
            V(0, 0, -0.3),
            V(0, 0.5, 0),
            V(0, -0.5, 0),
            V(0.5, 0, 0),
            V(-0.5, 0, 0),
        ):
            plc = App.Placement(rot.multVec(d), App.Rotation()).multiply(hm_plc(t))
            v = sum(ovl(q, moved(pod0, plc)) for q in near)
            check(v > 1e-3, "pod not held at %g deg when pushed by %s" % (t, d))
    look, side = polar(1, POD_LOOK), polar(1, POD_LOOK + 90)
    print(
        "HMMD pod: tilt | lowest point below the trim face | high edge of the radome face below the trim face"
    )
    for t in POD_TILTS:
        plc = hm_plc(t)
        low = -BEZEL_T - moved(pod0, plc).BoundBox.ZMin
        edge = -BEZEL_T - plc.multVec(V(POD_HW, 0, 0)).z
        print("  %4.1f deg | %5.1f mm | %+5.2f mm" % (t, low, edge))
        c = plc.multVec(V(0, 0, 0))
        st, ct = math.sin(t * D2R), math.cos(t * D2R)
        nrm = V(look.x * st, look.y * st, -ct)  # outward face normal
        tw = (
            min(60.0, 85.0 - t) + t
        )  # toward the tilt: 60 deg off the normal, at most 85 deg from vertical
        for k, d in (
            (
                "toward the tilt",
                look * math.sin(tw * D2R) + V(0, 0, -math.cos(tw * D2R)),
            ),
            (
                "away from the tilt",
                look * -math.sin((60 - t) * D2R) + V(0, 0, -math.cos((60 - t) * D2R)),
            ),
            ("sideways", nrm * math.cos(60 * D2R) + side * math.sin(60 * D2R)),
            ("other side", nrm * math.cos(60 * D2R) - side * math.sin(60 * D2R)),
        ):
            ray = Part.makeCylinder(0.3, 80, c + d * 1.0, d)
            check(
                ovl(body, ray) < 1e-4, "housing in the HMMD view %s at %g deg" % (k, t)
            )

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
            ok = inside(pod0, V(x, y, 0.05)) and inside(pod0, V(x, y, Z_POCKET - 0.05))
            bad += 0 if ok and not inside(pod0, V(x, y, Z_POCKET + 0.05)) else 1
    print(
        "radome: %d sample points, %d off (floor %.2f mm, air gap %.2f mm)"
        % (n, bad, RADOME_T, HM_H)
    )
    check(bad == 0, "radome floor is not a uniform %.2f mm" % RADOME_T)
    for k, (u0, u1, v0, v1) in HM_RAILS.items():
        c = V(HM_X0 + (u0 + u1) / 2, HM_Y0 + (v0 + v1) / 2, Z_HM_FRONT)
        check(
            inside(pod0, c - V(0, 0, 0.05)) and not inside(pod0, c + V(0, 0, 0.05)),
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
            ovl(pod0, kb) < 1e-6,
            "pod material within %.1f mm of antenna patch %s" % (m, k),
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
    # the vents open into the plenum: through the deck, and nothing within 2 mm under it
    vent = fuse_all([moved(c, mz_plc()) for c in mz_bay()[2]])
    full = vent.common(cyl(0, 0, HOUSING_Z + 0.1, BODY_R, 0.3))
    frac = full.cut(tower).Volume / full.Volume
    under = vent.common(cyl(0, 0, HOUSING_Z - 2.0, BODY_R, 2.0))
    shut = sum(ovl(under, world[k]) for k in ("Body", "Pod", "Keeper", "Yoke"))
    print(
        "MH-Z19B vents: %.0f%% open through the deck, %.0f%% blocked within 2 mm under it"
        % (100 * frac, 100 * shut / under.Volume)
    )
    check(frac > 0.99, "MH-Z19B vents blocked in the deck")
    check(shut / under.Volume < 0.05, "MH-Z19B vents blocked under the deck")
    vents = trim_vents()
    area = sum(s.Volume for s in vents) / (BEZEL_T + 1.5)
    print("trim vents: %d slots, %.0f mm2" % (len(vents), area))
    check(len(vents) >= 4 and area > 80, "too few trim vents")
    for s in vents[:1]:
        check(not inside(body, s.CenterOfMass), "trim vents missing")
    for (
        q
    ) in lead_holes():  # one per lead, from the space under the deck up into the tower
        check(
            not inside(tower, q + V(0, 0, HOUSING_Z + 1.0))
            and not inside(tower, q + V(0, 0, DECK_Z + 1.0))
            and not inside(body, q + V(0, 0, HOUSING_Z - 1.0)),
            "lead hole at (%.1f, %.1f) missing or blocked" % (q.x, q.y),
        )
    # the space under the deck is closed off from the tower (and the ceiling void behind it): besides the MH-Z19B
    # vents (into its bay) and the spindle bores (over the columns), only the lead holes go through the deck, and
    # the frame closes the bay round the pin spacers
    zm = HOUSING_Z + DECK_T / 2
    hx, hy = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR
    known = [moved(box(-hx, -hy, -DECK_T - 1, hx, hy, 1), mz_plc())] + [
        moved(column(COL_R_IN, COL_R_OUT, zm - 1, zm + 1), clamp_plc(a))
        for a in CLAMP_ANGLES
    ]
    deck_open = cyl(0, 0, zm - 0.05, BODY_R - WALL - 0.05, 0.1).cut(tower)
    holes = deck_open.cut(fuse_all(known)).Volume / 0.1
    n = len(lead_holes())
    rings = holes - n * math.pi * LEAD_D**2 / 4
    zf = MZ_BAY_TOP + MZ_FRAME_T / 2
    spacers = fuse_all(
        [
            moved(s, mz_plc())
            for k, (_, s, _) in build_mz().items()
            if k.startswith("Spacer")
        ]
    )
    bay_up = (
        moved(box(-hx, -hy, zf - 0.05, hx, hy, zf + 0.05), mz_plc())
        .cut(world["MZ_Frame"])
        .cut(spacers)
        .Volume
        / 0.1
    )
    opening = moved(cradle()[2][0], cradle_plc())
    pod_out = moved(pod_barrel().fuse(trunnions()), hp)  # outline, pocket filled
    gap = opening.cut(pod_out).common(box(-60, -60, -1.55, 60, 60, -1.45)).Volume / 0.1
    print(
        "space under the deck: %.0f mm2 open to the room (trim vents, gap round the pod), into the tower only %.1f mm2 round the %d leads; the MH-Z19B bay opens to the tower through %.1f mm2 round the pin spacers"
        % (area + gap, rings, n, bay_up)
    )
    # the chamber rests on a rim round the vents: the sensor breathes only through them, and whatever gets into the
    # rest of its bay cannot reach its inlet
    rim = mz_rim()
    if rim is not None:
        mzp, mz = mz_plc(), build_mz()
        ch, pcb = mz["Chamber"][1], mz["PCB"][1]
        ix, iy = MZ_CH_L / 2 - MZ_RIM_IN - MZ_RIM_W, MZ_CH_W / 2 - MZ_RIM_IN - MZ_RIM_W
        check(
            all(
                max(-v.BoundBox.XMin, v.BoundBox.XMax) <= ix
                and max(-v.BoundBox.YMin, v.BoundBox.YMax) <= iy
                for v in mz_bay()[2]
            ),
            "MH-Z19B vents not all inside the rim",
        )
        down = App.Placement(V(0, 0, -0.05), App.Rotation())
        check(
            ovl(tower, moved(ch, mzp.multiply(down))) > 1e-4
            and ovl(tower, moved(pcb, mzp.multiply(down))) < 1e-6,
            "MH-Z19B does not rest on its rim",
        )
        z1 = MZ_GAP + MZ_BUMP_H
        top = rim.common(box(-50, -50, z1 - 0.1, 50, 50, z1 + 1))
        uncovered = max(
            top.cut(
                rbox(
                    -MZ_CH_L / 2 + dx,
                    -MZ_CH_W / 2 + dy,
                    -5,
                    MZ_CH_L / 2 + dx,
                    MZ_CH_W / 2 + dy,
                    20,
                    1.0,
                )
            ).Volume
            for dx in (-MZ_CLR, MZ_CLR)
            for dy in (-MZ_CLR, MZ_CLR)
        )
        check(
            uncovered < 1e-4, "rim not under the chamber when it shifts in its pocket"
        )
        print(
            "MH-Z19B rests on a %.1f mm rim round its vents (ledges %.2f mm low): it breathes only through the vents"
            % (MZ_RIM_W, MZ_LEDGE_DROP)
        )
    check(
        holes <= n * math.pi * ((LEAD_D + LEAD_CLR) / 2) ** 2 + 0.5,
        "space under the deck has other openings into the tower (%.1f mm2)" % holes,
    )
    check(bay_up <= 15.0, "MH-Z19B bay open to the tower (%.1f mm2)" % bay_up)
    check(LEAD_CLR <= 0.4, "lead holes too loose to close the deck")
    for a in CLAMP_ANGLES:  # spindle path: trim, housing column, deck, tower column
        p = clamp_plc(a)
        q = p.Base
        check(
            not inside(body, q + V(0, 0, -BEZEL_T + 0.3))
            and not inside(body, q + V(0, 0, HOUSING_Z - 1))
            and inside(body, p.multVec(V(-SP_BORE - 0.5, 0, HOUSING_Z - 1))),
            "no spindle bore / column in the housing at clamp %g" % a,
        )
        check(
            not inside(tower, q + V(0, 0, DECK_Z + 3))
            and inside(tower, p.multVec(V(-SP_BORE - 0.5, 0, DECK_Z + 3))),
            "no spindle column in the tower at clamp %g" % a,
        )
    for q in lid_cols():
        check(
            inside(tower, q + V(LID_PIN_D / 2 + 0.6, 0, TOP_Z - 1))
            and not inside(tower, q + V(0, 0, TOP_Z - 1)),
            "lid column or its pin hole missing",
        )
    q = polar(BODY_R - WALL / 2, CABLE_ANGLE)
    check(
        not inside(tower, q + V(0, 0, CABLE_Z0 + 1))
        and inside(tower, q + V(0, 0, CABLE_Z0 - 1))
        and not inside(lid, polar(BODY_R - 1.0, CABLE_ANGLE) + V(0, 0, TOP_Z + 0.5)),
        "power cable slot missing",
    )
    check(CABLE_Z0 >= CEIL_T_MAX + 0.5, "power cable slot below the top of the layer")

    for k, s in world.items():
        check(s.ShapeType == "Solid" and s.isValid(), "%s is not one valid solid" % k)
    print("printed parts (bounding box, mm):")
    for k in ("Body", "Tower", "Pod", "Keeper", "Yoke", "Lid", "MZ_Frame", "Arm_0"):
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
    side = App.Placement(V(0, 0, 0), App.Rotation(V(0, 1, 0), 90))
    parts = {
        "housing": build_body(),  # trim face down
        "sensor_tower": build_tower(),  # deck down
        "hmmd_pod": build_pod(),  # radome face down, smooth bed
        "hmmd_keeper": build_keeper(),
        "pod_yoke": moved(build_yoke(), side),  # on its side
        "top_lid": moved(build_lid(), flip),  # top face down, pins up
        "mhz19b_bay_frame": build_mz_frame(),  # plate down, posts up
        "clamp_arm_x3": moved(build_arm(), flip),  # arm top down, spindle up
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


doc, world, arm = build()
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
