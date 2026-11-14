"""ESP32-C3 SuperMini Plus (red) + DHT22 + MH-Z19B + Waveshare HMMD - round ceiling unit with an eyeball gimbal.

Run inside FreeCAD:
    exec(open("/Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor/esp32c3_ceiling_multisensor.py").read())
or headless:
    freecadcmd /Users/antek/projektowanie/parts/esp32c3-ceiling-multisensor/esp32c3_ceiling_multisensor.py

Frame (mm): origin on the hole axis at the UNDERSIDE of the ceiling, +Z up into the ceiling. The trim (bezel) sits on
the room side (z = -BEZEL_T..0) and covers the edge of the hole; the housing fills the hole and stands up into the open
space above the ceiling. Edit a parameter and re-run; the model is only saved / exported if every check passes.

 * Mounting: three swing clamps, like in-ceiling speakers. Push the unit into the hole and turn the three countersunk
   M4 screws in the trim clockwise: the nyloc nut in each arm drags the arm round until it hits its stop (arm now
   pointing outward over the ceiling), then the arm is pulled down onto the top of the ceiling. Any ceiling between
   CEIL_T_MIN and CEIL_T_MAX works. Undo: turn anticlockwise, the arms swing back in and the unit drops out.
 * Gimbal: the HMMD sits in a ball (eyeball) whose flat face is the radome (same radome rules as the other case). The
   ball turns in a spherical socket formed by the trim and a retainer ring, up to TILT_MAX in any direction; a neck on
   the back of the ball is the end stop. Three countersunk M3 screws in the trim pull the retainer down = friction.
 * DHT22 module: its grille sits in a window of the trim, in a closed bay of its own, away from the warm electronics.
 * MH-Z19B: hangs chamber-down in a vented bay of its own, pins up; the bay frame keeps it in place.
 * ESP32-C3: stands on edge under the top lid with its USB-C in a slot of the lid, so a cable plugs in from above
   (the open space over the ceiling). The lid also caps the clamp channels and holds the top ends of the M4 screws.
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
# display only: ball tilt (direction deg, tilt deg); the checks use their own angles
SHOW_TILT = (0.0, 0.0)

# ------------------------------------------------------------------ ceiling (user spec)
HOLE_R = 70.0  # radius of the hole in the ceiling
CEIL_T = 35.0  # ceiling thickness (open space above it); used for the reference model
CEIL_T_MIN, CEIL_T_MAX = 30.0, 40.0  # the clamps grip any thickness in this range

# ------------------------------------------------------------------ housing
BODY_R = HOLE_R - 1.0  # outer radius of everything that goes into the hole
WALL = 2.0
BEZEL_R, BEZEL_T = HOLE_R + 12.0, 3.0  # trim ring on the room side

# ------------------------------------------------------------------ swing clamps (M4 x 55 countersunk + M4 nyloc)
CLAMP_ANGLES = (90.0, 210.0, 330.0)
CLAMP_R = 59.0  # screw axis radius
# ISO 10642 countersunk screw; the length includes the head
M4_D, M4_BORE, M4_HEAD_D, M4_LEN = 4.0, 4.5, 8.0, 55.0
NUT4_AF, NUT4_H = 7.0, 5.0  # DIN 985
# arm: thickness, bar width, axis to tip, nut boss radius
ARM_T, ARM_W, ARM_L, ARM_BOSS_R = 7.0, 10.0, 24.0, 6.5
ARM_CLR = 0.8  # clamp channel clearance around the swinging arm
ARM_STOP_GAP = 0.3  # arm side to its stop face when pointing straight out
TUBE_WALL = 1.6  # screw guide tube
PARK_GAP = 2.0  # parked (retracted) arm bottom above CEIL_T_MAX

# ------------------------------------------------------------------ gimbal (ball + socket)
BALL_R = 25.0
BALL_FACE_ANG = 40.0  # half-angle of the flat radome face (deg); 90 - this = overhang
NECK_R, NECK_TOP = 16.0, 27.0  # neck on the back of the ball: tilt stop + wire exit
SEAT_CLR = 0.15  # ball to socket, radial
SEAT_RIM_ANG = 68.0  # lowest socket contact, polar angle from straight down (deg)
TILT_MAX = 25.0  # deg, any direction
RET_GAP = 1.2  # retainer to hub at nominal; the M3 screws close it to add friction
HUB_R = 35.5  # outer radius of the socket hub and the retainer ring
# M3 x 20 countersunk friction screws into M3 nuts in the retainer
FRICTION_R, FRICTION_ANGLES = 30.5, (30.0, 150.0, 270.0)
M3_D, M3_BORE, M3_HEAD_D, M3_PILOT = 3.0, 3.4, 6.0, 2.6
NUT3_AF, NUT3_H = 5.5, 2.4
KEEPER_WALL, KEEPER_CLR, CAP_T = 1.0, 0.1, 2.0  # HMMD keeper: square tube + neck cap

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
DUPONT_L = 14.0  # female jumper housing length (reference, for wiring room)
# the board stands on edge at ESP_ANGLE, its back face ESP_BACK_R from the axis
ESP_ANGLE, ESP_BACK_R = 260.0, 57.0
ESP_TOP_GAP = 0.3  # PCB end (USB side) to the lid underside

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
# bay direction and distance of its centre line from the axis
DHT_ANGLE, DHT_DIST = 20.0, 47.5
DHT_BODY_W, DHT_BODY_L, DHT_BODY_T = 15.1, 25.1, 7.7
DHT_BODY_T_MIN, DHT_BODY_T_MAX = 7.4, 8.0
DHT_PCB_W, DHT_PCB_L, DHT_PCB_T = 13.0, 28.2, 1.6
DHT_OVERHANG = 9.0  # sensor body (hole tab) beyond the far end of the PCB
DHT_PIN_OUT = 6.5  # header pins beyond the near end of the PCB
DHT_LIP, DHT_CLR = 0.6, 0.25  # window lip under the grille; clearance around the body
DHT_BAY_TOP = 8.4  # bay lid underside

# ------------------------------------------------------------------ MH-Z19B (pins type), Winsen manual v1.7 drawing
MZ_ANGLE, MZ_DIST = 140.0, 50.0
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
# slots in the trim + lid: a chimney through the electronics (False over a cold attic)
INTERIOR_VENTS = True
WIRE_HOLE_D = 7.0  # extra hole in the lid for a power cable instead of USB-C (0 = none)
BAY_WALL = 1.6
BOSS_R = 3.25  # screw bosses (M3 into a 2.6 mm pilot)

# ------------------------------------------------------------------ derived
D2R = math.pi / 180.0
ARM_PARK_Z = CEIL_T_MAX + PARK_GAP  # parked arm bottom
# top of the housing = lid underside; the lid is also the top stop of the arms
TOP_Z = ARM_PARK_Z + ARM_T + 1.0
LID_T = 2.4
TUBE_TOP = CEIL_T_MIN - 1.0  # screw guide tubes end below the lowest arm position
# clamp channel, in clamp coordinates (x radial out, y anticlockwise seen from above)
NOTCH_X0 = -(ARM_W / 2 + ARM_CLR)
NOTCH_Y1 = ARM_W / 2 + ARM_STOP_GAP  # straight-out stop face
NOTCH_RAD = ARM_L + ARM_CLR
NOTCH_BOSS = ARM_BOSS_R + ARM_CLR
# lid screw column (x, y, r) beside each channel, clamp coordinates
LID_COL = (4.0, NOTCH_Y1 + WALL + 3.5, 3.5)
NUT4_POCKET = NUT4_H + 0.2

BALL_FACE_D = BALL_R * math.cos(BALL_FACE_ANG * D2R)  # ball centre to the radome face
BALL_FACE_R = BALL_R * math.sin(BALL_FACE_ANG * D2R)
SEAT_R = BALL_R + SEAT_CLR
# ball centre (nominal, concentric with the socket)
Z_BALL = -BEZEL_T + SEAT_R * math.cos(SEAT_RIM_ANG * D2R)
# neck root, polar angle from the ball axis
NECK_ANG = math.degrees(math.asin(NECK_R / BALL_R))
RET_TOP_ANG = NECK_ANG + TILT_MAX  # retainer rim, polar angle from straight up
Z_RET0 = Z_BALL + RET_GAP
Z_RET1 = Z_BALL + SEAT_R * math.cos(RET_TOP_ANG * D2R)
Z_POCKET = -BALL_FACE_D + RADOME_T  # ball frame: radome inner face
Z_HM_FRONT = Z_POCKET + HM_H  # ball frame: HMMD antenna face
HM_X0, HM_Y0 = -HM_W / 2, -HM_L / 2  # ball frame: HMMD plan origin
PKT = HM_W / 2 + HM_CLR  # half size of the square HMMD pocket

# DHT22 bay frame (x along the bay, y toward the axis)
DHT_X_TAB = 28.1 - 0.5  # far end of the sensor body
DHT_X_PCB0 = DHT_X_TAB - DHT_OVERHANG - DHT_PCB_L  # header end of the PCB
# inner x0, x1, half width: room for a jumper housing on the pins + the wire bend
DHT_BAY = (DHT_X_PCB0 - DUPONT_L - 4.5, DHT_X_TAB + 0.5, DHT_BODY_W / 2 + 0.55)
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


def bay_plc(angle, dist):
    """Bay coordinates -> world: x anticlockwise along the chord, y toward the axis, origin on the bay centre line."""
    return App.Placement(polar(dist, angle), App.Rotation(V(0, 0, 1), angle + 90.0))


def esp_plc():
    """ESP32 coordinates (x across, y toward USB, z = back face to front face) -> world: upright, front facing in."""
    ur, ut = polar(1, ESP_ANGLE), polar(1, ESP_ANGLE + 90)
    ex, ey, ez = -ut, V(0, 0, 1), -ur
    t = ur * ESP_BACK_R + ut * (PCB_W / 2) + V(0, 0, ESP_Z0)
    m = App.Matrix(
        ex.x, ey.x, ez.x, t.x, ex.y, ey.y, ez.y, t.y, ex.z, ey.z, ez.z, t.z, 0, 0, 0, 1
    )
    return App.Placement(m)


def ball_plc(direction=0.0, tilt=0.0):
    """Ball coordinates (centre at origin, radome face toward -z) -> world, tilted toward `direction`."""
    # this axis tilts the back of the ball toward `direction`
    axis = V(-math.sin(direction * D2R), math.cos(direction * D2R), 0)
    return App.Placement(V(0, 0, Z_BALL), App.Rotation(axis, tilt))


# ------------------------------------------------------------------ ESP32 board model (reference only, its own frame)
PIN_YS = [PIN_Y_FIRST - i * PIN_PITCH for i in range(PIN_COUNT)]
HDR_Y0, HDR_Y1 = (
    PIN_Y_FIRST - (PIN_COUNT - 0.5) * PIN_PITCH,
    PIN_Y_FIRST + PIN_PITCH / 2,
)


def build_esp32(pcb_t=PCB_T):
    """Back face at z = 0, component side up; headers soldered housing-up with jumper housings on the long pins."""
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
        parts["Hdr_%s" % tag] = (
            "ESP32 header %s 1x8" % tag,
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
        parts["Pins_%s" % tag] = (
            "ESP32 header %s pins" % tag,
            Part.makeCompound(pins),
            (0.85, 0.70, 0.25),
        )
        jump = [
            box(
                x - 1.27,
                y - 1.27,
                z1 + HDR_H,
                x + 1.27,
                y + 1.27,
                z1 + HDR_H + DUPONT_L,
            )
            for y in PIN_YS
        ]
        parts["Jumpers_%s" % tag] = (
            "jumper housings %s (wiring room)" % tag,
            Part.makeCompound(jump),
            (0.2, 0.2, 0.8),
        )
    return parts


# ------------------------------------------------------------------ HMMD model (reference only, ball frame)
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
    hx = P(HM_PIN_X)
    parts["Hdr"] = (
        "HMMD J2 header 1x5",
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
    parts["Pins"] = (
        "HMMD J2 pins (OT2 RX TX GND 3V3)",
        Part.makeCompound(pins),
        (0.85, 0.70, 0.25),
    )
    jump = [
        box(
            hx - 1.27,
            Q(v) - 1.27,
            zb + HDR_H,
            hx + 1.27,
            Q(v) + 1.27,
            zb + HDR_H + DUPONT_L,
        )
        for v in HM_PIN_V
    ]
    parts["Jumpers"] = (
        "HMMD jumper housings (wiring room)",
        Part.makeCompound(jump),
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
    parts["Jumper"] = (
        "DHT22 jumper housing 1x3 (wiring room)",
        box(xp - DUPONT_L, -3.81, zp - 1.27, xp, 3.81, zp + 1.27),
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
        jump = [
            box(
                x - 1.27,
                y - 1.27,
                zt + MZ_SPACER_H,
                x + 1.27,
                y + 1.27,
                zt + MZ_SPACER_H + DUPONT_L,
            )
            for y in ys
        ]
        parts["Jumpers_" + tag] = (
            "MH-Z19B jumper housings %s" % tag,
            Part.makeCompound(jump),
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


def dht_bay():
    """DHT22 bay (bay coordinates): walls + screw bosses, the cuts, boss x positions."""
    x0, x1, hw = DHT_BAY
    t = BAY_WALL
    bx = (x0 - t - BOSS_R + 1.0, x1 + t + BOSS_R - 1.0)
    walls = fuse_all(
        [box(x0 - t, -hw - t, 0, x1 + t, hw + t, DHT_BAY_TOP)]
        + [cyl(b, 0, 0, BOSS_R, DHT_BAY_TOP) for b in bx]
    )
    xb0, c, w = DHT_X_TAB - DHT_BODY_L, DHT_CLR, DHT_BODY_W / 2
    cuts = [
        box(x0, -hw, 0, x1, hw, DHT_BAY_TOP + 1),
        box(
            xb0 - c, -w - c, DHT_Z_FACE, DHT_X_TAB + c, w + c, 0.5
        ),  # body pocket in the trim
        box(
            xb0 + 0.75,
            -w + 0.75,
            -BEZEL_T - 1,
            DHT_X_TAB - 0.75,
            w - 0.75,
            DHT_Z_FACE + 0.1,
        ),  # grille window
    ] + [cyl(b, 0, DHT_BAY_TOP - 7, M3_PILOT / 2, 8) for b in bx]
    return walls, cuts, bx


def mz_bay():
    """MH-Z19B bay (bay coordinates): chamber pocket with vents, ledges for the PCB ends, screw bosses."""
    hx, hy, t = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR, BAY_WALL
    bx = (-(hx + t + BOSS_R - 1.0), hx + t + BOSS_R - 1.0)
    walls = fuse_all(
        [box(-hx - t, -hy - t, 0, hx + t, hy + t, MZ_BAY_TOP)]
        + [cyl(b, 0, 0, BOSS_R, MZ_BAY_TOP) for b in bx]
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
    for i in range(7):  # vents under the chamber (the gas inlet faces down)
        x = -10.5 + 3.5 * i
        cuts.append(box(x - 0.8, -7.5, -BEZEL_T - 1, x + 0.8, 7.5, 0.1))
    cuts += [cyl(b, 0, MZ_BAY_TOP - 8, M3_PILOT / 2, 9) for b in bx]
    return walls, cuts, bx


def esp_socket():
    """ESP32 coordinates: column from the trim up to the antenna end of the board, with jaws that miss every part."""
    back = -(BODY_R - WALL + 0.5 - ESP_BACK_R)  # reaches into the outer wall
    t1 = PCB_T_MAX + 0.1
    return fuse_all(
        [
            box(-1.5, -ESP_Z0, back, PCB_W + 1.5, 0, 3.4),  # column + floor
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
    disc = cyl(0, 0, 0, BODY_R, TOP_Z)
    adds = [
        bezel,
        disc.cut(cyl(0, 0, -1, BODY_R - WALL, TOP_Z + 2)),
        cyl(0, 0, 0, HUB_R, Z_BALL),
    ]
    cuts = [Part.makeSphere(SEAT_R, V(0, 0, Z_BALL))]
    for a in CLAMP_ANGLES:
        p = clamp_plc(a)
        adds.append(moved(notch_local(WALL, 0, TOP_Z), p).common(disc))
        adds.append(moved(cyl(LID_COL[0], LID_COL[1], 0, LID_COL[2], TOP_Z), p))
        cuts.append(moved(notch_local(0, 0, TOP_Z + 1), p))
        cuts.append(moved(cyl(LID_COL[0], LID_COL[1], TOP_Z - 12, M3_PILOT / 2, 13), p))
    for bay, plc in (
        (dht_bay, bay_plc(DHT_ANGLE, DHT_DIST)),
        (mz_bay, bay_plc(MZ_ANGLE, MZ_DIST)),
    ):
        walls, bcuts, _ = bay()
        adds.append(moved(walls, plc))
        cuts += [moved(c, plc) for c in bcuts]
    adds.append(moved(esp_socket(), esp_plc()))
    for a in (
        FRICTION_ANGLES
    ):  # M3 friction screws: bore through the hub + countersink in the trim
        p = polar(FRICTION_R, a)
        cuts.append(cyl(p.x, p.y, -BEZEL_T - 1, M3_BORE / 2, Z_BALL + BEZEL_T + 2))
        cuts.append(
            cone(
                p.x,
                p.y,
                -BEZEL_T - 0.01,
                M3_HEAD_D / 2 + 0.2,
                M3_BORE / 2,
                M3_HEAD_D / 2 + 0.2 - M3_BORE / 2,
            )
        )
    if INTERIOR_VENTS:  # inlet under the ESP32 sector
        for da in (-20, -10, 0, 10, 20):
            a = ESP_ANGLE + da
            p0, p1 = polar(38.0, a), polar(50.0, a)
            cuts.append(slot(p0.x, p0.y, p1.x, p1.y, -BEZEL_T - 1, BEZEL_T + 1.1, 2.0))
    body = fuse_all(adds).cut(fuse_all(cuts))
    tubes, bores = [], []
    for a in (
        CLAMP_ANGLES
    ):  # M4 guide tubes back into the channels, then the bores + countersinks
        p = clamp_plc(a)
        tubes.append(
            moved(
                cyl(0, 0, 0, M4_BORE / 2 + TUBE_WALL, TUBE_TOP).fuse(
                    box(NOTCH_X0 - 0.5, -1.25, 0, 0, 1.25, TUBE_TOP)
                ),
                p,
            )
        )
        bores.append(
            moved(cyl(0, 0, -BEZEL_T - 1, M4_BORE / 2, TUBE_TOP + BEZEL_T + 2), p)
        )
        rc = M4_HEAD_D / 2 + 0.2
        bores.append(
            moved(cone(0, 0, -BEZEL_T - 0.01, rc, M4_BORE / 2, rc - M4_BORE / 2), p)
        )
    body = body.fuse(tubes).cut(fuse_all(bores))
    return solid(body.removeSplitter())


def rail_box(u0, u1, v0, v1):
    """HMMD ledge (ball frame): from the pocket wall to the contact strip, radome up to the antenna face."""
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


def build_ball():
    """Ball frame: centre at the origin, flat radome face toward -z, neck toward +z."""
    r = BALL_R + 1
    ball = Part.makeSphere(BALL_R, V(0, 0, 0)).cut(box(-r, -r, -r, r, r, -BALL_FACE_D))
    ball = ball.fuse(cyl(0, 0, 0, NECK_R, NECK_TOP))
    pocket = box(-PKT, -PKT, Z_POCKET, PKT, PKT, NECK_TOP + 1).cut(
        fuse_all([rail_box(*q) for q in HM_RAILS.values()])
    )
    return solid(ball.cut(pocket).removeSplitter())


def build_keeper(ribs=True):
    """Ball frame: square tube on the HMMD back (clear of every part, open over J2) + cap on the neck, wire exit."""
    zb = Z_HM_FRONT + HM_T_MAX + 0.1
    o = PKT - KEEPER_CLR
    i = o - KEEPER_WALL
    tube = box(-o, -o, zb, o, o, NECK_TOP).cut(box(-i, -i, zb - 1, i, i, NECK_TOP + 1))
    # the J2 header + jumper housings stay outside the keeper
    xj = HM_X0 + HM_PIN_X + HDR_W / 2 + 0.3
    tube = tube.cut(box(-o - 1, HM_Y0 + 6.0, zb - 1, xj, o + 1, NECK_TOP + 1))
    parts = [tube, cyl(0, 0, NECK_TOP, NECK_R - 0.4, CAP_T)]
    if ribs:  # crush ribs: 0.15 mm interference, so the keeper stays in when the unit is upside down
        h = KEEPER_CLR + 0.15
        parts += [
            box(o - 0.01, -0.5, NECK_TOP - 8, o + h, 0.5, NECK_TOP),
            box(1.5, o - 0.01, NECK_TOP - 8, 2.5, o + h, NECK_TOP),
            box(1.5, -o - h, NECK_TOP - 8, 2.5, -o + 0.01, NECK_TOP),
        ]
    keeper = fuse_all(parts).cut(
        box(-o - 1, HM_Y0 + 6.0, NECK_TOP - 1, HM_X0 + 5.0, o, NECK_TOP + CAP_T + 1)
    )
    return solid(keeper.removeSplitter())


def build_retainer():
    ring = cyl(0, 0, Z_RET0, HUB_R, Z_RET1 - Z_RET0).cut(
        Part.makeSphere(SEAT_R, V(0, 0, Z_BALL))
    )
    for a in FRICTION_ANGLES:
        p = polar(FRICTION_R, a)
        ring = ring.cut(cyl(p.x, p.y, Z_RET0 - 1, M3_BORE / 2, Z_RET1 - Z_RET0 + 2))
        ring = ring.cut(
            hexagon(
                p.x, p.y, Z_RET1 - NUT3_H - 0.2, NUT3_AF + 0.3, NUT3_H + 1, rot=a + 30
            )
        )
    return solid(ring.removeSplitter())


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
    for a in CLAMP_ANGLES:
        p = clamp_plc(a)
        # top bearing of the M4 screw, then the lid screw
        cuts.append(moved(cyl(0, 0, z0 - 1, M4_D / 2 + 0.3, LID_T + 2), p))
        cuts.append(
            moved(cyl(LID_COL[0], LID_COL[1], z0 - 1, M3_BORE / 2, LID_T + 2), p)
        )
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
        for da in (-30, -20, 20, 30):
            p0, p1 = polar(30.0, ESP_ANGLE + da), polar(44.0, ESP_ANGLE + da)
            cuts.append(slot(p0.x, p0.y, p1.x, p1.y, z0 - 1, LID_T + 2, 2.0))
    if WIRE_HOLE_D > 0:
        p = polar(40.0, ESP_ANGLE)
        cuts.append(cyl(p.x, p.y, z0 - 1, WIRE_HOLE_D / 2, LID_T + 2))
    return solid(lid.cut(fuse_all(cuts)).removeSplitter())


def build_dht_lid():
    x0, x1, hw = DHT_BAY
    t, z = BAY_WALL, DHT_BAY_TOP
    _, _, bx = dht_bay()
    lid = fuse_all(
        [box(x0 - t, -hw - t, z, x1 + t, hw + t, z + 2.0)]
        + [cyl(b, 0, z, BOSS_R, 2.0) for b in bx]
    )
    cuts = [cyl(b, 0, z - 1, M3_BORE / 2, 4) for b in bx] + [
        box(x0, -hw + 1.5, z - 1, x0 + 5.5, hw - 1.5, z + 3)
    ]  # wires
    return solid(lid.cut(fuse_all(cuts)).removeSplitter())


def build_mz_frame():
    hx, hy, t, z = MZ_PCB_L / 2 + 0.35, MZ_CH_W / 2 + MZ_CLR, BAY_WALL, MZ_BAY_TOP
    _, _, bx = mz_bay()
    frame = fuse_all(
        [box(-hx - t, -hy - t, z, hx + t, hy + t, z + 2.0)]
        + [cyl(b, 0, z, BOSS_R, 2.0) for b in bx]
    )
    cuts = [cyl(b, 0, z - 1, M3_BORE / 2, 4) for b in bx]
    cuts += [
        box(s * MZ_PIN_X - 2.1, -hy + 0.6, z - 1, s * MZ_PIN_X + 2.1, hy - 0.6, z + 3)
        for s in (-1, 1)
    ]
    return solid(frame.cut(fuse_all(cuts)).removeSplitter())


def build_ceiling(t=CEIL_T):
    return box(-110, -110, 0, 110, 110, t).cut(cyl(0, 0, -1, HOLE_R, t + 2))


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
    for i, a in enumerate(FRICTION_ANGLES):
        q = polar(FRICTION_R, a)
        scr = cone(q.x, q.y, -BEZEL_T, M3_HEAD_D / 2, M3_D / 2, k3).fuse(
            cyl(q.x, q.y, -BEZEL_T + k3 - 0.01, M3_D / 2, 20 - k3)
        )
        parts["M3_%d" % i] = (
            "M3 x 20 countersunk (gimbal friction %d)" % (i + 1),
            scr,
            (0.75, 0.75, 0.78),
        )
        nut = hexagon(q.x, q.y, Z_RET1 - NUT3_H - 0.2, NUT3_AF, NUT3_H, rot=a + 30).cut(
            cyl(q.x, q.y, Z_RET0, M3_D / 2, 20)
        )
        parts["M3nut_%d" % i] = (
            "M3 nut (gimbal friction %d)" % (i + 1),
            nut,
            (0.75, 0.75, 0.78),
        )
    return parts


# ------------------------------------------------------------------ document
SPECS = [
    (
        "Ceiling",
        "hole radius %g mm, ceiling %g mm thick, open space above; clamps grip %g-%g mm"
        % (HOLE_R, CEIL_T, CEIL_T_MIN, CEIL_T_MAX),
        "user spec",
    ),
    (
        "Clamps",
        "3 swing arms on M4 x 55 countersunk screws (ISO 10642) with M4 nyloc nuts (DIN 985) - the nyloc friction swings the arm out",
        "design choice",
    ),
    (
        "Gimbal",
        "ball %g mm, flat radome face %.1f mm across; tilt %g deg in any direction (neck hits the retainer)"
        % (2 * BALL_R, 2 * BALL_FACE_R, TILT_MAX),
        "design choice",
    ),
    (
        "Gimbal friction",
        "3 x M3 x 20 countersunk from the room side into M3 nuts in the retainer ring; tighten until the ball holds its aim",
        "design choice",
    ),
    (
        "Other screws",
        "2 x M3 x 8 per bay lid (DHT22, MH-Z19B), 3 x M3 x 10 for the top lid - all into 2.6 mm pilot holes",
        "design choice",
    ),
    (
        "Board",
        "ESP32-C3 SuperMini Plus (V2), red PCB, 22.52 x 18 mm; on edge, USB-C in the top lid; PCB 1.0-1.6 mm checked",
        "datasheet + illustration",
    ),
    (
        "HMMD",
        "Waveshare HMMD (S3KM1110, 24 GHz) 20 x 20 mm, J2 header on the back; radome 0.8 mm, antenna gap 6.2 mm (half wavelength)",
        "Waveshare wiki + radome guide",
    ),
    (
        "HMMD thickness",
        "NOT published. Modelled 1.0; ball checked for 0.8-1.6 mm",
        "assumption - measure yours",
    ),
    (
        "DHT22",
        "AM2302 body 15.1 x 25.1 x 7.7 mm on a 13 x 28.2 mm black PCB, right-angle 3-pin header (- out +), 43 mm overall",
        "AM2302 datasheet, retailer drawing",
    ),
    (
        "DHT22 fit",
        "grille flush in a 13.6 x 23.6 mm window of the trim (0.6 mm lip); body 7.4-8.0 mm checked",
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
        "housing trim-down, ball face-down (smooth bed for the radome), retainer and keeper upside down, lid/arms/bay lids flat",
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
        ("HMMD", build_hmmd, ball_plc()),
        ("DHT22", build_dht, bay_plc(DHT_ANGLE, DHT_DIST)),
        ("MHZ19B", build_mz, bay_plc(MZ_ANGLE, MZ_DIST)),
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
            "Housing: trim + socket hub + clamp channels + bays",
            build_body(),
            (0.93, 0.93, 0.92),
            0,
        ),
        "Ball": (
            "Gimbal ball (HMMD inside, radome face)",
            build_ball(),
            (0.97, 0.97, 0.97),
            0,
        ),
        "Keeper": ("HMMD keeper + neck cap", build_keeper(), (0.30, 0.60, 0.90), 0),
        "Retainer": ("Gimbal retainer ring", build_retainer(), (0.30, 0.60, 0.90), 0),
        "Lid": (
            "Top lid (USB-C slot, clamp screw bearings)",
            build_lid(),
            (0.30, 0.60, 0.90),
            55,
        ),
        "DHT_Lid": (
            "DHT22 bay lid",
            moved(build_dht_lid(), bay_plc(DHT_ANGLE, DHT_DIST)),
            (0.30, 0.60, 0.90),
            0,
        ),
        "MZ_Frame": (
            "MH-Z19B bay frame",
            moved(build_mz_frame(), bay_plc(MZ_ANGLE, MZ_DIST)),
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
    for k in ("Ball", "Keeper"):  # ball frame -> world
        lab, s, rgb, tr = case[k]
        world[k] = (lab, moved(s, ball_plc()), rgb, tr)
    shown = dict(world)
    for k in ("Ball", "Keeper"):
        lab, s, rgb, tr = case[k]
        shown[k] = (lab, moved(s, ball_plc(*SHOW_TILT)), rgb, tr)
    for k, (lab, s, rgb, tr) in shown.items():
        new_part(doc, "Case_" + k, lab, s, rgb, transparency=tr)
    refs = module_parts()
    if SHOW_TILT[1]:
        refs["HMMD"] = {
            k: (lab, moved(s, ball_plc(*SHOW_TILT)), rgb)
            for k, (lab, s, rgb) in build_hmmd().items()
        }
    for grp, parts in refs.items():
        g = doc.addObject("App::DocumentObjectGroup", grp + "_Reference")
        g.Label = grp + " - reference model"
        for k, (lab, s, rgb) in parts.items():
            new_part(doc, grp + "_" + k, lab, s, rgb, group=g)
    g = doc.addObject("App::DocumentObjectGroup", "Ceiling_Reference")
    g.Label = "Ceiling - reference (%g mm, hole R%g)" % (CEIL_T, HOLE_R)
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
    """Numeric checks: overlaps, thickness sweeps, clamp swing / reach / stops, gimbal tilt / stop, radome, openings."""
    fails = []

    def check(ok, msg):
        if not ok:
            fails.append(msg)

    def inside(shape, p):
        return shape.isInside(p, 1e-6, False)

    body, lid, ret = world["Body"], world["Lid"], world["Retainer"]
    case = dict(world)
    # the keeper's crush ribs are measured on their own
    case["Keeper"] = moved(build_keeper(ribs=False), ball_plc())
    names = list(case)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            v = ovl(case[a], case[b])
            check(v < 1e-4, "%s and %s overlap (%.4f mm3)" % (a, b, v))
    rib = ovl(world["Keeper"], world["Ball"])
    print("keeper crush ribs: %.2f mm3 interference with the ball (by design)" % rib)
    check(0.5 < rib < 10, "keeper crush ribs missing or too big")

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
        v = sum(ovl(body, s) + ovl(lid, s) for s in parts.values())
        usb = parts["USB_C"]
        tip = usb.BoundBox.ZMax
        print(
            "  T=%.1f overlap=%.4f | USB-C to lid %.2f, to housing %.2f | receptacle tip %.2f below the lid top"
            % (
                t,
                v,
                usb.distToShape(lid)[0],
                usb.distToShape(body)[0],
                TOP_Z + LID_T - tip,
            )
        )
        check(v < 1e-4, "ESP32 interference at PCB thickness %.1f" % t)
    ball0, keep0 = build_ball(), build_keeper(ribs=False)
    print("HMMD PCB thickness sweep (in the ball):")
    for t in (HM_T_MIN, HM_T, 1.2, HM_T_MAX):
        v = sum(ovl(ball0, s) + ovl(keep0, s) for _, s, _ in build_hmmd(t).values())
        print(
            "  T=%.1f overlap=%.4f | board can lift %.2f mm before the keeper"
            % (t, v, HM_T_MAX + 0.1 - t)
        )
        check(v < 1e-4, "HMMD interference at PCB thickness %.1f" % t)
    dp = bay_plc(DHT_ANGLE, DHT_DIST)
    print("DHT22 body thickness sweep (grille in the trim window):")
    for bt in (DHT_BODY_T_MIN, DHT_BODY_T, DHT_BODY_T_MAX):
        parts = [moved(s, dp) for _, s, _ in build_dht(bt).values()]
        v = sum(ovl(body, s) + ovl(world["DHT_Lid"], s) for s in parts)
        top = max(s.BoundBox.ZMax for s in parts)
        print(
            "  T=%.1f overlap=%.4f | module top %.2f, bay lid at %.2f"
            % (bt, v, top, DHT_BAY_TOP)
        )
        check(v < 1e-4, "DHT22 interference at body thickness %.1f" % bt)

    # ---- swing clamps
    for i, a in enumerate(CLAMP_ANGLES):
        near = body.common(
            moved(box(-16, -36, -BEZEL_T - 1, 36, 22, TOP_Z + 1), clamp_plc(a))
        )
        for z in (CEIL_T_MIN, CEIL_T, CEIL_T_MAX, ARM_PARK_Z):
            for sw in (-90, -60, -30, 0):
                s = arm_at(arm, a, sw, z)
                v = ovl(near, s) + ovl(lid, s)
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
        "M4 screw end z=%.2f in the lid bearing (%.1f..%.1f); parked nyloc top %.2f; travel park->%g mm ceiling %.1f mm"
        % (end, TOP_Z, TOP_Z + LID_T, nut_top, CEIL_T, ARM_PARK_Z - CEIL_T)
    )
    check(rt <= HOLE_R - 1.0, "tucked-in arms do not pass through the hole")
    check(ro - HOLE_R >= 10.0, "arms grip less than 10 mm of the ceiling")
    check(end >= nut_top + 1.0, "M4 screw too short for a parked arm")
    check(TOP_Z < end <= TOP_Z + LID_T, "M4 screw end not in the lid bearing")
    check(
        max_radius(body.common(cyl(0, 0, 0.1, 100, TOP_Z))) <= HOLE_R - 0.9,
        "housing too big for the hole",
    )

    # ---- gimbal
    near = body.common(cyl(0, 0, -BEZEL_T - 1, HUB_R + 6, TOP_Z))
    asm = Part.makeCompound([ball0, keep0] + [s for _, s, _ in build_hmmd().values()])
    stops = []
    for d in range(0, 360, 45):
        for t in (12.0, TILT_MAX - 1.0):
            s = moved(asm, ball_plc(d, t))
            v = ovl(near, s) + ovl(ret, s)
            check(
                v < 1e-4, "ball hits the socket at %g deg toward %d (%.3f)" % (t, d, v)
            )
        stops.append(ovl(ret, moved(ball0, ball_plc(d, TILT_MAX + 2.0))))
    check(min(stops) > 1e-3, "no tilt stop in some direction")
    t = TILT_MAX * D2R
    z_edge = Z_BALL - BALL_FACE_D * math.cos(t) + BALL_FACE_R * math.sin(t)
    r_rim = SEAT_R * math.sin(SEAT_RIM_ANG * D2R)
    # ball centre when pressed onto the rim; retainer travel until its rim touches it
    h = -BEZEL_T + math.sqrt(BALL_R**2 - r_rim**2)
    r_top = SEAT_R * math.sin(RET_TOP_ANG * D2R)
    reach = Z_RET1 - h - math.sqrt(BALL_R**2 - r_top**2)
    print(
        "gimbal: tilt %g deg ok, stop at <= %g deg in 8 directions | radome edge at full tilt %.2f below the trim face"
        % (TILT_MAX - 1, TILT_MAX + 2, -BEZEL_T - z_edge)
    )
    print(
        "socket lip %.2f mm under the ball; pressed ball sits %.2f low; retainer touches it after %.2f of its %.2f gap"
        % (BALL_R - r_rim, Z_BALL - h, reach, RET_GAP)
    )
    print(
        "ball face %.1f mm below the trim face (untilted)"
        % (-BEZEL_T - (Z_BALL - BALL_FACE_D))
    )
    check(z_edge < -BEZEL_T - 0.3, "radome edge enters the socket at full tilt")
    check(BALL_R - r_rim >= 1.0, "socket lip too small to hold the ball")
    check(0.2 < reach < RET_GAP - 0.2, "retainer cannot preload the ball")

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
            ok = inside(ball0, V(x, y, -BALL_FACE_D + 0.05)) and inside(
                ball0, V(x, y, Z_POCKET - 0.05)
            )
            bad += 0 if ok and not inside(ball0, V(x, y, Z_POCKET + 0.05)) else 1
    print(
        "radome: %d sample points, %d off (floor %.2f mm, air gap %.2f mm)"
        % (n, bad, RADOME_T, HM_H)
    )
    check(bad == 0, "radome floor is not a uniform %.2f mm" % RADOME_T)
    for k, (u0, u1, v0, v1) in HM_RAILS.items():
        c = V(HM_X0 + (u0 + u1) / 2, HM_Y0 + (v0 + v1) / 2, Z_HM_FRONT)
        check(
            inside(ball0, c - V(0, 0, 0.05)) and not inside(ball0, c + V(0, 0, 0.05)),
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
            ovl(ball0, kb) < 1e-6,
            "ball material within %.1f mm of antenna patch %s" % (m, k),
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
        not inside(body, bay_plc(MZ_ANGLE, MZ_DIST).multVec(V(0, 0, -BEZEL_T + 0.5))),
        "MH-Z19B vents missing",
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
            not inside(lid, V(p.x, p.y, TOP_Z + 1)),
            "no lid bearing over clamp screw at %g" % a,
        )
        check(
            not inside(body, V(p.x, p.y, -BEZEL_T + 0.5)),
            "no countersink at clamp %g" % a,
        )
    for a in FRICTION_ANGLES:
        p = polar(FRICTION_R, a)
        check(
            not inside(body, V(p.x, p.y, 3.0))
            and not inside(ret, V(p.x, p.y, Z_RET0 + 1)),
            "friction screw path blocked",
        )

    for k, s in world.items():
        check(s.ShapeType == "Solid" and s.isValid(), "%s is not one valid solid" % k)
    print("printed parts (bounding box, mm):")
    for k in (
        "Body",
        "Ball",
        "Keeper",
        "Retainer",
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
        "gimbal_ball": build_ball(),  # radome face down, smooth bed
        "hmmd_keeper": moved(build_keeper(), flip),  # cap down
        "gimbal_retainer": moved(build_retainer(), flip),  # top face down
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
