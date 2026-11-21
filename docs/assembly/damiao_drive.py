"""DM4340P 24 V layout in millimetres. Nominal geometry for unpowered fit checks."""
import math
from OCP.gp import gp_Pnt, gp_Vec
from OCP.GC import GC_MakeArcOfCircle
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire,
                                BRepBuilderAPI_MakeFace)
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism

CENTRE = 147.5
REAR = 65.5
FLANGE = REAR + 56.50
TEETH = 36
PITCH = 5.0
TOOTH_START = FLANGE + 1.5
TOOTH_WIDTH = 17.0
REAR_PCD = 38.0
REAR_CLEARANCE = 3.4  # Shared plate clearance only. Motor threads depend on the supplied revision.
OUTPUT_PCD = 27.0


def bolt_centres(count, pcd, first):
    for i in range(count):
        angle = math.radians(first + i * 360 / count)
        yield pcd / 2 * math.cos(angle), CENTRE + pcd / 2 * math.sin(angle)


def toothed_rim():
    """Nominal HTD 5M groove, circular root/edge blends and 14-degree flanks.

    Tangency to the actual OD circle is solved from its radius; angles in radians.
    D&D Timing Pulleys dimension table: 2.16 depth, R1.56, R0.48, 14 degrees.
    """
    od = TEETH * PITCH / (2 * math.pi) - 0.5715
    depth, root, tip = 2.16, 1.56, 0.48
    half_angle = math.radians(7)
    nx, ny = math.cos(half_angle), -math.sin(half_angle)
    dx, dy = -ny, nx
    root_y = od - depth + root
    foot = (root * nx, root_y + root * ny)
    ax, ay = (root + tip) * nx, root_y + (root + tip) * ny
    b = ax * dx + ay * dy
    t = -b + math.sqrt(b*b + (od-tip)**2 - ax*ax - ay*ay)
    cx, cy = ax + t*dx, ay + t*dy
    flank = (cx-tip*nx, cy-tip*ny)
    top = (od*cx/(od-tip), od*cy/(od-tip))
    def mirror(p):
        return -p[0], p[1]
    def midpoint_on_blend():
        vx = -nx + cx/(od-tip)
        vy = -ny + cy/(od-tip)
        length = math.hypot(vx, vy)
        return cx+tip*vx/length, cy+tip*vy/length
    blend = midpoint_on_blend()
    step = 2*math.pi / TEETH
    outline = BRepBuilderAPI_MakeWire()
    for i in range(TEETH):
        angle = i*step
        def point(p, offset=0):
            a = angle + offset
            return gp_Pnt(p[0]*math.cos(a)+p[1]*math.sin(a),
                          CENTRE-p[0]*math.sin(a)+p[1]*math.cos(a), TOOTH_START)
        def arc(a, mid, end):
            curve = GC_MakeArcOfCircle(a, mid, end).Value()
            outline.Add(BRepBuilderAPI_MakeEdge(curve).Edge())
        def line(a, end):
            outline.Add(BRepBuilderAPI_MakeEdge(point(a), point(end)).Edge())
        arc(point(mirror(top)), point(mirror(blend)), point(mirror(flank)))
        line(mirror(flank), mirror(foot))
        arc(point(mirror(foot)), point((0, od-depth)), point(foot))
        line(foot, flank)
        arc(point(flank), point(blend), point(top))
        arc(point(top), point((0, od), step/2), point(mirror(top), step))
    face = BRepBuilderAPI_MakeFace(outline.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0,0,TOOTH_WIDTH)).Shape()


def make_pulley(cyl, cut, fuse):
    # Only the raised inner foot seats on the rotor. The wide rim clears the casing.
    shape = fuse(cyl(16.5, FLANGE, TOOTH_START, 0, CENTRE), toothed_rim())
    shape = cut(shape, cyl(5, FLANGE-1, TOOTH_START+TOOTH_WIDTH+1, 0, CENTRE))
    for x, y in bolt_centres(6, OUTPUT_PCD, 90):
        shape = cut(shape, cyl(1.7, FLANGE-1, TOOTH_START+TOOTH_WIDTH+1, x, y))
        shape = cut(shape, cyl(3.1, FLANGE+4.0, TOOTH_START+TOOTH_WIDTH+1, x, y))
    return shape
