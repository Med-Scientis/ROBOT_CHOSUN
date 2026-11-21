"""Elbow pulley and motor slide, nominal geometry for unpowered bench fit, mm.

Run with cadquery-ocp, numpy, and the assembly scripts in this directory.
The two print samples are separate from the proposed metal joint parts.
"""
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.StlAPI import StlAPI_Writer

import build_assembly as c
import damiao_drive as dm
from export_dm_fit_print import read_triangles
from export_dm_rear_plate import read_step

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "joint_drive"
BASE_Z = dm.REAR - 8
TRAVEL = 3
CLAMPS = [(x, y) for x in (-29, 29) for y in (-21, 21)]
box, cyl, cut, fuse = c.box, c.cyl, c.cut, c.fuse


def shift(shape, x=0, y=0, z=0):
    t = c.gp_Trsf()
    t.SetTranslation(c.gp_Vec(x, y, z))
    return c.BRepBuilderAPI_Transform(shape, t, False).Shape()


def unify(shape):
    tool = ShapeUpgrade_UnifySameDomain(shape, True, True, False)
    tool.Build()
    return tool.Shape()


def cylinder_along_y(radius, x, y0, y1, z):
    axis = c.gp_Ax2(c.gp_Pnt(x, y0, z), c.gp_Dir(0, 1, 0))
    return c.BRepPrimAPI_MakeCylinder(axis, radius, y1 - y0).Shape()


def slot(x, y, radius, travel):
    return fuse(cyl(radius, -1, 5, x, y - travel),
                cyl(radius, -1, 5, x, y + travel),
                box(x - radius, x + radius, y - travel, y + travel, -1, 5))


def motor_mount():
    # Local origin projects the motor centre onto the underside of the fixed plate.
    base = fuse(box(-37, 37, -38, 38, 0, 4),
                box(-15, 15, -44, -37, 0, 8), box(-15, 15, 37, 44, 0, 8),
                box(-15, 15, -48, -42, 4, 8), box(-15, 15, 42, 72.5, 4, 8),
                box(-37, -35.2, -37, 37, 4, 6), box(35.2, 37, -37, 37, 4, 6),
                box(-6, 6, -44, -38, 7, 16.5))
    base = cut(base, box(-23, 23, -24, 24, -1, 5))
    # At the shoulder end the metal cuff rises to Z58; the saddle begins at Y191.
    # Return to the original rail underside before meeting the saddle.
    base = cut(base, box(-16, 16, 34, 45, -1, 1.5))
    base = cut(base, box(-16, 16, 42, 45, -1, 4))
    for x, y in CLAMPS:
        base = cut(base, slot(x, y, 2.25, TRAVEL))
    for y in (50.5, 64.5):
        base = cut(base, cyl(2.25, 3, 9, 0, y))
    # Tap-drill representation only. Threads and clamping preload are not simulated.
    base = cut(base, cylinder_along_y(1.65, 0, -45, -37, 12))
    moving = fuse(box(-35, 35, -33, 28, 4, 8), box(-6, 6, -33, -29, 7, 16.5))
    for x, y in CLAMPS:
        moving = cut(moving, cyl(2.25, 3, 9, x, y))
    for x, y in dm.bolt_centres(4, dm.REAR_PCD, 45):
        moving = cut(moving, cyl(dm.REAR_CLEARANCE / 2, 3, 9, x, y - dm.CENTRE))
    return unify(base), unify(moving)


def elbow():
    # Reuse exactly the same 36-tooth profile and face width as the motor pulley.
    rim = shift(dm.toothed_rim(), y=-dm.CENTRE)
    wheel = fuse(rim, cyl(11, 118, 146), cyl(31, 122, 123.5), cyl(31, 140.5, 142))
    wheel = cut(cut(wheel, cyl(6, 117, 147)), box(0, 7.8, -2, 2, 117, 147))
    # Round-ended shaft seat and parallel key. The output-hub key lies outside this shaft sample.
    axis = c.gp_Ax2(c.gp_Pnt(3.5, 0, 124), c.gp_Dir(1, 0, 0))
    upper = c.gp_Ax2(c.gp_Pnt(3.5, 0, 140), c.gp_Dir(1, 0, 0))
    def rounded_key(length_x):
        return fuse(box(3.5, 3.5 + length_x, -2, 2, 124, 140),
                    c.BRepPrimAPI_MakeCylinder(axis, 2, length_x).Shape(),
                    c.BRepPrimAPI_MakeCylinder(upper, 2, length_x).Shape())
    shaft = cut(cyl(6, 108, 156), rounded_key(3))
    return unify(wheel), unify(shaft), unify(rounded_key(4))


def step(name, shapes, directory=OUT):
    path = directory / (name + ".step")
    writer = c.STEPControl_Writer()
    assert all(writer.Transfer(s, c.STEPControl_AsIs) == c.IFSelect_RetDone for s in shapes)
    assert writer.Write(str(path)) == c.IFSelect_RetDone
    reopened = read_step(path)
    assert c.BRepCheck_Analyzer(reopened).IsValid(), name
    bodies = c.TopExp_Explorer(reopened, c.TopAbs_SOLID)
    count = 0
    while bodies.More():
        count += 1
        bodies.Next()
    assert count == len(shapes), (name, count)
    assert abs(c.volume(reopened) - sum(c.volume(s) for s in shapes)) < 0.05


def export_sample(name, shape, directory=OUT):
    path = directory / (name + "_FIT_ONLY.stl")
    on_bed = shift(shape, z=-c.bbox(shape)[2])
    assert c.BRepMesh_IncrementalMesh(on_bed, 0.015, False, 0.09, False).IsDone()
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    assert writer.Write(on_bed, str(path))
    faces = read_triangles(path)
    pts, ids = np.unique(faces.reshape(-1, 3).round(5), axis=0, return_inverse=True)
    assert np.isfinite(pts).all()
    np.testing.assert_allclose([*pts.min(axis=0), *pts.max(axis=0)], c.bbox(on_bed), atol=0.02)
    tri = ids.reshape(-1, 3)
    edges = Counter((a, b) for f in tri for a, b in zip(f, np.roll(f, -1)))
    assert all(a != b and n == 1 and edges[b, a] == 1 for (a, b), n in edges.items())
    vol = np.einsum('ij,ij->i', faces[:, 0], np.cross(faces[:, 1], faces[:, 2])).sum() / 6
    assert abs(vol / c.volume(on_bed) - 1) < 0.001
    return {"size_mm": np.ptp(pts, axis=0).round(2).tolist(), "triangles": len(tri)}


def no_overlap(a, b):
    common = c.BRepAlgoAPI_Common(a, b)
    assert common.IsDone() and abs(c.volume(common.Shape())) < 0.001


def gap(a, b):
    check = c.BRepExtrema_DistShapeShape(a, b)
    assert check.IsDone()
    return check.Value()


def clamp_envelopes():
    return [fuse(cyl(2, -4, 8, x, y), cyl(4.5, 8, 12.5, x, y),
                  cyl(4.5, -4.5, 0, x, y)) for x, y in CLAMPS]


def verify_slide(base, slider, locks):
    motor = cyl(28.5, 8, 64.5)
    motor_heads = [cyl(3.4, 0.5, 4, x, y - dm.CENTRE)
                   for x, y in dm.bolt_centres(4, dm.REAR_PCD, 45)]
    for part in [motor, *motor_heads, *locks]:
        no_overlap(slider, part)
    for lock in locks:
        no_overlap(lock, motor)
    reference = read_step(ROOT / "references" / "DM4340P.step")
    pose = c.gp_Trsf()
    pose.SetRotation(c.gp_Ax1(c.gp_Pnt(), c.gp_Dir(1, 0, 0)), -math.pi / 2)
    pose.SetTranslationPart(c.gp_Vec(0, 0, 56.5 + 8))
    reference = c.BRepBuilderAPI_Transform(reference, pose, False).Shape()
    no_overlap(reference, slider)
    assert gap(reference, slider) < 0.001
    for lock in locks:
        no_overlap(reference, lock)
    for dy in (-TRAVEL, 0, TRAVEL):
        no_overlap(shift(reference, y=dy), base)
    gap_to_motor = min(gap(lock, motor) for lock in locks)
    assert gap_to_motor > 2
    for dy in range(-TRAVEL, TRAVEL + 1):
        for part in [slider, motor, *motor_heads, *locks]:
            no_overlap(base, shift(part, y=dy))
        # Axis height 12; head sits clear of the rail. Thread engagement idealized.
        tip = -33 + dy
        screw_head = cylinder_along_y(3.5, 0, tip - 24, tip - 20, 12)
        screw = cylinder_along_y(2, 0, tip - 20, tip, 12)
        no_overlap(screw_head, base)
        no_overlap(screw, shift(slider, y=dy))
        assert abs(gap(screw, shift(slider, y=dy))) < 0.001
        assert tip >= -38 and tip - 20 <= -44
    return round(gap_to_motor, 3)


def main():
    OUT.mkdir(exist_ok=True)
    base, slider = motor_mount()
    wheel, shaft, key = elbow()
    for a, b in [(wheel, shaft), (wheel, key), (shaft, key)]:
        no_overlap(a, b)
        assert gap(a, b) < 0.001
    bearings = [c.ring(14, 6, 108, 116), c.ring(14, 6, 148, 156)]
    spacers = [c.ring(9, 6, 116, 118), c.ring(9, 6, 146, 148)]
    for bearing, spacer in zip(bearings, spacers):
        assert abs(gap(wheel, bearing) - 2) < 0.001
        assert gap(wheel, spacer) < 0.001 and gap(bearing, spacer) < 0.001
        no_overlap(wheel, spacer)
    bridge = box(-42, -36, -18, 18, 116, 148)
    assert abs(gap(bridge, wheel) - 5) < 0.001
    locks = clamp_envelopes()
    clamp_gap = verify_slide(base, slider, locks)
    # Restore arm coordinates; preserve the unmodified elbow end of the upper frame.
    c.PARTS.clear()
    c.create_model()
    parts = {p['id']: p['shape'] for p in c.PARTS}
    upper_stub = c.BRepAlgoAPI_Common(parts['upper_frame_mount_envelope'],
                                      box(-55, 55, -40, dm.CENTRE - 48, 50, 75)).Shape()
    fixed = unify(fuse(upper_stub, shift(base, y=dm.CENTRE, z=BASE_Z)))
    moving = shift(slider, y=dm.CENTRE, z=BASE_Z)
    for part in ['U31_shoulder_saddle', 'U31_elbow_saddle', 'U31_shoulder', 'U31_elbow',
                 'upper_arm_guide', 'U31_shoulder_pad', 'U31_elbow_pad']:
        no_overlap(fixed, parts[part])
    for dy in (-TRAVEL, 0, TRAVEL):
        fitted = shift(moving, y=dy)
        no_overlap(fixed, fitted)
        no_overlap(shift(parts['motor_envelope'], y=dy), fitted)
        for lock in locks:
            placed_lock = shift(lock, y=dm.CENTRE + dy, z=BASE_Z)
            for adjacent in ['upper_arm_guide', 'U31_shoulder', 'U31_elbow',
                             'U31_shoulder_pad', 'U31_elbow_pad']:
                no_overlap(placed_lock, parts[adjacent])
    pulley_z = (dm.TOOTH_START + dm.TOOTH_WIDTH / 2)
    assert abs(pulley_z - 132) < 0.001 and abs(pulley_z - (123.5 + 140.5) / 2) < 0.001
    assert 2 * dm.CENTRE + dm.TEETH * dm.PITCH == 475
    step("ELBOW_36T_KEYED_REVIEW", [wheel])
    # Nominal screw root for display in the tapped hole. Major diameter was checked above.
    jack = fuse(cylinder_along_y(1.6, 0, -53, -33, 12),
                cylinder_along_y(3.5, 0, -57, -53, 12))
    hardware = [shift(s, y=dm.CENTRE, z=BASE_Z) for s in [*locks, jack]]
    step("UPPER_DRIVE_LAYOUT_REVIEW", [fixed, moving, parts['motor_envelope'], parts['motor_pulley'],
         wheel, shaft, key, *bearings, *spacers, parts['housing_A'], parts['housing_B'], bridge,
         parts['belt_path_reservation'], *hardware])
    samples = {name: export_sample(name, shape) for name, shape in
               [('DM4340P_SLIDE_BASE', base), ('DM4340P_SLIDE_CARRIER', slider)]}
    report = {"status": "Nominal CAD, unpowered fit only", "nominal_centres_mm": dm.CENTRE,
              "bench_slide_limits_mm": [dm.CENTRE - TRAVEL, dm.CENTRE + TRAVEL],
              "travel_samples_mm": list(range(-TRAVEL, TRAVEL + 1)),
              "motor_and_elbow_belt_plane_z_mm": pulley_z, "nominal_belt_pitch_length_mm": 475,
              "bearing_to_pulley_hub_mm_each": 2, "bridge_to_flange_mm": 5,
              "motor_envelope_to_clamp_head_mm": clamp_gap, "print_files": samples,
              "limits": "Nominal envelopes only; cable, machined tolerances, clamp force, strength, pretension and belt insertion path unverified."}
    (OUT / "checks.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
