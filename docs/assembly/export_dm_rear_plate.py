"""Extract the shoulder-side motor mounting sample from the assembly STEP (mm).

The square motor pad and short shoulder rail are the actual upper-frame solid.
Run next to the assembly and the saved manufacturer reference, with cadquery-ocp.
"""
from collections import Counter
from pathlib import Path
import math

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.StlAPI import StlAPI_Writer

import build_assembly as cad
import damiao_drive as dm
from export_dm_fit_print import read_triangles

ROOT = Path(__file__).resolve().parent
NAME = "DM4340P_REAR_PLATE_FIT_ONLY"


def read_step(path):
    reader = cad.STEPControl_Reader()
    assert reader.ReadFile(str(path)) == cad.IFSelect_RetDone, path
    assert reader.TransferRoots() > 0
    return reader.OneShape()


def upper_frame_sample():
    assembly = read_step(ROOT / "ASSEMBLY_REFERENCE_NOT_FOR_FABRICATION.step")
    solids = cad.TopExp_Explorer(assembly, cad.TopAbs_SOLID)
    matches = []
    while solids.More():
        solid = solids.Current()
        if np.allclose(cad.bbox(solid), [-35, -32, 61.5, 35, 220, dm.REAR], atol=0.001):
            matches.append(solid)
        solids.Next()
    assert len(matches) == 1, "Expected the upper frame from the DM4340P layout"
    # Cut at the elbow-side edge of the 70 mm pad and keep the rail to the shoulder.
    window = cad.box(-40, 40, dm.CENTRE - 35, 221, 60, 67)
    sample = cad.BRepAlgoAPI_Common(matches[0], window).Shape()
    unify = ShapeUpgrade_UnifySameDomain(sample, True, True, False)
    unify.Build()
    shift = cad.gp_Trsf()
    shift.SetTranslation(cad.gp_Vec(0, -dm.CENTRE, 4 - dm.REAR))
    return cad.BRepBuilderAPI_Transform(unify.Shape(), shift, True).Shape()


def motor_hole_check(plate):
    motor = read_step(ROOT / "references" / "DM4340P.step")
    faces = cad.TopExp_Explorer(motor, cad.TopAbs_FACE)
    rear_centres = set()
    while faces.More():
        surface = BRepAdaptor_Surface(cad.TopoDS.Face_s(faces.Current()))
        if surface.GetType() == GeomAbs_Cylinder:
            cyl = surface.Cylinder()
            x, y, z = cyl.Location().Coord()
            if (abs(y - 56.5) < 0.01 and abs(abs(cyl.Axis().Direction().Y()) - 1) < 0.001
                    and abs(cyl.Radius() - 1.25) < 0.01 and abs(math.hypot(x, z) - 19) < 0.01):
                rear_centres.add((round(x, 3), round(z, 3)))
        faces.Next()
    assert len(rear_centres) == 4
    position = cad.gp_Trsf()
    position.SetRotation(cad.gp_Ax1(cad.gp_Pnt(), cad.gp_Dir(1, 0, 0)), -math.pi / 2)
    position.SetTranslationPart(cad.gp_Vec(0, 0, 56.5 + 4))
    seated = cad.BRepBuilderAPI_Transform(motor, position, False).Shape()
    assert abs(cad.volume(cad.BRepAlgoAPI_Common(plate, seated).Shape())) < 0.001
    distance = cad.BRepExtrema_DistShapeShape(plate, seated)
    assert distance.IsDone() and distance.Value() < 0.001
    return rear_centres


def check_mesh(path, plate, rear_centres):
    faces = read_triangles(path)
    points, ids = np.unique(faces.reshape(-1, 3), axis=0, return_inverse=True)
    assert np.isfinite(points).all()
    np.testing.assert_allclose(points.min(axis=0), [-35, -35, 0], atol=0.001)
    np.testing.assert_allclose(points.max(axis=0), [35, 72.5, 4], atol=0.001)
    triangles = ids.reshape(-1, 3)
    edges = Counter((a, b) for tri in triangles for a, b in zip(tri, np.roll(tri, -1)))
    assert all(a != b and count == 1 and edges[b, a] == 1 for (a, b), count in edges.items())
    assert len(points) - len(edges) // 2 + len(triangles) == 2 - 2 * 6
    solid_volume = np.einsum('ij,ij->i', faces[:, 0], np.cross(faces[:, 1], faces[:, 2])).sum() / 6
    assert abs(solid_volume / cad.volume(plate) - 1) < 0.001
    expected = [(x, y, 1.7) for x, y in rear_centres] + [(0, 50.5, 2.25), (0, 64.5, 2.25)]
    # Every hole must open at both faces and stay cylindrical through the thickness.
    assert np.all(np.isclose(points[:, 2], 0) | np.isclose(points[:, 2], 4))
    for x, y, radius in expected:
        rings = []
        for height in [0, 4]:
            layer = points[np.isclose(points[:, 2], height), :2]
            delta = layer - [x, y]
            ring = delta[np.linalg.norm(delta, axis=1) < radius + 0.5]
            assert len(ring) >= 40
            np.testing.assert_allclose(np.linalg.norm(ring, axis=1), radius, atol=0.002)
            angles = np.sort(np.arctan2(ring[:, 1], ring[:, 0]))
            assert np.diff(np.r_[angles, angles[0] + 2 * math.pi]).max() < 0.13
            rings.append(ring)
        np.testing.assert_allclose(*rings, atol=0.002)
    return solid_volume


if __name__ == "__main__":
    plate = upper_frame_sample()
    step_path = ROOT / (NAME + ".step")
    exporter = cad.STEPControl_Writer()
    assert exporter.Transfer(plate, cad.STEPControl_AsIs) == cad.IFSelect_RetDone
    assert exporter.Write(str(step_path)) == cad.IFSelect_RetDone
    plate = read_step(step_path)
    assert cad.BRepCheck_Analyzer(plate).IsValid()
    solids = cad.TopExp_Explorer(plate, cad.TopAbs_SOLID)
    assert solids.More()
    solids.Next()
    assert not solids.More()
    rear_centres = motor_hole_check(plate)
    mesher = cad.BRepMesh_IncrementalMesh(plate, 0.015, False, 0.09, False)
    assert mesher.IsDone()
    stl_path = ROOT / (NAME + ".stl")
    stl = StlAPI_Writer()
    stl.ASCIIMode = False
    assert stl.Write(plate, str(stl_path))
    volume = check_mesh(stl_path, plate, rear_centres)
    print(f"Checked plate and motor in contact. Six open bores, closed STL, {volume / 1000:.2f} cm3.")
