"""Verify shaft patterns in the reference STEP and export the pulley on its own."""
import math
from pathlib import Path
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.gp import gp_Trsf, gp_Vec
import build_assembly as cad
import damiao_drive as dm

root = Path(__file__).resolve().parent
ref = root / 'references' / 'DM4340P.step'
reader = cad.STEPControl_Reader()
assert reader.ReadFile(str(ref)) == cad.IFSelect_RetDone
reader.TransferRoots()
motor = reader.OneShape()
faces = cad.TopExp_Explorer(motor, cad.TopAbs_FACE)
patterns = {'front':set(), 'rear':set()}
while faces.More():
    surface = BRepAdaptor_Surface(cad.TopoDS.Face_s(faces.Current()))
    if surface.GetType() == GeomAbs_Cylinder:
        cyl = surface.Cylinder()
        x, y, z = cyl.Location().Coord()
        # Supplied STEP outer face Y=0, back plate Y=56.5.
        if abs(cyl.Axis().Direction().Y()) > 0.9999 and abs(cyl.Radius()-1.25)<0.01:
            for side, plane, pcd in [('rear',56.5,dm.REAR_PCD), ('front',0,dm.OUTPUT_PCD)]:
                if abs(y-plane)<0.01 and abs(math.hypot(x,z)-pcd/2)<0.01:
                    patterns[side].add((round(x,3),round(z,3)))
    faces.Next()
for side, n, pcd, first in [('front',6,dm.OUTPUT_PCD,90),('rear',4,dm.REAR_PCD,45)]:
    expected={(round(x,3),round(y-dm.CENTRE,3)) for x,y in dm.bolt_centres(n,pcd,first)}
    assert expected == patterns[side], (side,expected,patterns[side])
local = gp_Trsf()
local.SetTranslation(gp_Vec(0,-dm.CENTRE,-dm.FLANGE))
pulley = cad.BRepBuilderAPI_Transform(dm.make_pulley(cad.cyl,cad.cut,cad.fuse),local,True).Shape()
assert cad.BRepCheck_Analyzer(pulley).IsValid()
writer = cad.STEPControl_Writer()
assert writer.Transfer(pulley,cad.STEPControl_AsIs)==cad.IFSelect_RetDone
out = root / 'DM4340P_36T_FIT_ONLY.step'
assert writer.Write(str(out))==cad.IFSelect_RetDone
reader = cad.STEPControl_Reader()
assert reader.ReadFile(str(out))==cad.IFSelect_RetDone
reader.TransferRoots()
reopened = reader.OneShape()
assert cad.BRepCheck_Analyzer(reopened).IsValid()
solids = cad.TopExp_Explorer(reopened,cad.TopAbs_SOLID)
count = 0
while solids.More():
    count += 1
    solids.Next()
assert count==1 and abs(cad.volume(reopened)-cad.volume(pulley))<0.001
print("Matched reference axes: 4 rear, 6 front. Exported and reopened one pulley solid.")
