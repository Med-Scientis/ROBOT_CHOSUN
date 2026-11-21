"""Export the existing unpowered fit-check STEP wide face down; verify the actual STL."""
from collections import Counter, defaultdict
from pathlib import Path
import math
import struct
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.StlAPI import StlAPI_Writer
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

ROOT = Path(__file__).resolve().parent
NAME = "DM4340P_36T_FIT_ONLY"
H = 18.5


def write_mesh():
    reader = STEPControl_Reader()
    assert reader.ReadFile(str(ROOT / (NAME + ".step"))) == IFSelect_RetDone
    reader.TransferRoots()
    transform = gp_Trsf()
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 0, 0)), math.pi)
    transform.SetTranslationPart(gp_Vec(0, 0, H))
    shape = BRepBuilderAPI_Transform(reader.OneShape(), transform, True).Shape()
    mesher = BRepMesh_IncrementalMesh(shape, 0.015, False, 0.09, False)
    assert mesher.IsDone()
    out = ROOT / (NAME + ".stl")
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    assert writer.Write(shape, str(out))
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return out, props.Mass()


def read_triangles(path):
    data = path.read_bytes()
    n, = struct.unpack_from('<I', data, 80)
    assert len(data) == 84 + 50*n
    records = np.frombuffer(data, offset=84, dtype=[('normal','<f4',3),
                            ('points','<f4',(3,3)),('attribute','<u2')])
    return records['points'].astype(float)


def check_section(triangles, z, mounting_radius):
    # Closed plane sections recover the six fastener bores, central bore and tooth outline.
    neighbours = defaultdict(set)
    for face in triangles:
        if min(face[:,2]) >= z or max(face[:,2]) <= z:
            continue
        points = []
        for a,b in zip(face, np.roll(face,-1,axis=0)):
            if (a[2]-z)*(b[2]-z) < 0:
                p = a[:2] + (z-a[2])/(b[2]-a[2])*(b[:2]-a[:2])
                points.append(tuple(p.round(5)))
        assert len(points) == 2
        a,b = points
        if a != b:
            neighbours[a].add(b)
            neighbours[b].add(a)
    assert all(len(v)==2 for v in neighbours.values())
    loops = []
    while neighbours:
        start = next(iter(neighbours))
        cur, prev = start, None
        loop = []
        while cur in neighbours:
            loop.append(cur)
            links = neighbours.pop(cur)
            nxt = next(p for p in links if p != prev)
            prev, cur = cur, nxt
        assert cur == start
        loops.append(np.array(loop))
    assert len(loops)==8
    loops.sort(key=lambda p: np.ptp(p[:,0]), reverse=True)
    outer = np.linalg.norm(loops[0], axis=1)
    assert abs(max(outer)-(36*5/(2*math.pi)-0.5715))<0.03
    high = outer > (max(outer)+min(outer))/2
    assert np.count_nonzero(high & ~np.roll(high,1))==36
    expected = [(13.5*math.cos(math.radians(90+60*i)),
                 13.5*math.sin(math.radians(90+60*i))) for i in range(6)]
    for loop in loops[1:]:
        centre = (loop.max(axis=0)+loop.min(axis=0))/2
        r = np.linalg.norm(loop-centre,axis=1)
        if np.linalg.norm(centre)<0.05:
            assert np.max(abs(r-5))<0.03
        else:
            nearest = min(expected, key=lambda point: np.linalg.norm(centre-point))
            assert np.linalg.norm(centre-nearest)<0.03
            expected.remove(nearest)
            assert np.max(abs(r-mounting_radius))<0.03
    assert not expected


def verify(path, cad_volume):
    faces = read_triangles(path)
    nodes = faces.reshape(-1,3)
    assert np.isfinite(nodes).all()
    assert abs(nodes[:,2].min())<1e-5 and abs(nodes[:,2].max()-H)<1e-5
    verts, ids = np.unique(nodes.round(5), axis=0, return_inverse=True)
    tri = ids.reshape(-1,3)
    assert len(np.unique(np.sort(tri,axis=1),axis=0))==len(tri)
    links = Counter((a,b) for face in tri for a,b in zip(face,np.roll(face,-1)))
    assert all(a!=b and count==1 and links[b,a]==1 for (a,b),count in links.items())
    assert len(verts)-len(links)//2+len(tri)==2-2*7
    volumes = np.einsum('ij,ij->i',faces[:,0],np.cross(faces[:,1],faces[:,2]))
    volume = volumes.sum()/6
    assert abs(volume/cad_volume-1)<0.001
    for z, radius in [(8.3,3.1),(15.7,1.7)]:
        check_section(faces,z,radius)
    print(f"{len(tri)} oriented triangles, {volume/1000:.2f} cm3 solid; "
          "36 teeth, 6 holes at PCD 27, diameter 3.4 and counterbore 6.2 mm.")


if __name__ == "__main__":
    verify(*write_mesh())
