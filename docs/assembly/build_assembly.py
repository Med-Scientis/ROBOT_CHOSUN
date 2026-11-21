"""Second Muscle: common-coordinate 3D packaging model, NOT fabrication CAD.

Python 3 + cadquery-ocp-novtk 7.9.3.1.1. Run from any directory.
Set PYTHONPATH to an isolated installation of that package if needed.
All output dimensions are mm. Detailed fasteners, teeth, keys, fits, guards,
stops and cable bends are deliberately not represented as finished designs.
"""
from pathlib import Path
import math, json, argparse
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Ax1,gp_Trsf,gp_Vec
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakeCone
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse,BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform,BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.STEPControl import STEPControl_Writer,STEPControl_Reader,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED,TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
import damiao_drive as dm

ROOT=Path(__file__).resolve().parent
PARTS=[]
F23_FRONT_ANGLE=25
def box(x0,x1,y0,y1,z0,z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0,y0,z0),x1-x0,y1-y0,z1-z0).Shape()
def cyl(r,z0,z1,x=0,y=0):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z0),gp_Dir(0,0,1)),r,z1-z0).Shape()
def fuse(*shapes):
    s=shapes[0]
    for q in shapes[1:]:s=BRepAlgoAPI_Fuse(s,q).Shape()
    return s
def cut(a,b):return BRepAlgoAPI_Cut(a,b).Shape()
def ring(ro,ri,z0,z1,x=0,y=0):return cut(cyl(ro,z0,z1,x,y),cyl(ri,z0-1,z1+1,x,y))
def bbox(s):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,False);return list(b.Get())
def volume(s):
    g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);return g.Mass()
def rotate(s,deg):
    t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),math.radians(deg))
    return BRepBuilderAPI_Transform(s,t,True).Shape()
def add(name,s,group,kind,role):
    assert BRepCheck_Analyzer(s).IsValid(),name
    assert volume(s)>0.001,(name,volume(s))
    PARTS.append({'id':name,'shape':s,'group':group,'kind':kind,'role':role,'bbox':bbox(s),'volume':volume(s)})
    return s
def cone(r0,r1,y0,y1,sector=360,start_angle=None):
    start=-sector/2 if start_angle is None else start_angle
    a=gp_Ax2(gp_Pnt(0,y0,0),gp_Dir(0,1,0),gp_Dir(math.sin(math.radians(start)),0,math.cos(math.radians(start))))
    if abs(r0-r1)<1e-9:return BRepPrimAPI_MakeCylinder(a,r0,y1-y0,math.radians(sector)).Shape()
    return BRepPrimAPI_MakeCone(a,r0,r1,y1-y0,math.radians(sector)).Shape()
def wire(pts):
    p=BRepBuilderAPI_MakePolygon()
    for v in pts:p.Add(gp_Pnt(*v))
    p.Close();return p.Wire()
def loft(q0,q1):
    l=BRepOffsetAPI_ThruSections(True,True);l.AddWire(wire(q0));l.AddWire(wire(q1));l.Build();return l.Shape()
def cuff(name,station,base_r,slope,frame_z,width,group):
    yc=station if group=='fixed' else -station
    y0,y1=yc-22.5,yc+22.5
    radius=lambda y:base_r+slope*(y-yc)
    front_angle=F23_FRONT_ANGLE if name=='F23' else 60
    sector=60+front_angle
    s=cut(cone(radius(y0)+2,radius(y1)+2,y0,y1,sector,-60),cone(radius(y0),radius(y1),y0,y1,sector,-60))
    for sign in [-1,1]:
        angle=math.radians(front_angle if sign==1 else 60)
        nr=(sign*math.sin(angle),math.cos(angle))
        tangent=(0,1) if name=='F23' and sign==1 else (sign*math.cos(angle),-math.sin(angle))
        thickness_direction=(1,0) if name=='F23' and sign==1 else nr
        def section(y):
            r=radius(y)
            return [(nr[0]*r+thickness_direction[0]*t+tangent[0]*u,y,nr[1]*r+thickness_direction[1]*t+tangent[1]*u) for t,u in [(0,0),(2,0),(2,20),(0,20)]]
        tab=loft(section(y0),section(y1))
        # 33mm along arm, 8mm along tab. Square envelope, not finished slot ends.
        def slotsection(y):
            r=radius(y)
            return [(nr[0]*r+thickness_direction[0]*t+tangent[0]*u,y,nr[1]*r+thickness_direction[1]*t+tangent[1]*u) for t,u in [(-2,6),(4,6),(4,14),(-2,14)]]
        tab=cut(tab,loft(slotsection(yc-16.5),slotsection(yc+16.5)))
        s=fuse(s,tab)
    for yy in [yc-7,yc+7]:s=cut(s,cyl(2.25,0,100,0,yy))
    add(name,s,group,'cuff','Tapered proposal with unfinished slot envelope')
    # Saddle underside follows the same cone; top is the shared frame plane.
    saddle=cut(box(-width/2,width/2,yc-14,yc+14,0,frame_z),cone(radius(yc-15)+2,radius(yc+15)+2,yc-15,yc+15))
    for yy in [yc-7,yc+7]:saddle=cut(saddle,cyl(2.25,0,100,0,yy))
    add(name+'_saddle',saddle,group,'saddle','Position-specific curved block; material/fastening pending')
    pad=cut(cone(radius(yc-27.5),radius(yc+27.5),yc-27.5,yc+27.5),cone(radius(yc-27.5)-6,radius(yc+27.5)-6,yc-27.5,yc+27.5))
    add(name+'_pad',pad,group,'pad','Nominal full-wrap 6mm pad; compression not simulated')

def create_model():
    cuff('U31_elbow',80,56,0,61.5,30,'fixed')
    cuff('U31_shoulder',205,56,0,61.5,30,'fixed')
    cuff('F23',70,43,0.1,52,25,'moving')
    cuff('F16',180,32,0.1,52,25,'moving')
    # Relieve the forward/lower head corner that hit the rotating offset wall.
    upper=fuse(box(-15,15,30,220,61.5,65.5),box(-26,-16,-32,40,61.5,65.5),box(-26,15,20,40,61.5,65.5),box(-35,35,112.5,182.5,61.5,65.5))
    upper=cut(upper,cyl(14,60,67))
    for y in [73,87,198,212]:upper=cut(upper,cyl(2.25,60,67,0,y))
    for x, y in dm.bolt_centres(4, dm.REAR_PCD, 45):
        upper=cut(upper,cyl(dm.REAR_CLEARANCE/2,60,67,x,y))
    add('upper_frame_mount_envelope',upper,'fixed','frame','Shared 3.4 mm plate clearance for M2.5/M3 at PCD 38. Check seating, location and alignment with actual hardware; tensioning not final.')
    add('motor_envelope',cyl(28.5,dm.REAR,dm.FLANGE,0,dm.CENTRE),'fixed','motor','DM4340P housing clearance cylinder; simplified connectors and rotor')
    add('motor_pulley',dm.make_pulley(cyl,cut,fuse),'drive','pulley','36 tooth HTD5M nominal, flange mounting and counterbores. Unpowered fit reference.')
    add('bearing_A',ring(14,6,108,116),'fixed','bearing','6001 nominal 12x28x8 ring envelope')
    add('bearing_B',ring(14,6,148,156),'fixed','bearing','6001 nominal 12x28x8 ring envelope')
    for name,z0,z1 in [('housing_A',104,116),('housing_B',148,160)]:
        housing=fuse(box(-26,26,-32,32,z0,z1),box(-42,-26,-18,18,z0,z1))
        add(name,cut(housing,cyl(14,z0-1,z1+1)),'fixed','support','Bearing housing envelope; shoulders/covers/fit not designed')
    columns=fuse(box(-26,-20,-32,-24,65.5,104),box(-26,-20,24,32,65.5,104))
    add('inboard_rear_supports',columns,'fixed','support','Rear-side support pair; attachment and stiffness pending')
    add('bearing_bridge',box(-42,-36,-18,18,116,148),'fixed','support','Bridge behind belt path; assembled fastening pending')
    add('joint_pulley_envelope',ring(31,6,118,146),'moving','pulley','28mm reservation, pitch centre Z132; teeth/key not represented')
    add('shaft_envelope',cyl(6,80,162),'moving','shaft','12mm shaft envelope; keyways/retention/tolerances pending')
    for name,z0,z1 in [('hub_spacer',106,108),('pulley_spacer_A',116,118),('pulley_spacer_B',146,148),('retainer_inner',82,86),('retainer_outer',156,160)]:
        add(name,ring(9,6,z0,z1),'moving','shaft','Axial reservation; bearing ring contact and retention pending')
    hub=fuse(ring(25,6,86,90),ring(12,6,90,106))
    for deg in [45,135,225,315]:
        a=math.radians(deg);hub=cut(hub,cyl(2.25,85,91,16*math.cos(a),16*math.sin(a)))
    add('inboard_forearm_hub',hub,'moving','frame','Moved inboard of bearing A; 20mm reservation with 4-hole flange proposal')
    lower=fuse(ring(25,10,82,86),box(-12.5,12.5,-42,0,82,86),box(-12.5,12.5,-42,-36,52,86),box(-12.5,12.5,-210,-38,52,56))
    lower=cut(lower,cyl(10,81,87))
    for y in [-63,-77,-173,-187]:lower=cut(lower,cyl(2.25,50,58,0,y))
    for deg in [45,135,225,315]:
        a=math.radians(deg);lower=cut(lower,cyl(2.25,81,87,16*math.cos(a),16*math.sin(a)))
    add('stepped_forearm_frame',lower,'moving','frame','30mm offset; square envelope requires manufacturable corner/joint design')
    # Belt is a continuous pitch-path band, represented as a reserved annular capsule.
    inner=fuse(cyl(27.65,124.5,139.5),cyl(27.65,124.5,139.5,0,147.5),box(-27.65,27.65,0,147.5,124.5,139.5))
    outer=fuse(cyl(31.15,124.5,139.5),cyl(31.15,124.5,139.5,0,147.5),box(-31.15,31.15,0,147.5,124.5,139.5))
    add('belt_path_reservation',cut(outer,inner),'drive','belt','15mm width, conservative path band; not tooth engagement model')
    add('sensor_space',box(-15,15,-15,15,164,175),'fixed','reservation','Magnet/sensor/bracket space only; actual mounting and airgap not set')
    # Anatomy guides: only the measured regions, no invented elbow anatomy.
    add('upper_arm_guide',cone(310/(2*math.pi),310/(2*math.pi),57.5,232.5),'fixed','anatomy','Circular uncompressed guide; real arm cross-section unknown')
    r=lambda s:230/(2*math.pi)+(s-70)*(160-230)/(2*math.pi*110)
    add('forearm_guide',cone(r(202.5),r(47.5),-202.5,-47.5),'moving','anatomy','Linear taper guide limited to cuff region; no elbow model')

def meshes():
    result=[]
    for p in PARTS:
        BRepMesh_IncrementalMesh(p['shape'],0.7,False,0.25,True).Perform()
        vertices=[];faces=[];exp=TopExp_Explorer(p['shape'],TopAbs_FACE)
        while exp.More():
            f=TopoDS.Face_s(exp.Current());loc=TopLoc_Location();tr=BRep_Tool.Triangulation_s(f,loc)
            if tr is not None:
                offset=len(vertices)
                for i in range(1,tr.NbNodes()+1):vertices.append([round(v,4) for v in tr.Node(i).Transformed(loc.Transformation()).Coord()])
                for i in range(1,tr.NbTriangles()+1):
                    ids=list(tr.Triangle(i).Get())
                    if f.Orientation()==TopAbs_REVERSED:ids.reverse()
                    faces.append([offset+j-1 for j in ids])
            exp.Next()
        result.append({k:p[k] for k in ['id','group','kind','role','bbox']}|{'vertices':vertices,'faces':faces})
    return result

def overlap(a,b):return all(a[i+3]>b[i]+1e-6 and b[i+3]>a[i]+1e-6 for i in range(3))
def collision_sweep():
    fixed=[p for p in PARTS if p['group']=='fixed' and p['kind'] not in ['pad','reservation','anatomy','bearing']]
    moving=[p for p in PARTS if p['group']=='moving' and p['kind'] in ['frame','cuff','saddle']]
    guide=next(p for p in PARTS if p['id']=='upper_arm_guide')
    hits=[];anatomy=[];min_rigid={'distance_mm':float('inf')};min_guide={'distance_mm':float('inf')}
    def box_distance(a,b):return math.sqrt(sum(max(0,a[i]-b[i+3],b[i]-a[i+3])**2 for i in range(3)))
    for deg in range(0,121):
        for p in moving:
            shape=rotate(p['shape'],deg);bb=bbox(shape)
            for q in fixed+[guide]:
                minimum=min_guide if q['kind']=='anatomy' else min_rigid
                if box_distance(bb,q['bbox'])<minimum['distance_mm']:
                    ds=BRepExtrema_DistShapeShape(shape,q['shape']);ds.Perform()
                    assert ds.IsDone()
                    if ds.Value()<minimum['distance_mm']:minimum.update({'distance_mm':ds.Value(),'angle_deg':deg,'moving':p['id'],'fixed':q['id']})
                if overlap(bb,q['bbox']):
                    v=volume(BRepAlgoAPI_Common(shape,q['shape']).Shape())
                    if v>0.1:
                        record={'angle_deg':deg,'moving':p['id'],'fixed':q['id'],'intersection_mm3':round(v,3)}
                        (anatomy if q['kind']=='anatomy' else hits).append(record)
        if deg%20==0:print('Sweep',deg,flush=True)
    return {'angle_start_deg':0,'angle_end_deg':120,'step_deg':1,'volume_threshold_mm3':0.1,
        'rigid_collisions':hits,'upper_arm_guide_intersections':anatomy,
        'minimum_rigid_gap':min_rigid,'minimum_upper_arm_guide_gap':min_guide,
        'scope':'Moving frame/hub/cuffs/saddles against fixed frame/motor/cuffs/saddles/supports; pads/soft straps/fasteners/guards/stops/cables excluded. Sampled poses, not continuous-motion or safety approval.'}

def static_intersections():
    parts=[p for p in PARTS if p['kind'] not in ['pad','anatomy','reservation']]
    excluded=[{'belt_path_reservation','motor_pulley'},{'belt_path_reservation','joint_pulley_envelope'}]
    hits=[]
    for i,p in enumerate(parts):
        for q in parts[i+1:]:
            if {p['id'],q['id']} in excluded or not overlap(p['bbox'],q['bbox']):continue
            v=volume(BRepAlgoAPI_Common(p['shape'],q['shape']).Shape())
            if v>0.1:hits.append({'a':p['id'],'b':q['id'],'intersection_mm3':round(v,3)})
    return {'angle_deg':0,'positive_volume_intersections':hits,'excluded_envelope_pairs':[sorted(e) for e in excluded],
            'exclusion_reason':'The reserved belt band intentionally intersects both pulley profiles. Actual belt meshing is not simulated.'}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--skip-sweep',action='store_true');args=parser.parse_args()
    create_model()
    m=meshes();(ROOT/'assembly_mesh.json').write_text(json.dumps(m,separators=(',',':')),encoding='utf-8')
    metadata={'status':'PACKAGING PROPOSAL; FABRICATION HOLD','units':'mm','origin':'Ideal elbow axis / arm centreline intersection','axes':{'X':'flexion-forward','Y':'upper arm toward shoulder','Z':'outboard'},
        'motor':{'model':'Damiao DM-J4340P-2EC 24V','centre_Y':dm.CENTRE,'rear_Z':dm.REAR,'flange_Z':dm.FLANGE,'belt_plane_Z':dm.TOOTH_START+dm.TOOTH_WIDTH/2,'rear_PCD':dm.REAR_PCD,'rear_thread':'V1.0 M2.5 x4; alternate drawing M3 x4. Verify supplied revision.','rear_plate_clearance_diameter_mm':dm.REAR_CLEARANCE,'output_PCD':dm.OUTPUT_PCD,'output_pattern':'6 x M3 depth 6','tooth_count':dm.TEETH,'pitch_mm':dm.PITCH,'toothed_face_width':dm.TOOTH_WIDTH,'centring':'Register/dowels and printed hole tolerances not qualified.'},
        'joint':{'bearing_A':[108,116],'bearing_B':[148,156],'bearing_centre_span':40,'pulley_reservation':[118,146],'belt_plane_Z':132,'hub':[86,106],'shaft':[80,162]},
        'cuffs':{'upper_frame_inner_Z':61.5,'upper_block_centre':3.5,'forearm_frame_inner_Z':52,'F23_block_centre':7,'F16_block_centre':18,'forearm_shell_radius_formula':'R(s)=50-0.1*s, s distal from elbow','forearm_frame_offset':30,'F23_arc_degrees':[-60,F23_FRONT_ANGLE],'F23_front_tab_direction':'outboard +Z','other_cuff_arc_degrees':[-60,60],'F23_contact_arc_reduction_percent':100*(60-F23_FRONT_ANGLE)/120},
        'maximum_outboard_Z_with_sensor_reservation':175,'approx_projection_from_upper_arm_skin':175-310/(2*math.pi),
        'belt_pitch_length':475,'parts':[{k:v for k,v in p.items() if k!='shape'} for p in PARTS]}
    (ROOT/'ASSEMBLY_PARAMETERS.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    writer=STEPControl_Writer()
    for p in PARTS:
        if p['kind'] not in ['anatomy','pad','reservation']:writer.Transfer(p['shape'],STEPControl_AsIs)
    path=ROOT/'ASSEMBLY_REFERENCE_NOT_FOR_FABRICATION.step'
    assert writer.Write(str(path))==IFSelect_RetDone
    reader=STEPControl_Reader();assert reader.ReadFile(str(path))==IFSelect_RetDone;reader.TransferRoots()
    assert BRepCheck_Analyzer(reader.OneShape()).IsValid()
    joint_pulley=next(p for p in PARTS if p['id']=='joint_pulley_envelope')['bbox']
    motor_pulley=next(p for p in PARTS if p['id']=='motor_pulley')['bbox']
    motor_plane=motor_pulley[5]-dm.TOOTH_WIDTH/2
    joint_plane=(joint_pulley[2]+joint_pulley[5])/2
    centre_distance=((motor_pulley[1]+motor_pulley[4])-(joint_pulley[1]+joint_pulley[4]))/2
    assert abs(motor_plane-joint_plane)<1e-5 and abs(centre_distance-dm.CENTRE)<1e-5
    check={'all_parts_valid':True,'step_reopened_valid':True,'part_count':len(PARTS),'motor_tooth_plane_mm':round(motor_plane,4),'joint_face_midplane_mm':round(joint_plane,4),'belt_plane_error_mm':round(abs(motor_plane-joint_plane),4),'motor_centre_distance_mm':round(centre_distance,4),'physical_validation':False}
    if not args.skip_sweep:
        check['static_check']=static_intersections()
        check['motion_sweep']=collision_sweep()
    (ROOT/'ASSEMBLY_CHECKS.json').write_text(json.dumps(check,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'part_count':len(PARTS),'mesh_bytes':(ROOT/'assembly_mesh.json').stat().st_size,'step_bytes':path.stat().st_size,'checks':check},ensure_ascii=False))

if __name__=='__main__':main()
