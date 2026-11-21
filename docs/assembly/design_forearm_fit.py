"""Forearm hub design study and full-size, hand-operated fit mockup (mm)."""
import json
import math
import shutil
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import build_assembly as c
import design_joint_drive as drive
from design_joint_drive import box, cyl, cut, fuse, gap, no_overlap, shift, unify

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "forearm_fit"
ORDER = OUT / "print"
BRIDGE_X = -22
POST_Y = 28
FOOT_Y = 20.5
HUB_BOLTS = [(16 * math.cos(math.radians(a)), 16 * math.sin(math.radians(a)))
             for a in (45, 135, 225, 315)]


def tilt_x(shape, degrees):
    tr = c.gp_Trsf()
    tr.SetRotation(c.gp_Ax1(c.gp_Pnt(), c.gp_Dir(1, 0, 0)), math.radians(degrees))
    return c.BRepBuilderAPI_Transform(shape, tr, True).Shape()


def solid(shape):
    assert c.BRepCheck_Analyzer(shape).IsValid()
    it = c.TopExp_Explorer(shape, c.TopAbs_SOLID)
    assert it.More()
    it.Next()
    assert not it.More()
    return shape


def hub_blank(hole_radius):
    flange = fuse(cyl(25, 86, 92), cyl(11, 92, 106))
    for x, y in HUB_BOLTS:
        flange = cut(flange, cyl(hole_radius, 85, 93, x, y))
    return flange


def hub_key(radial_length):
    # Rounded DIN form A profile, total axial length 16. Square cross-section 4 x 4.
    key = box(3.5, 3.5 + radial_length, -2, 2, 90, 102)
    for z in (90, 102):
        axis = c.gp_Ax2(c.gp_Pnt(3.5, 0, z), c.gp_Dir(1, 0, 0))
        key = fuse(key, c.BRepPrimAPI_MakeCylinder(axis, 2, radial_length).Shape())
    return unify(key)


def metal_joint(parts):
    wheel, shaft_middle, wheel_key = drive.elbow()
    hub = cut(cut(hub_blank(1.65), cyl(6, 85, 107)), box(0, 7.8, -2, 2, 85, 107))
    shaft = fuse(cyl(8.5, 82, 86), cyl(6, 86, 108), shaft_middle,
                 cyl(6, 156, 158.2), cyl(5.75, 158.2, 159.3), cyl(6, 159.3, 162))
    shaft = solid(unify(cut(shaft, hub_key(3))))
    hub = solid(unify(hub))
    key = solid(hub_key(4))
    bearing_a, bearing_b = parts['bearing_A'], parts['bearing_B']
    # The 6001 inner-ring abutment is 14..15.5 mm OD in the NSK listing.
    stack = [hub, c.ring(7.5, 6, 106, 108), bearing_a, c.ring(7.5, 6, 116, 118),
             wheel, c.ring(7.5, 6, 146, 148), bearing_b, c.ring(7.5, 6, 156, 158)]
    for a, b in zip(stack, stack[1:]):
        no_overlap(a, b)
        assert gap(a, b) < 0.001
    for part in [*stack, wheel_key, key, parts['stepped_forearm_frame']]:
        no_overlap(shaft, part)
    for a, b in [(hub, key), (shaft, key), (wheel, wheel_key), (shaft, wheel_key),
                 (parts['stepped_forearm_frame'], hub)]:
        no_overlap(a, b)
        assert gap(a, b) < 0.001
    for x, y in HUB_BOLTS:
        # Test clear axes over the frame and tap drill; nominal threads are cosmetic.
        no_overlap(cyl(2, 82, 86, x, y), parts['stepped_forearm_frame'])
        no_overlap(cyl(1.64, 86, 92, x, y), hub)
    for housing in ['housing_A', 'housing_B']:
        no_overlap(hub, parts[housing])
    drive.step("FOREARM_KEYED_JOINT_REVIEW", [solid(parts['stepped_forearm_frame']),
               shaft, key, wheel_key, *stack], OUT)
    drive.step("TWO_KEY_SHAFT_REVIEW", [shaft], OUT)
    drive.step("FOREARM_FLANGE_HUB_REVIEW", [hub], OUT)


def bench_bridge():
    # Separate open support for the smooth printed pin. It bolts to the elbow end
    # of the upper mock rail so no bearings or custom shaft are needed for a hand-fit check.
    top = fuse(box(-26, -18, -28, 28, 108, 116), box(-22, 0, -10, 10, 108, 116),
               cyl(14, 108, 116))
    for sign in (-1, 1):
        y0, y1 = sorted([sign * 16.5, sign * 31.5])
        foot = box(-26, -18, y0, y1, 65.5, 69.5)
        foot = cut(foot, cyl(1.7, 64, 71, BRIDGE_X, sign * FOOT_Y))
        top = fuse(top, cyl(3.25, 65.5, 116, BRIDGE_X, sign * POST_Y), foot)
    return solid(unify(cut(top, cyl(6.3, 107, 117))))


def bench_upper(parts):
    base, _ = drive.motor_mount()
    start = drive.dm.CENTRE - 48
    stub = c.BRepAlgoAPI_Common(parts['upper_frame_mount_envelope'],
                                box(-55, 55, -40, start, 50, 75)).Shape()
    upper = fuse(stub, shift(base, y=drive.dm.CENTRE, z=drive.BASE_Z))
    for y in (-FOOT_Y, FOOT_Y):
        upper = cut(upper, cyl(1.7, 60, 67, BRIDGE_X, y))
    return solid(unify(upper))


def check_motion(fixed, moving):
    fixed_boxes = [c.bbox(shape) for shape in fixed]
    for angle in range(121):
        for part in moving:
            turned = c.rotate(part, angle)
            bounds = c.bbox(turned)
            for target, target_box in zip(fixed, fixed_boxes):
                if c.overlap(bounds, target_box):
                    no_overlap(turned, target)


def add_sample(name, part, quantity, label, listing, angle_x=0, diagonal=False):
    part = solid(part)
    posed = tilt_x(part, angle_x) if angle_x else part
    if diagonal:
        posed = c.rotate(posed, 45)
    x0, y0, _, x1, y1, _ = c.bbox(posed)
    centered = shift(posed, x=-(x0 + x1)/2, y=-(y0 + y1)/2)
    meta = drive.export_sample(name, centered, ORDER)
    listing.append(dict(file=name + "_FIT_ONLY.stl", qty=quantity, name=label, **meta))


def main():
    ORDER.mkdir(exist_ok=True, parents=True)
    c.PARTS.clear()
    c.create_model()
    p = {entry['id']: entry['shape'] for entry in c.PARTS}
    metal_joint(p)

    upper = bench_upper(p)
    lower = p['stepped_forearm_frame']
    bridge = bench_bridge()
    pin = solid(unify(cut(fuse(hub_blank(2.25), cyl(8.5, 106, 108),
                               cyl(5.9, 108, 116.8)), cyl(2.25, 85, 118))))
    for a, b in [(upper, bridge), (pin, lower), (pin, bridge)]:
        no_overlap(a, b)
        assert gap(a, b) < 0.001
    # The retaining washer bears on the pin; it must not clamp the fixed member.
    washer = c.ring(8, 2.25, 116.8, 118.3)
    assert abs(gap(washer, bridge) - 0.8) < 0.001
    pivot_bolt = fuse(cyl(3.9, 82.5, 86), cyl(2, 86, 126))
    pivot_nut = c.ring(4.1, 2, 118.3, 122.3)
    for fitting in [pivot_bolt, pivot_nut, washer]:
        for adjacent in [pin, bridge, lower]:
            no_overlap(fitting, adjacent)
    for x, y in HUB_BOLTS:
        no_overlap(cyl(2, 82, 92, x, y), fuse(lower, pin))
    feet_hardware = []
    for y in (-FOOT_Y, FOOT_Y):
        shank = cyl(1.5, 61.5, 73.5, BRIDGE_X, y)
        no_overlap(bridge, shank)
        no_overlap(upper, shank)
        nut = cyl(3.25, 69.5, 72.5, BRIDGE_X, y)
        no_overlap(bridge, nut)
        feet_hardware.append(fuse(shank, nut, cyl(3, 58.5, 61.5, BRIDGE_X, y)))
    hub_hardware = [fuse(cyl(2, 82, 98, x, y), cyl(4.5, 78.5, 82, x, y),
                         cyl(4.1, 92, 96.5, x, y)) for x,y in HUB_BOLTS]
    for bolt in hub_hardware:
        no_overlap(bolt, pin)
        no_overlap(bolt, lower)
    # Joining each shell to its shaped saddle removes four loose pieces from the quote.
    cuffs = {k: solid(unify(fuse(p[k], p[k+'_saddle']))) for k in
             ['U31_elbow', 'U31_shoulder', 'F23', 'F16']}
    u31 = shift(cuffs['U31_elbow'], y=125)
    assert abs(c.volume(c.BRepAlgoAPI_Common(u31, cuffs['U31_shoulder']).Shape())
               - c.volume(u31)) < 0.001
    for name, frame in [('U31_elbow', upper), ('U31_shoulder', upper),
                        ('F23', lower), ('F16', lower)]:
        no_overlap(cuffs[name], frame)
        assert gap(cuffs[name], frame) < 0.001
        station = {'U31_elbow':80, 'U31_shoulder':205,'F23':-70,'F16':-180}[name]
        for y in [station-7, station+7]:
            gauge = cyl(2, 0, 70, 0, y)
            no_overlap(gauge, cuffs[name])
            no_overlap(gauge, frame)
    _, carrier = drive.motor_mount()
    carrier = shift(carrier, y=drive.dm.CENTRE, z=drive.BASE_Z)
    # Sweep only potential new collisions. The static cylindrical pin/journal and
    # retaining washer are rotationally invariant. Cover rigid cuffs and fastener envelopes.
    check_motion([upper, bridge, *feet_hardware, carrier,
                  cuffs['U31_elbow'], cuffs['U31_shoulder']],
                 [lower, cuffs['F23'], cuffs['F16'], *hub_hardware])
    for target in [upper, bridge, *feet_hardware]:
        no_overlap(pin, target)
    drive.step("HAND_FIT_ASSEMBLY_REVIEW", [upper, carrier, bridge, pin, lower, *cuffs.values(),
               washer, pivot_bolt, pivot_nut, *feet_hardware, *hub_hardware], OUT)

    listing = []
    for file, source, label in [
        ("DM4340P_36T_FIT_ONLY.stl", ROOT, "모터 앞 톱니바퀴"),
        ("DM4340P_SLIDE_CARRIER_FIT_ONLY.stl", ROOT/"joint_drive", "움직이는 모터판")]:
        shutil.copyfile(source/file, ORDER/file)
        faces = drive.read_triangles(ORDER/file).reshape(-1,3)
        listing.append(dict(file=file, qty=1, name=label,
                            size_mm=drive.np.ptp(faces,axis=0).round(1).tolist()))
    add_sample("UPPER_FRAME_WITH_BASE", upper, 1, "위팔 프레임 · 고정 받침판 일체형", listing, diagonal=True)
    add_sample("FOREARM_FRAME", lower, 1, "아래팔 프레임", listing, diagonal=True)
    add_sample("ELBOW_PIVOT_SUPPORT", bridge, 1, "손 조립용 팔꿈치 축 받침대", listing, angle_x=180)
    add_sample("FOREARM_HUB_WITH_PIN", pin, 1, "손 조립용 아래팔 연결통 · 축 일체형", listing)
    add_sample("UPPER_CUFF_U31", cuffs['U31_elbow'], 2, "위팔 받침 · 둘레 약 31cm", listing, angle_x=90)
    add_sample("FOREARM_CUFF_F23", cuffs['F23'], 1, "팔꿈치 쪽 아래팔 받침 · 둘레 약 23cm", listing, angle_x=90)
    add_sample("FOREARM_CUFF_F16", cuffs['F16'], 1, "손목 쪽 아래팔 받침 · 둘레 약 16cm", listing, angle_x=90)
    report = {"print_files": listing, "unique_parts": len(listing), "total_quantity": sum(i["qty"] for i in listing),
              "hand_fit_sweep_deg": [0,120,1], "smooth_pin_mm":11.8, "journal_mm":12.6, "washer_clearance_mm":0.8}
    assert report["unique_parts"] == 9 and report["total_quantity"] == 10
    (OUT/"checks.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    lines = ["조선대학교 경진대회 | 팔꿈치 보조로봇 무동력 맞춤 모형", "",
             "STL 9종 / 합계 10개. 위팔 받침 U31만 2개, 나머지는 각 1개.",
             "PETG 단색, FDM, mm 단위, 배율 100%, 내부 채움 25%로 견적 요청합니다.", "",
             "수량    파일명    용도    저장 방향 가로 × 세로 × 높이(mm)"]
    for item in listing:
        sizes = " × ".join(f"{v:g}" for v in item["size_mm"])
        lines.append(f'{item["qty"]}개    {item["file"]}    {item["name"]}    {sizes}')
    lines.extend(["", "기본 서포트 제거 포함. 긴 프레임은 바닥에서 대각선으로 놓았습니다. "
                  "크기 조정이나 분할이 필요하면 제작 전에 알려 주세요.",
                  "두 판의 옆 간격은 한쪽 0.2mm입니다. 팔꿈치 시험축은 Ø11.8, 받침 구멍은 Ø12.6mm입니다. "
                  "핀, 끈 구멍, 조립면과 톱니의 서포트 자국·돌기를 정리해 주세요.",
                  "견적에 부가세·광주 배송비·예상 수령일을 표시해 주세요. "
                  "조선대학교 산학협력단 명의 후불, 날인 견적서·거래명세서·전자세금계산서 발행을 요청합니다."])
    manifest = ORDER/"출력_수량표_견적요청.txt"
    manifest.write_text("\n".join(lines)+"\n", encoding="utf-8-sig")
    files = [ORDER/i["file"] for i in listing]+[manifest]
    archive = OUT/"팔꿈치로봇_일괄시험출력.zip"
    with ZipFile(archive, 'w', ZIP_DEFLATED) as z:
        for path in files:
            z.write(path, path.name)
    with ZipFile(archive) as z:
        assert z.testzip() is None and set(z.namelist()) == {f.name for f in files}
        assert all(z.read(f.name) == f.read_bytes() for f in files)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
