"""Reproduce the cuff geometry review; this is not a physical validation.

Run with Python 3. Reads the adjacent parameter file and prints JSON.
Units: mm and radians internally. No material, pressure or strength model.
"""
from pathlib import Path
import json
import math

params = json.loads(Path(__file__).with_name('CUFF_PARAMETERS_2026-11-17.json').read_text(encoding='utf-8'))
rows = []
for p in params['parts']:
    # Circular arm, uncompressed pad, neutral shell surface and slot centres.
    radius = p['circ'] / (2 * math.pi) + params['pad_thickness_proposal']
    neutral = p['radius'] + params['shell_thickness'] / 2
    tab_half = params['slot_end_tab'] / 2
    a = math.radians(params['shell_wrap_deg'] / 2)
    x = -neutral * math.cos(a) + tab_half * math.sin(a)
    y = neutral * math.sin(a) + tab_half * math.cos(a)
    distance = math.hypot(x, y)
    assert distance > radius
    theta = math.atan2(y, x) - math.acos(radius / distance)
    tangent_length = math.sqrt(distance**2 - radius**2)
    route = 2 * tangent_length + 2 * radius * theta
    contact = (radius * math.cos(theta), radius * math.sin(theta))
    tangent_dot = (x - contact[0]) * contact[0] + (y - contact[1]) * contact[1]
    assert abs(tangent_dot) < 1e-8
    # Independent sampled length check: two segments + 2000 arc chords.
    points = [(x, -y)] + [(radius * math.cos(t), radius * math.sin(t))
        for t in [-theta + 2 * theta * i / 2000 for i in range(2001)]] + [(x, y)]
    sampled = sum(math.dist(p0, p1) for p0, p1 in zip(points, points[1:]))
    assert abs(sampled - route) < 0.001
    rows.append({
        'id': p['id'], 'circular_pad_outer_radius_mm': radius,
        'slot_centre_xy_mm': [x, y], 'ideal_main_route_mm': route,
        'sampled_route_mm': sampled,
        'original_150mm_loop_minus_route_mm': 150 - route,
        'frame_inner_radius_with_original_3mm_block_mm': p['radius'] + params['shell_thickness'] + 3,
    })
f23, f16 = params['parts'][1:]
station_span = 180 - 70
delta = f23['radius'] - f16['radius']
result = {
    'status': 'REVIEW CALCULATIONS ONLY; NO FABRICATION OR POWERED-USE RELEASE',
    'assumptions': [
        'Circular concentric cross-sections; common straight forearm centreline.',
        '6mm uncompressed pad; zero-thickness webbing; slot centreline endpoints.',
        'Straight tangent tab; ignores slot-edge bending, seams, compression and human anatomy.',
        'Linear taper between measured forearm stations; local edge taper is illustrative.',
        'All dimensions are proposal inputs, not measured assembled hardware.',
    ],
    'strap_routes': rows,
    'forearm': {
        'station_span_mm': station_span,
        'original_mount_face_offset_difference_mm': delta,
        'frame_tilt_if_joining_face_centres_deg': math.degrees(math.atan(delta / station_span)),
        'wrist_block_example_for_parallel_frame_mm': 3 + delta,
        'example_is_not_a_released_dimension': True,
        'linear_radius_change_over_45mm_cuff_mm': (f23['circ'] - f16['circ']) / (2 * math.pi) / station_span * params['shell_axial_width'],
    },
    'checks': {'tangency_and_sampled_route_agree': True},
}
print(json.dumps(result, ensure_ascii=False, indent=2))
