#!/usr/bin/env python3
"""
trajectory_generator.py

Pure math, no ROS imports -- usable standalone, from a node, or in a unit
test. Turns a set of waypoints into a DENSE list of intermediate 3D points
along either a straight line or a circular arc, so you can trace a shape
instead of just jumping point-to-point.

Two different things live in this codebase now, and they're not the same
consumption pattern:
    1. autonomy_sequencer_node -- discrete targets, "visit, dwell, fire
       laser, move on." Built for a handful of well-separated points.
    2. This module -- a continuous path shape (line or arc) through a set
       of points, meant to be walked densely.

IMPORTANT caveat about feeding a dense path to the current ESP32 firmware:
Since CMD_MOVE_COORDINATE on the ESP32 currently does a full trapezoidal
accel/cruise/decel profile per point, sending it hundreds of closely-spaced
points back to back will look like stop-start motion, not a smooth sweep --
it decelerates to ~zero at every intermediate point before accelerating
into the next. That's fine for tracing at low speed / for laser marking
where dwell doesn't matter, but if you want genuinely smooth continuous
motion along the path you'll eventually need the ESP32 to support a
queued/blended move (look ahead to the next point and don't fully decelerate
first) -- that's a firmware change, not something this module can paper
over from the ROS2 side.
"""

import math


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def linear_segment(p0, p1, resolution=0.005):
    """Points from p0 to p1 (inclusive), spaced ~resolution meters apart."""
    length = _dist(p0, p1)
    if length < 1e-9:
        return [p0]
    n = max(1, math.ceil(length / resolution))
    return [_lerp(p0, p1, i / n) for i in range(n + 1)]


def linear_path(waypoints, resolution=0.005):
    """Chain linear_segment through an ordered list of >=2 waypoints,
    without duplicating the shared point at each junction."""
    if len(waypoints) < 2:
        raise ValueError('need at least 2 waypoints')
    path = [waypoints[0]]
    for p0, p1 in zip(waypoints, waypoints[1:]):
        path.extend(linear_segment(p0, p1, resolution)[1:])
    return path


def fit_circle_3pt(p0, p1, p2):
    """Fit the circle passing through 3 non-collinear 3D points.
    Returns (center, radius, normal) where normal is a unit vector
    orthogonal to the plane the 3 points lie in.
    Raises ValueError if the points are (near) collinear."""
    p0 = tuple(p0); p1 = tuple(p1); p2 = tuple(p2)

    v1 = tuple(p1[i] - p0[i] for i in range(3))
    v2 = tuple(p2[i] - p0[i] for i in range(3))

    # normal = v1 x v2
    normal = (
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    )
    normal_len = math.sqrt(sum(c * c for c in normal))
    if normal_len < 1e-9:
        raise ValueError('points are collinear -- no unique circle through them')
    normal = tuple(c / normal_len for c in normal)

    # Work in the plane's local 2D basis (u, w) with origin at p0.
    u = v1
    u_len = math.sqrt(sum(c * c for c in u))
    u = tuple(c / u_len for c in u)
    w = (  # w = normal x u  (completes an orthonormal basis in the plane)
        normal[1] * u[2] - normal[2] * u[1],
        normal[2] * u[0] - normal[0] * u[2],
        normal[0] * u[1] - normal[1] * u[0],
    )

    def to_2d(p):
        d = tuple(p[i] - p0[i] for i in range(3))
        return (sum(d[i] * u[i] for i in range(3)), sum(d[i] * w[i] for i in range(3)))

    a2 = (0.0, 0.0)
    b2 = to_2d(p1)
    c2 = to_2d(p2)

    # Circumcenter of the 2D triangle (standard formula).
    ax, ay = a2; bx, by = b2; cx, cy = c2
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        raise ValueError('points are collinear -- no unique circle through them')
    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d

    center_3d = tuple(p0[i] + ux * u[i] + uy * w[i] for i in range(3))
    radius = math.sqrt((ux - ax) ** 2 + (uy - ay) ** 2)
    return center_3d, radius, normal


def circular_arc_path(p_start, p_mid, p_end, resolution=0.005):
    """Points along the circular arc from p_start through p_mid to p_end,
    spaced ~resolution meters apart. p_mid fixes both the circle and which
    of the two possible arcs (short way / long way) between start and end
    to take -- same convention as G02/G03 with an intermediate point."""
    center, radius, normal = fit_circle_3pt(p_start, p_mid, p_end)

    # local basis in the circle's plane
    ref = tuple(p_start[i] - center[i] for i in range(3))
    ref_len = math.sqrt(sum(c * c for c in ref))
    u = tuple(c / ref_len for c in ref)
    w = (
        normal[1] * u[2] - normal[2] * u[1],
        normal[2] * u[0] - normal[0] * u[2],
        normal[0] * u[1] - normal[1] * u[0],
    )

    def angle_of(p):
        d = tuple(p[i] - center[i] for i in range(3))
        x = sum(d[i] * u[i] for i in range(3))
        y = sum(d[i] * w[i] for i in range(3))
        return math.atan2(y, x)

    a_start = 0.0  # by construction, p_start is along +u
    a_mid = angle_of(p_mid) % (2 * math.pi)
    a_end = angle_of(p_end) % (2 * math.pi)

    # choose the sweep direction (CCW vs CW) that passes through a_mid
    def normalize_sweep(end_angle):
        e = end_angle % (2 * math.pi)
        return e

    sweep_ccw = normalize_sweep(a_end)
    goes_through_mid_ccw = 0 <= a_mid <= sweep_ccw if sweep_ccw >= 0 else False
    sweep = sweep_ccw if (a_mid <= sweep_ccw) else -(2 * math.pi - sweep_ccw)

    arc_len = abs(sweep) * radius
    n = max(1, math.ceil(arc_len / resolution))

    points = []
    for i in range(n + 1):
        a = a_start + sweep * (i / n)
        x, y = radius * math.cos(a), radius * math.sin(a)
        points.append(tuple(center[j] + x * u[j] + y * w[j] for j in range(3)))
    return points


if __name__ == '__main__':
    # quick sanity check
    line = linear_path([(0, 0, 0), (0.1, 0, 0), (0.1, 0.1, 0)], resolution=0.02)
    print(f'linear_path: {len(line)} points')

    arc = circular_arc_path((0.1, 0, 0), (0.0707, 0.0707, 0), (0, 0.1, 0), resolution=0.02)
    print(f'circular_arc_path: {len(arc)} points, first={arc[0]}, last={arc[-1]}')