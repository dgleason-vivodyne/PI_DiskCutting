import csv
import json
import math
import os
import tkinter as tk
from tkinter import filedialog

import ezdxf
from ezdxf.path import make_path
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import CheckButtons
from scipy.spatial import KDTree

# Cut paths: only entities on layers that are **on** in the LAYER table. Layer "Startpoints" is
# excluded from cut geometry; CIRCLEs on Startpoints are used only as seam/start markers.
_EXCLUDED_LAYER_NAME = "Startpoints"

# Companion absolute-coordinate export: P(abs) = Origin + P(rel), pattern vertex P(rel) = full[i].
""" MICROSCOPE_CHUCK_CENTER_TO_OBJECTIVE_OFFSET_X_MM = 96.5632 """
""" MICROSCOPE_CHUCK_CENTER_TO_OBJECTIVE_OFFSET_Y_MM = 103.6625 """
MICROSCOPE_CHUCK_CENTER_TO_OBJECTIVE_OFFSET_X_MM = 0
MICROSCOPE_CHUCK_CENTER_TO_OBJECTIVE_OFFSET_Y_MM = 0

def _entity_layer_processed(doc, entity) -> bool:
    """True if entity's layer is on and not the Startpoints layer."""
    name = str(getattr(entity.dxf, "layer", "") or "").strip()
    if not name:
        return False
    if name.lower() == _EXCLUDED_LAYER_NAME.lower():
        return False
    if name not in doc.layers:
        return True
    return doc.layers.get(name).is_on()


def _is_startpoints_layer_name(entity) -> bool:
    return str(getattr(entity.dxf, "layer", "") or "").strip().lower() == _EXCLUDED_LAYER_NAME.lower()


def collect_startpoint_circle_centers(doc) -> np.ndarray:
    """CIRCLE entities on layer Startpoints: centers used to pick seam start on each cut contour."""
    centers = []
    for entity in doc.modelspace():
        if entity.dxftype() != "CIRCLE":
            continue
        if not _is_startpoints_layer_name(entity):
            continue
        centers.append((float(entity.dxf.center.x), float(entity.dxf.center.y)))
    if not centers:
        return np.empty((0, 2), dtype=float)
    return np.asarray(centers, dtype=float)


def _assign_startpoint_centers_to_contours(contours, centers: np.ndarray):
    """
    Greedy 1:1 match in contour order: each contour gets the closest unused circle center
    (minimum distance from center to any vertex of that contour).
    """
    if centers is None or len(centers) == 0:
        return [None] * len(contours)
    K = len(centers)
    used = [False] * K
    out = [None] * len(contours)
    for i, c in enumerate(contours):
        pts = np.asarray(c["points"], dtype=float)
        if len(pts) == 0:
            continue
        best_k = None
        best_d = np.inf
        for k in range(K):
            if used[k]:
                continue
            cen = centers[k]
            dmin = float(np.min(np.linalg.norm(pts - cen.reshape(1, 2), axis=1)))
            if dmin < best_d:
                best_d = dmin
                best_k = k
        if best_k is not None:
            used[best_k] = True
            out[i] = np.asarray(centers[best_k], dtype=float).copy()
    return out


def rotate_polyline_start_at_closest_vertex(pts: np.ndarray, center: np.ndarray) -> np.ndarray:
    """Closed contours only: cyclic shift so the vertex nearest ``center`` is first (preserves edge adjacency)."""
    p = np.asarray(pts, dtype=float)
    n = len(p)
    if n < 2:
        return p
    cen = np.asarray(center, dtype=float).reshape(2)
    d2 = np.sum((p - cen) ** 2, axis=1)
    k = int(np.argmin(d2))
    return np.vstack([p[k:], p[:k]])


def orient_open_polyline_toward_startpoint(pts: np.ndarray, center: np.ndarray) -> np.ndarray:
    """
    Open contours: cannot cyclic-shift without a bogus jump (last vertex would connect to first).
    Choose traversal direction so the **endpoint** nearer to ``center`` is the start of the path
    (reverse the vertex list if needed).
    """
    p = np.asarray(pts, dtype=float)
    if len(p) < 2:
        return p
    cen = np.asarray(center, dtype=float).reshape(2)
    d0 = float(np.sum((p[0] - cen) ** 2))
    d1 = float(np.sum((p[-1] - cen) ** 2))
    if d1 < d0:
        return np.flipud(p).copy()
    return p.copy()


def apply_startpoint_seam_rotations(doc, contours) -> bool:
    """Match Startpoints circles to contours; reorder seam start (closed = cyclic shift, open = maybe reverse)."""
    centers = collect_startpoint_circle_centers(doc)
    if len(centers) == 0:
        return False
    assigned = _assign_startpoint_centers_to_contours(contours, centers)
    did_any = False
    for c, cen in zip(contours, assigned):
        if cen is None:
            continue
        pts = np.asarray(c["points"], dtype=float)
        if len(pts) < 2:
            continue
        closed = bool(c.get("closed", False))
        if closed:
            new_pts = rotate_polyline_start_at_closest_vertex(pts, cen)
        else:
            new_pts = orient_open_polyline_toward_startpoint(pts, cen)
        c["points"] = new_pts
        c["startpoint_rotated"] = True
        did_any = True
    return did_any


# Interpolation functions
def interpolate_arc(center, radius, start_angle, end_angle, spacing):
    """Interpolates points along an arc with equal spacing."""
    start_angle = np.radians(start_angle)
    end_angle = np.radians(end_angle)

    if end_angle < start_angle:
        end_angle += 2 * np.pi

    arc_length = abs(radius * (end_angle - start_angle))
    num_points = max(2, int(arc_length / spacing))

    angles = np.linspace(start_angle, end_angle, num_points)
    arc_points = [(center[0] + np.cos(a) * radius, center[1] + np.sin(a) * radius) for a in angles]

    return arc_points


def interpolate_circle(center, radius, spacing):
    """Interpolates points along a full circle with equal spacing."""
    circumference = 2 * np.pi * radius
    num_points = max(2, int(circumference / spacing))

    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_points = [(center[0] + np.cos(a) * radius, center[1] + np.sin(a) * radius) for a in angles]

    return circle_points


def interpolate_ellipse(center, major_axis, minor_axis, start_param, end_param, spacing):
    """Interpolates points along an ellipse with equal spacing."""
    angles = np.linspace(start_param, end_param, int(2 * np.pi * max(major_axis, minor_axis) / (5 * spacing)))
    ellipse_points = [(center[0] + np.cos(a) * major_axis, center[1] + np.sin(a) * minor_axis) for a in angles]
    return ellipse_points


def interpolate_entity_xy(entity, spacing):
    """Sample one LINE/ARC/CIRCLE/ELLIPSE to a polyline in WCS (mm)."""
    dt = entity.dxftype()
    if dt == "LINE":
        start_point = np.array((entity.dxf.start.x, entity.dxf.start.y), dtype=float)
        end_point = np.array((entity.dxf.end.x, entity.dxf.end.y), dtype=float)
        dist = np.linalg.norm(end_point - start_point)
        n = max(2, int(dist / spacing))
        return np.linspace(start_point, end_point, n)
    if dt == "ARC":
        center = (entity.dxf.center.x, entity.dxf.center.y)
        arc_pts = interpolate_arc(center, entity.dxf.radius, entity.dxf.start_angle, entity.dxf.end_angle, spacing)
        return np.array(arc_pts, dtype=float)
    if dt == "CIRCLE":
        center = (entity.dxf.center.x, entity.dxf.center.y)
        circ_pts = interpolate_circle(center, entity.dxf.radius, spacing)
        return np.array(circ_pts, dtype=float)
    if dt == "ELLIPSE":
        center = (entity.dxf.center.x, entity.dxf.center.y)
        major_axis = entity.dxf.major_axis.magnitude
        minor_axis = major_axis * entity.dxf.ratio
        ellipse_points = interpolate_ellipse(
            center, major_axis, minor_axis, entity.dxf.start_param, entity.dxf.end_param, spacing
        )
        rotated = []
        for (x, y) in ellipse_points:
            x_shifted, y_shifted = x - center[0], y - center[1]
            rotated.append((-y_shifted + center[0], x_shifted + center[1]))
        return np.array(rotated, dtype=float)
    return None


def _infer_chain_closed(points, spacing, gap_threshold, single_circle, single_full_ellipse):
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        return False
    if single_circle:
        return True
    if single_full_ellipse:
        return True
    d_close = float(np.linalg.norm(p[0] - p[-1]))
    return d_close <= max(spacing * 3.0, gap_threshold * 0.5)


def _chord_dense_samples(p0: np.ndarray, p1: np.ndarray, spacing: float) -> np.ndarray:
    """Straight samples from ``p0`` to ``p1`` at ``spacing`` (exclusive of ``p0``, inclusive of ``p1``)."""
    p0 = np.asarray(p0, dtype=float).reshape(2)
    p1 = np.asarray(p1, dtype=float).reshape(2)
    d = float(np.linalg.norm(p1 - p0))
    if d <= 1e-12:
        return np.empty((0, 2))
    n_steps = max(1, int(np.ceil(d / float(spacing))))
    t = np.linspace(0.0, 1.0, n_steps + 1)
    xy = (1.0 - t)[:, None] * p0 + t[:, None] * p1
    return xy[1:]


def _orient_segment_to_follow_chain(raw: np.ndarray, prev_tail: np.ndarray) -> np.ndarray:
    """
    Prefer the sample direction whose **start** is closer to ``prev_tail`` (the last chain point).
    DXF entity order is often draw order, not always head-to-tail along the cut; reversing removes
    bogus long hops that look like wrong NN order in previews.
    """
    r = np.asarray(raw, dtype=float)
    if len(r) < 2:
        return r.copy()
    d_fwd = float(np.linalg.norm(r[0] - prev_tail))
    d_rev = float(np.linalg.norm(r[-1] - prev_tail))
    if d_rev < d_fwd - 1e-9:
        return np.flipud(r)
    return r.copy()


def generate_contours_from_dxf(
    dxf_path: str,
    spacing: float,
    gap_multiplier: float = 5.0,
    min_stitch_gap_mm: float = 1.0,
):
    """
    Entity-order chains: merge LINE/ARC/CIRCLE/ELLIPSE on allowed layers until gap > threshold.
    Each new entity is oriented (forward vs reversed) so its start matches the previous tail when
    possible. Returns a list of dicts: {"points": (N,2) float array}.

    ``min_stitch_gap_mm`` (default 1 mm): CAD endpoints often sit ~0.5–0.7 mm apart on corners; a
    threshold of only ``gap_multiplier * spacing`` would split outlines and leave long chord hops
    with no intermediate samples. The effective stitch limit is ``max(gap_multiplier * spacing,
    min_stitch_gap_mm)``. When two segments join with a non-zero gap, a straight chord is densified
    to ``spacing`` so previews and PVT steps stay sequential.
    """
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    gap_threshold = max(float(gap_multiplier) * float(spacing), float(min_stitch_gap_mm))
    stitch_eps = max(float(spacing) * 0.25, 1e-9)

    chains = []
    current = None
    chain_entities = []

    for entity in msp:
        dt = entity.dxftype()
        if dt not in ("LINE", "ARC", "CIRCLE", "ELLIPSE"):
            continue
        if not _entity_layer_processed(doc, entity):
            # Startpoints (seam markers) are not cut geometry; skipping them must NOT break the
            # current chain — otherwise a marker circle between bean segments splits one outline
            # into multiple contours and the flat path shows sub-mm "teleports" at vstack joins.
            if _is_startpoints_layer_name(entity):
                continue
            if current is not None and len(current) > 0:
                chains.append((current, chain_entities))
                current = None
                chain_entities = []
            continue
        raw = interpolate_entity_xy(entity, spacing)
        if raw is None or len(raw) == 0:
            continue

        if current is None:
            current = np.asarray(raw, dtype=float).copy()
            chain_entities = [entity]
            continue

        prev_tail = current[-1]
        seg = _orient_segment_to_follow_chain(raw, prev_tail)
        d_joint = float(np.linalg.norm(seg[0] - prev_tail))

        if d_joint > gap_threshold:
            chains.append((current, chain_entities))
            current = np.asarray(raw, dtype=float).copy()
            chain_entities = [entity]
            continue

        if d_joint < stitch_eps:
            add = seg[1:] if len(seg) > 1 else np.empty((0, 2))
        else:
            chord = _chord_dense_samples(prev_tail, seg[0], spacing)
            if len(chord) > 0 and len(seg) > 1:
                add = np.vstack([chord, seg[1:]])
            elif len(chord) > 0:
                add = chord
            else:
                add = seg

        if len(add) > 0:
            current = np.vstack([current, add])
        chain_entities.append(entity)

    if current is not None and len(current) > 0:
        chains.append((current, chain_entities))

    contours = []
    for pts, ents in chains:
        types = [e.dxftype() for e in ents]
        single_circle = len(ents) == 1 and ents[0].dxftype() == "CIRCLE"
        single_full_ellipse = False
        if len(ents) == 1 and ents[0].dxftype() == "ELLIPSE":
            span = abs(ents[0].dxf.end_param - ents[0].dxf.start_param)
            single_full_ellipse = span >= 2 * math.pi * 0.95
        closed = _infer_chain_closed(pts, spacing, gap_threshold, single_circle, single_full_ellipse)
        contours.append({"points": pts.copy(), "closed": closed, "dxftype": "+".join(types) if len(types) <= 3 else f"CHAIN[{len(types)}]"})

    return doc, contours


def generate_points_from_dxf(dxf_file, spacing):
    """
    Extract points from DXF: entity-order contours on allowed layers.

    Vertex order along each contour follows stitched LINE/ARC/CIRCLE/ELLIPSE samples; when
    chaining entities, each segment is **reversed if needed** so its start matches the previous
    tail (CAD file order is not always head-to-tail). Do **not** run ``optimize_path`` on those points:
    angle-sort + nearest-neighbor reorders dense samples on a closed curve and produces long
    chords that zigzag across the interior (looks like non-sequential hops in a preview).

    Startpoints: **closed** contours get a **cyclic shift** so the vertex nearest the marker is
    first (still one continuous loop). **Open** contours only **reverse** if needed so the nearer
    **endpoint** is first—cyclic shifts are not used on open polylines (they would add an
    artificial wrap jump mid-contour). Contours are concatenated in contour order (large moves
    between separate contours remain).

    Returns ``(flat_points, contour_chunks)`` where ``contour_chunks`` is a list of ``(N,2)``
    arrays used for laser on/off insertion in the CSV exporter.

    ``dxf_file`` must be the path to the DXF on disk (e.g. from a file dialog when run as a script).
    """
    # dxf_file = "C:/Users/DaveGleason/Desktop/FILETEST/SingleChipUpperLung.dxf"  # legacy hardcoded override
    doc, contours = generate_contours_from_dxf(dxf_file, spacing)
    apply_startpoint_seam_rotations(doc, contours)
    if not contours:
        return np.empty((0, 2)), []
    chunks = [np.asarray(c["points"], dtype=float).copy() for c in contours]
    flat = np.vstack(chunks)
    return flat, chunks


def dedupe_consecutive_points(points, eps_mm):
    """Drop consecutive duplicate positions only (keeps overlap retraces that revisit earlier XY)."""
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts)):
        if float(np.linalg.norm(pts[i] - out[-1])) >= eps_mm:
            out.append(pts[i])
    return np.array(out, dtype=float)


def flatten_contours_with_per_contour_overlap(
    contour_chunks: list,
    overlap_count: int,
    dedupe_eps_mm: float = 0.001,
) -> np.ndarray:
    """
    Concatenate contours in order. When ``overlap_count`` > 0, each contour is extended by
    appending its own first ``N`` vertices again (``N = min(overlap_count, len(contour))``) so the
    seam can be re-cut before laser-off / travel to the next shape.
    """
    overlap_n = max(0, int(overlap_count))
    chunks_d = [dedupe_consecutive_points(np.asarray(c, dtype=float), dedupe_eps_mm) for c in contour_chunks]
    chunks_d = [c for c in chunks_d if len(c) >= 1]
    parts = []
    for cc in chunks_d:
        if overlap_n > 0:
            n_take = min(overlap_n, len(cc))
            parts.append(np.vstack([cc, cc[:n_take]]))
        else:
            parts.append(cc)
    if not parts:
        return np.empty((0, 2))
    return np.vstack(parts)


# Function to compute time based on max velocity and max acceleration
def calculate_time_to_move(distance, max_velocity, max_acceleration):
    """Calculate the time to move a given distance considering max velocity and max acceleration."""
    if distance == 0:  # Avoid division by zero in case of zero distance
        return 0

    # Time to accelerate to max velocity
    time_to_max_velocity = max_velocity / max_acceleration

    # Distance covered during acceleration (using s = (1/2) * a * t^2)
    distance_accel = 0.5 * max_acceleration * (time_to_max_velocity ** 2)

    if distance <= 2 * distance_accel:  # If the distance is less than twice the acceleration distance

        # Calculate time using kinematic equation: t = sqrt(2 * distance / acceleration)
        time_to_move = np.sqrt(2 * distance / max_acceleration)
    else:
        # Time to accelerate, then constant speed, then decelerate
        distance_at_max_velocity = distance - 2 * distance_accel
        time_at_max_velocity = distance_at_max_velocity / max_velocity
        time_to_move = 2 * time_to_max_velocity + time_at_max_velocity

    return time_to_move


# Function to compute cumulative time and velocity for all points
def compute_relative_time_and_velocity(points, max_velocity, max_acceleration):
    """Compute cumulative time and velocity for each point ensuring smooth velocity transitions."""
    relative_times = [0]
    horizontal_velocities = [0]  # Start with zero velocity for the first point
    vertical_velocities = [0]    # Start with zero velocity for the first point

    # Ensure the first point starts with (0,0) velocity
    for i in range(1, len(points)):
        p1, p2 = np.array(points[i - 1]), np.array(points[i])
        dist = float(np.linalg.norm(p2 - p1))

        time_needed = calculate_time_to_move(dist, max_velocity, max_acceleration)

        # Zero-length step: calculate_time_to_move returns 0; avoid divide-by-zero / invalid velocity
        if dist <= 1e-15 or time_needed <= 1e-15:
            horizontal_velocity = 0.0
            vertical_velocity = 0.0
        else:
            horizontal_velocity = (p2[0] - p1[0]) / time_needed
            vertical_velocity = (p2[1] - p1[1]) / time_needed

        # Append time and velocity for the current step
        relative_times.append(time_needed) #time in msec
        horizontal_velocities.append(horizontal_velocity)
        vertical_velocities.append(vertical_velocity)

    return relative_times, horizontal_velocities, vertical_velocities


# Function to compute the path traversal in counter-clockwise direction
def optimize_path(points):
    """Reorders points using a Nearest Neighbor heuristic for optimal path traversal."""

    # Compute centroid (center of mass)
    centroid = np.mean(points, axis=0)

    # Compute angles from the centroid to the points
    angles = np.arctan2(points[:, 1] - centroid[1], points[:, 0] - centroid[0])

    # Sort points based on the angle to get a counter-clockwise order
    sorted_points = points[np.argsort(angles)]

    # Nearest Neighbor optimization on the counter-clockwise sorted path
    tree = KDTree(sorted_points)
    num_points = len(sorted_points)
    visited = np.zeros(num_points, dtype=bool)
    optimized_order = []

    # Start at the first point
    current_idx = 0
    for _ in range(num_points):
        optimized_order.append(current_idx)
        visited[current_idx] = True

        # Find the nearest unvisited point
        remaining_points = np.where(~visited)[0]
        if len(remaining_points) == 0:
            break

        distances, indices = tree.query(sorted_points[current_idx], k=len(sorted_points))
        for idx in indices:
            if not visited[idx]:
                current_idx = idx
                break

    return sorted_points[optimized_order]


_LASER_ON_ROWS = (
    ("set laser voltage to", "1", "", "", ""),
    ("turn laser on", "", "", "", ""),
)
_LASER_OFF_ROWS = (
    ("set laser voltage to", "0", "", "", ""),
    ("turn laser off", "", "", "", ""),
)


def _segment_time_velocity(p0, p1, max_velocity, max_acceleration):
    """Returns (dt, vx, vy) for motion from p0 to p1."""
    p0 = np.asarray(p0, dtype=float).reshape(2)
    p1 = np.asarray(p1, dtype=float).reshape(2)
    dist = float(np.linalg.norm(p1 - p0))
    dt = calculate_time_to_move(dist, max_velocity, max_acceleration)
    if dist <= 1e-15 or dt <= 1e-15:
        return 0.0, 0.0, 0.0
    return dt, float((p1[0] - p0[0]) / dt), float((p1[1] - p0[1]) / dt)


def _unit2d(v):
    v = np.asarray(v, dtype=float).reshape(2)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.array([1.0, 0.0], dtype=float)
    return (v / n).astype(float)


def _lead_length_mm(max_velocity, max_acceleration):
    """Distance (mm) to accelerate from 0 to ``max_velocity`` at constant ``max_acceleration``."""
    if max_acceleration <= 1e-12:
        return 0.0
    return float(max_velocity * max_velocity) / (2.0 * max_acceleration)


def _sample_polyline_straight(p_from, p_to, n_seg):
    """``n_seg`` >= 1 segments along p_from -> p_to; returns (n_seg + 1, 2) vertices inclusive."""
    p_from = np.asarray(p_from, dtype=float).reshape(2)
    p_to = np.asarray(p_to, dtype=float).reshape(2)
    n_seg = max(1, int(n_seg))
    t = np.linspace(0.0, 1.0, n_seg + 1, dtype=float)
    return (1.0 - t)[:, None] * p_from + t[:, None] * p_to


def build_export_segments_with_leads(
    full, starts, chunks_d, overlap_n, spacing, max_velocity, max_acceleration
):
    """
    Build ordered motion segments: per contour (lead-in -> cut polyline with overlap ->
    lead-out), then straight travel to the next contour's lead-in start.

    Returns ``(segs, replay, schedule)`` where ``segs`` is a list of (p0, p1), ``replay`` maps
    destination segment index -> source index for overlap retrace, and ``schedule`` is a list of
    (kind, lo, hi) with hi exclusive segment indices into ``segs``.
    """
    L = _lead_length_mm(max_velocity, max_acceleration)
    n_lead = max(1, int(np.ceil(L / spacing))) if spacing > 1e-12 else 1

    segs = []
    replay = {}
    schedule = []
    k_count = len(chunks_d)

    for k in range(k_count):
        V = full[starts[k] : starts[k + 1]]
        if len(V) < 1:
            continue

        if len(V) >= 2:
            dir_in = _unit2d(V[1] - V[0])
            dir_out = _unit2d(V[-1] - V[-2])
        else:
            u = np.array([1.0, 0.0], dtype=float)
            dir_in = u
            dir_out = u

        p_start = V[0]
        A = p_start - dir_in * L
        pts_in = _sample_polyline_straight(A, p_start, n_lead)
        li_lo = len(segs)
        for i in range(n_lead):
            segs.append((pts_in[i].copy(), pts_in[i + 1].copy()))
        li_hi = len(segs)
        schedule.append(("lead_in", li_lo, li_hi))

        if len(V) >= 2:
            len_c = len(chunks_d[k])
            nt = min(overlap_n, len_c)
            cut_lo = len(segs)
            for i in range(len(V) - 1):
                segs.append((V[i].copy(), V[i + 1].copy()))
                if nt > 1 and len_c <= i <= len_c + nt - 2:
                    dst = len(segs) - 1
                    src = cut_lo + (i - len_c)
                    replay[dst] = src
            cut_hi = len(segs)
            schedule.append(("cut", cut_lo, cut_hi))
        else:
            cut_lo = len(segs)
            schedule.append(("cut", cut_lo, cut_lo))

        p_end = V[-1]
        E = p_end + dir_out * L
        pts_out = _sample_polyline_straight(p_end, E, n_lead)
        lo_lo = len(segs)
        for i in range(n_lead):
            segs.append((pts_out[i].copy(), pts_out[i + 1].copy()))
        lo_hi = len(segs)
        schedule.append(("lead_out", lo_lo, lo_hi))

        if k < k_count - 1:
            W = full[starts[k + 1] : starts[k + 2]]
            if len(W) < 1:
                continue
            if len(W) >= 2:
                dn = _unit2d(W[1] - W[0])
            else:
                dn = np.array([1.0, 0.0], dtype=float)
            A_next = W[0] - dn * L
            dist = float(np.linalg.norm(A_next - E))
            n_tr = max(1, int(np.ceil(dist / spacing))) if spacing > 1e-12 else 1
            pts_tr = _sample_polyline_straight(E, A_next, n_tr)
            tr_lo = len(segs)
            for i in range(n_tr):
                segs.append((pts_tr[i].copy(), pts_tr[i + 1].copy()))
            tr_hi = len(segs)
            schedule.append(("travel", tr_lo, tr_hi))

    return segs, replay, schedule


def generate_csv_from_points(
    points,
    output_filename,
    max_velocity,
    max_acceleration,
    contour_chunks=None,
    overlap_count: int = 0,
    spacing: float = 0.01,
):
    """
    Writes the main PVT CSV with laser I/O only around **cut** motion (not during lead-in/out or
    travel). Lead-in/out length (mm) is ``v_max^2 / (2 a)``; each lead is sampled into
    ``ceil(lead_mm / spacing)`` straight segments along entry/exit tangent. Inter-contour **travel**
    is a straight chord from end of lead-out to start of the next contour's lead-in, sampled by
    ``spacing``. ``0,0,0,0,0`` sync starts the file; relative file ends with laser-off rows.

    Writes a sibling ``*_absolute.csv`` with the same headers and motion order, **no** laser text
    rows, ``0,0,0,0,0`` at start and end; columns 2 and 4 are absolute endpoints ``Origin + p1``.

    Writes a sibling ``<stem>.pvt.meta.json`` (same stem as the main CSV, e.g. ``foo_pvt.csv`` →
    ``foo_pvt.pvt.meta.json``) with ``origin_xy_mm`` = DXF WCS (mm) at the **first export segment
    start** (lead-in start). Delta PVT omits this rigid shift; DMS applies it before ``PointRelative``
    when the meta file is present.

    Overlap retrace (cut polyline only): repeated edges reuse the same PVT row as the first pass.

    ``spacing``: DXF point spacing (mm), used to set lead/travel segment counts.
    """
    fuzz = 0.001
    overlap_n = max(0, int(overlap_count))

    if contour_chunks is not None and len(contour_chunks) > 0:
        chunks_d = [dedupe_consecutive_points(np.asarray(c, dtype=float), fuzz) for c in contour_chunks]
        chunks_d = [c for c in chunks_d if len(c) >= 1]
        if not chunks_d:
            raise ValueError("No points to export.")
        full = flatten_contours_with_per_contour_overlap(contour_chunks, overlap_n, fuzz)
        if len(full) < 1:
            raise ValueError("No points to export.")
        starts = [0]
        for c in chunks_d:
            ext_len = len(c) + (min(overlap_n, len(c)) if overlap_n > 0 else 0)
            starts.append(starts[-1] + ext_len)
    else:
        cc = dedupe_consecutive_points(np.asarray(points, dtype=float), fuzz)
        if len(cc) < 1:
            raise ValueError("No points to export.")
        if overlap_n > 0:
            n_take = min(overlap_n, len(cc))
            full = np.vstack([cc, cc[:n_take]])
        else:
            full = cc
        starts = [0, len(full)]
        chunks_d = [cc]

    # Relative CSV: columns 2 and 4 are segment *deltas* (mm) for Zaber PointRelative / DMS.
    headers_rel = [
        "Time (s)",
        "Horizontal delta (mm)",
        "Horizontal velocity (mm/s)",
        "Vertical delta (mm)",
        "Vertical velocity (mm/s)",
    ]
    headers_abs = [
        "Time (s)",
        "Horizontal position (mm)",
        "Horizontal velocity (mm/s)",
        "Vertical position (mm)",
        "Vertical velocity (mm/s)",
    ]
    ox = float(MICROSCOPE_CHUCK_CENTER_TO_OBJECTIVE_OFFSET_X_MM)
    oy = float(MICROSCOPE_CHUCK_CENTER_TO_OBJECTIVE_OFFSET_Y_MM)

    if output_filename.lower().endswith(".csv"):
        abs_output_filename = output_filename[:-4] + "_absolute.csv"
    else:
        abs_output_filename = output_filename + "_absolute.csv"

    segs, replay, schedule = build_export_segments_with_leads(
        full, starts, chunks_d, overlap_n, spacing, max_velocity, max_acceleration
    )
    if not segs:
        raise ValueError("No export segments (empty path).")

    rel_rows = []
    for j in range(len(segs)):
        p0, p1 = segs[j]
        dt, vx, vy = _segment_time_velocity(p0, p1, max_velocity, max_acceleration)
        dx = float(p1[0] - p0[0])
        dy = float(p1[1] - p0[1])
        rel_rows.append((dt, dx, vx, dy, vy))

    for dst, src in replay.items():
        if 0 <= dst < len(rel_rows) and 0 <= src < len(rel_rows):
            rel_rows[dst] = rel_rows[src]

    def _emit_motion_rows(wr, wabs, j0, j1_exclusive):
        for j in range(j0, j1_exclusive):
            dt, dx, vx, dy, vy = rel_rows[j]
            _, p1 = segs[j]
            wr.writerow([dt, dx, vx, dy, vy])
            wabs.writerow([dt, ox + float(p1[0]), vx, oy + float(p1[1]), vy])

    with open(output_filename, "w", newline="", encoding="utf-8") as f, open(
        abs_output_filename, "w", newline="", encoding="utf-8"
    ) as fa:
        w = csv.writer(f)
        wa = csv.writer(fa)
        w.writerow(headers_rel)
        wa.writerow(headers_abs)
        w.writerow([0, 0, 0, 0, 0])
        wa.writerow([0, 0, 0, 0, 0])

        for kind, lo, hi in schedule:
            if kind == "cut" and hi > lo:
                for row in _LASER_ON_ROWS:
                    w.writerow(row)
            _emit_motion_rows(w, wa, lo, hi)
            if kind == "cut" and hi > lo:
                for row in _LASER_OFF_ROWS:
                    w.writerow(row)

        for row in _LASER_OFF_ROWS:
            w.writerow(row)

        wa.writerow([0, 0, 0, 0, 0])

    p0 = np.asarray(segs[0][0], dtype=float).reshape(2)
    meta_stem = output_filename[:-4] if output_filename.lower().endswith(".csv") else output_filename
    meta_filename = meta_stem + ".pvt.meta.json"
    meta_payload = {
        "schema_version": 1,
        "origin_xy_mm": [float(p0[0]), float(p0[1])],
        "description": (
            "origin_xy_mm is DXF WCS (mm) at the first PVT segment start (lead-in start). "
            "Relative CSV columns 2 and 4 are deltas; this origin is omitted there and applied by DMS when present."
        ),
        "max_velocity_mm_s": float(max_velocity),
        "max_acceleration_mm_s2": float(max_acceleration),
        "spacing_mm": float(spacing),
        "relative_csv_columns": ["time_s", "delta_x_mm", "vx_mm_s", "delta_y_mm", "vy_mm_s"],
        "source_csv": os.path.basename(output_filename),
    }
    try:
        with open(meta_filename, "w", encoding="utf-8") as mf:
            json.dump(meta_payload, mf, indent=2)
            mf.write("\n")
    except OSError as ex:
        print(f"Warning: could not write PVT meta JSON {meta_filename}: {ex}")

    print(f"CSV file saved to {output_filename}")
    print(f"Absolute-coordinate CSV saved to {abs_output_filename}")
    print(f"PVT CAD origin meta saved to {meta_filename}")


def load_pvt_csv_segment_deltas(csv_path):
    """Parse delta PVT CSV motion rows: time, delta_x, vx, delta_y, vy (columns 0–4) — return (N, 2) dx, dy."""
    deltas = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            try:
                float(row[0])
                dx = float(row[1])
                dy = float(row[3])
            except ValueError:
                continue
            deltas.append((dx, dy))
    return np.asarray(deltas, dtype=float)


def deltas_to_polyline(start_xy, deltas):
    """start_xy (2,), deltas (N, 2) -> (N + 1, 2) polyline including start (cumulative CSV path)."""
    start_xy = np.asarray(start_xy, dtype=float).reshape(2)
    if len(deltas) == 0:
        return start_xy.reshape(1, 2)
    cum = np.cumsum(deltas, axis=0)
    return np.vstack([start_xy, start_xy + cum])


def _pvt_csv_replay_start_xy(points, max_velocity, max_acceleration):
    """Lead-in start before ``points[0]``; same geometry as ``build_export_segments_with_leads``."""
    V = np.asarray(points, dtype=float)
    if len(V) < 1:
        return np.array([0.0, 0.0], dtype=float)
    if len(V) >= 2:
        dir_in = _unit2d(V[1] - V[0])
    else:
        dir_in = np.array([1.0, 0.0], dtype=float)
    L = _lead_length_mm(max_velocity, max_acceleration)
    return V[0] - dir_in * L


# Function to plot optimized path with velocity vectors
def plot_points_with_velocity_vectors(
    points,
    horizontal_velocities,
    vertical_velocities,
    offset_distance=0.001,
    csv_path=None,
    max_velocity=None,
    max_acceleration=None,
):
    """
    Plot DXF-derived polyline with optional velocity quivers and point indices.

    If ``csv_path`` is set and the file exists, overlay the path from cumulative
    CSV dx/dy. With lead-in/out, the replay must start at the exporter's first
    motion point: pass the same ``max_velocity`` and ``max_acceleration`` used
    for ``generate_csv_from_points`` so lead length matches the CSV. If either
    is omitted, the overlay is rooted at ``points[0]`` (legacy; misaligned when
    leads are present).

    Checkboxes toggle DXF path, CSV replay, quiver, and point labels.
    """
    centroid = np.mean(points, axis=0)

    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(right=0.78)

    (ln_dxf,) = ax.plot(
        points[:, 0],
        points[:, 1],
        color="blue",
        marker="o",
        markersize=3,
        linestyle="-",
        label="DXF samples",
        zorder=2,
    )

    quiv = ax.quiver(
        points[:, 0],
        points[:, 1],
        horizontal_velocities,
        vertical_velocities,
        angles="xy",
        scale_units="xy",
        scale=0.1,
        color="red",
        label="Velocity",
        zorder=3,
    )

    label_objs = []
    for i, point in enumerate(points):
        direction = np.array([point[0] - centroid[0], point[1] - centroid[1]])
        direction_normalized = direction / np.linalg.norm(direction)
        offset = direction_normalized * offset_distance
        label_x = point[0] + offset[0]
        label_y = point[1] + offset[1]
        t = ax.text(label_x, label_y, str(i), fontsize=7, color="black", ha="center", va="center", zorder=4)
        label_objs.append(t)

    ln_csv = None
    if csv_path and os.path.isfile(csv_path):
        deltas = load_pvt_csv_segment_deltas(csv_path)
        if max_velocity is not None and max_acceleration is not None:
            csv_start = _pvt_csv_replay_start_xy(points, max_velocity, max_acceleration)
        else:
            csv_start = np.asarray(points[0], dtype=float).reshape(2)
        csv_xy = deltas_to_polyline(csv_start, deltas)
        (ln_csv,) = ax.plot(
            csv_xy[:, 0],
            csv_xy[:, 1],
            color="tab:orange",
            marker="x",
            markersize=3,
            linestyle="--",
            linewidth=1.2,
            label="CSV",
            zorder=5,
        )

    ax.set_xlabel("X Coordinate (mm)")
    ax.set_ylabel("Y Coordinate (mm)")
    ax.set_title("Path: DXF vs CSV replay (use checkboxes)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left", fontsize=8)

    rax = plt.axes((0.80, 0.35, 0.18, 0.22))
    check_labels = ["DXF path", "Velocities", "Point labels"]
    actives = [True, True, True]
    if ln_csv is not None:
        check_labels.insert(1, "CSV replay")
        actives.insert(1, True)
    chk = CheckButtons(rax, check_labels, actives)

    def on_check_click(_label):
        status = chk.get_status()
        i = 0
        ln_dxf.set_visible(status[i])
        i += 1
        if ln_csv is not None:
            ln_csv.set_visible(status[i])
            i += 1
        quiv.set_visible(status[i])
        i += 1
        vis_labels = status[i]
        for t in label_objs:
            t.set_visible(vis_labels)
        fig.canvas.draw_idle()

    chk.on_clicked(on_check_click)

    plt.show()


def prompt_overlap_point_count():
    """Ask how many start vertices to repeat per contour before laser-off. Returns a non-negative int (0 = none)."""
    print()
    print("--- Overlap ---")
    print(
        "For each contour (each closed shape), append that contour's first N vertices again\n"
        "before laser-off / travel to the next shape — to re-cut the seam near the start.\n"
    )
    while True:
        raw = input("How many start points to repeat? [0 = none]: ").strip()
        if raw == "":
            print("Overlap: 0 (none).\n")
            return 0
        try:
            n = int(raw)
            if n < 0:
                print("Enter a non-negative integer.")
                continue
            print(f"Overlap: {n} start vertex/vertices replayed per contour before each laser-off (except if N=0).\n")
            return n
        except ValueError:
            print("Enter an integer, e.g. 0 or 10.")


def prompt_dxf_path():
    """Native file dialog to pick a DXF. Returns path string, or empty string if cancelled."""
    root = tk.Tk()
    root.withdraw()
    try:
        root.attributes("-topmost", True)
    except tk.TclError:
        pass
    path = filedialog.askopenfilename(
        title="Select DXF file to convert",
        filetypes=[("DXF files", "*.dxf"), ("All files", "*.*")],
    )
    root.destroy()
    return path


if __name__ == '__main__':
    dxf_file = prompt_dxf_path()
    if not dxf_file:
        print("No DXF file selected. Exiting.")
        raise SystemExit(0)
    print(f"Using DXF: {dxf_file}")

    spacing = 0.01  # Spacing between points (in mm)
    max_velocity = 100  # Max velocity (in mm/sec)
    max_acceleration = 5000  # Max acceleration (in mm/s^2)

    # DXF chain order + Startpoints seam rotation (no optimize_path — it scrambles closed curves)
    optimized_points, contour_chunks = generate_points_from_dxf(dxf_file, spacing)

    overlap_count = prompt_overlap_point_count()
    n_overlap = int(overlap_count) if overlap_count > 0 else 0
    optimized_points = flatten_contours_with_per_contour_overlap(contour_chunks, n_overlap)

    # Compute time and velocity for optimized path (plot)
    times, horizontal_velocities, vertical_velocities = compute_relative_time_and_velocity(optimized_points, max_velocity, max_acceleration)

    output_csv = dxf_file.replace(".dxf", "_pvt.csv")
    generate_csv_from_points(
        optimized_points,
        output_csv,
        max_velocity,
        max_acceleration,
        contour_chunks=contour_chunks,
        overlap_count=n_overlap,
        spacing=spacing,
    )

    plot_points_with_velocity_vectors(
        optimized_points,
        horizontal_velocities,
        vertical_velocities,
        csv_path=output_csv,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
    )
