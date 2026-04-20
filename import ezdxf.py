"""
DXF -> interpolated contours -> contour-order + bridge + overlap -> PVT CSV.
PI_DiskCutting project. See CSV_DMS_README.txt for PVT column layout and DMS consumption.
"""
import os
import ezdxf
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import math


def resolve_dxf_path(dxf_file):
    """Resolve DXF path: absolute paths unchanged; relative paths tried from CWD then this script's directory."""
    if not dxf_file or not str(dxf_file).strip():
        raise FileNotFoundError("DXF path is empty.")
    path = os.path.normpath(os.path.expanduser(str(dxf_file).strip()))
    if os.path.isabs(path) and os.path.isfile(path):
        return path
    if os.path.isfile(path):
        return os.path.abspath(path)
    here = os.path.dirname(os.path.abspath(__file__))
    alt = os.path.join(here, path)
    if os.path.isfile(alt):
        return alt
    raise FileNotFoundError(f"DXF not found: {dxf_file!r} (tried CWD and {alt})")

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

def _contour_dict(points, closed, dxftype, entity_index, source="entity"):
    return {
        "points": points,
        "closed": closed,
        "dxftype": dxftype,
        "entity_index": entity_index,
        "source": source,
    }


def interpolate_entity_xy(entity, spacing):
    """
    Sample one supported entity to a polyline in WCS (mm). Order follows entity geometry.
    Returns (N,2) float array or None if unsupported / empty.
    """
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
    """Decide if a merged point chain is a closed loop for overlap / traversal."""
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        return False
    if single_circle:
        return True
    if single_full_ellipse:
        return True
    d_close = float(np.linalg.norm(p[0] - p[-1]))
    # Closed bean: seam meets; allow modest sampling gap
    return d_close <= max(spacing * 3.0, gap_threshold * 0.5)


def generate_contours_from_dxf(dxf_file, spacing, gap_multiplier=5.0):
    """
    Walk modelspace in **entity order**. Many LINE/ARC pieces are un-joined segments of one
    logical shape: merge into one contour while the gap from the previous endpoint to this
    entity's start is **not** a large jump. A **new** contour starts when that gap exceeds
    ``gap_multiplier * spacing`` (e.g. 5 × 0.01 mm = 0.05 mm between separate shapes).

    Contour dict keys: points, closed, dxftype (label), entity_index (chain index), source.
    """
    path = resolve_dxf_path(dxf_file)
    doc = ezdxf.readfile(path)
    msp = doc.modelspace()
    gap_threshold = float(gap_multiplier) * float(spacing)
    stitch_eps = max(float(spacing) * 0.25, 1e-9)

    chains = []
    current = None
    chain_entities = []

    for entity in msp:
        dt = entity.dxftype()
        if dt not in ("LINE", "ARC", "CIRCLE", "ELLIPSE"):
            continue
        raw = interpolate_entity_xy(entity, spacing)
        if raw is None or len(raw) == 0:
            continue

        if current is None:
            current = raw.copy()
            chain_entities = [entity]
            continue

        d_joint = float(np.linalg.norm(raw[0] - current[-1]))
        # Weld: shared vertex between consecutive entities of the same logical shape
        if d_joint < stitch_eps:
            add = raw[1:] if len(raw) > 1 else np.empty((0, 2))
        else:
            add = raw

        if d_joint > gap_threshold:
            chains.append((current, chain_entities))
            current = raw.copy()
            chain_entities = [entity]
        else:
            if len(add) > 0:
                current = np.vstack([current, add])
            chain_entities.append(entity)

    if current is not None and len(current) > 0:
        chains.append((current, chain_entities))

    contours = []
    for idx, (pts, ents) in enumerate(chains):
        types = [e.dxftype() for e in ents]
        single_circle = len(ents) == 1 and ents[0].dxftype() == "CIRCLE"
        single_full_ellipse = False
        if len(ents) == 1 and ents[0].dxftype() == "ELLIPSE":
            span = abs(ents[0].dxf.end_param - ents[0].dxf.start_param)
            single_full_ellipse = span >= 2 * math.pi * 0.95
        closed = _infer_chain_closed(pts, spacing, gap_threshold, single_circle, single_full_ellipse)
        label = "+".join(types) if len(types) <= 3 else f"CHAIN[{len(types)}]"
        contours.append(_contour_dict(pts, closed, label, idx, source="entity_order"))

    return contours


def split_flat_polyline_by_gap(points, spacing, k_multiplier=5.0, absolute_min_mm=None):
    """
    Diagnostic: split when **consecutive** point distance exceeds threshold.
    Prefer `generate_contours_from_dxf` (entity-order merge) for normal DXF input.
    Do not use this on raw circle polylines: the closing chord can falsely exceed the threshold.
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return []
    abs_min = absolute_min_mm if absolute_min_mm is not None else 0.0
    threshold = max(spacing * k_multiplier, abs_min, spacing * 3.0)
    segments = []
    start = 0
    for i in range(len(pts) - 1):
        d = float(np.linalg.norm(pts[i + 1] - pts[i]))
        if d > threshold:
            chunk = pts[start : i + 1]
            if len(chunk) >= 1:
                seg_idx = len(segments)
                c = _infer_chain_closed(chunk, spacing, threshold, False, False)
                segments.append(_contour_dict(chunk.copy(), c, "GAP_SPLIT", seg_idx, source="gap_split"))
            start = i + 1
    chunk = pts[start:]
    if len(chunk) >= 1:
        seg_idx = len(segments)
        c = _infer_chain_closed(chunk, spacing, threshold, False, False)
        segments.append(_contour_dict(chunk.copy(), c, "GAP_SPLIT", seg_idx, source="gap_split"))
    return segments


def generate_points_from_dxf(dxf_file, spacing, gap_multiplier=5.0):
    """Flat array: all chains concatenated in modelspace order (for gap-split diagnostic)."""
    contours = generate_contours_from_dxf(dxf_file, spacing, gap_multiplier=gap_multiplier)
    if not contours:
        return np.empty((0, 2))
    return np.vstack([c["points"] for c in contours])

def contour_centroid(points):
    p = np.asarray(points, dtype=float)
    if len(p) == 0:
        return np.zeros(2)
    return np.mean(p, axis=0)

def order_contours_greedy_nn(contours, start_idx=0):
    """
    Order contour groups by greedy nearest-neighbor on centroids (contour-level TSP heuristic).
    """
    n = len(contours)
    if n <= 1:
        return list(contours)
    centroids = np.array([contour_centroid(c["points"]) for c in contours])
    visited = [False] * n
    order_idx = [start_idx % n]
    visited[order_idx[0]] = True
    current = order_idx[0]
    for _ in range(n - 1):
        best_j = -1
        best_d = np.inf
        for j in range(n):
            if visited[j]:
                continue
            d = np.sum((centroids[j] - centroids[current]) ** 2)
            if d < best_d:
                best_d = d
                best_j = j
        visited[best_j] = True
        order_idx.append(best_j)
        current = best_j
    return [contours[i] for i in order_idx]

def nearest_vertex_index(exit_pt, pts):
    p = np.asarray(pts, dtype=float)
    if len(p) == 0:
        return 0
    d2 = np.sum((p - np.asarray(exit_pt, dtype=float)) ** 2, axis=1)
    return int(np.argmin(d2))

def rotate_contour_to_entry(pts, closed, prev_exit):
    """
    Orient contour so cutting starts at the best entry from prev_exit.
    Closed: rotate so nearest vertex is first.
    Open: reverse so the nearer endpoint is first (cut along polyline direction).
    """
    p = np.asarray(pts, dtype=float)
    if len(p) == 0 or prev_exit is None:
        return p
    prev_exit = np.asarray(prev_exit, dtype=float)
    if closed:
        j = nearest_vertex_index(prev_exit, p)
        return np.vstack([p[j:], p[:j]])
    d0 = np.sum((p[0] - prev_exit) ** 2)
    d1 = np.sum((p[-1] - prev_exit) ** 2)
    if d1 < d0:
        return np.flipud(p).copy()
    return p

def apply_overlap_closed(pts, closed, overlap_count=0, overlap_fraction=0.0):
    """
    For closed contours, append the first N vertices again to overlap the laser path.
    overlap_count takes precedence if > 0; else overlap_fraction in (0,1] sets N = floor(fraction * len).
    """
    p = np.asarray(pts, dtype=float)
    if not closed or len(p) < 2:
        return p
    if overlap_count > 0:
        n_extra = min(overlap_count, len(p))
    elif overlap_fraction > 0:
        n_extra = max(1, int(len(p) * overlap_fraction))
        n_extra = min(n_extra, len(p))
    else:
        return p
    return np.vstack([p, p[:n_extra]])

def interpolate_line_2d(a, b, spacing):
    """Polyline from a to b with approximate spacing (at least 2 points)."""
    a = np.asarray(a, dtype=float).reshape(2)
    b = np.asarray(b, dtype=float).reshape(2)
    dist = np.linalg.norm(b - a)
    if dist < 1e-12:
        return np.vstack([a, b])
    n = max(2, int(dist / spacing))
    t = np.linspace(0.0, 1.0, n)
    xs = a[0] + t * (b[0] - a[0])
    ys = a[1] + t * (b[1] - a[1])
    return np.column_stack([xs, ys])


def travel_polyline_between(prev_exit, entry, spacing, dense=True):
    """
    Points for a rapid move from prev_exit to entry (between contours).

    dense=True: sample along the segment at ~`spacing` (many rows for long jumps — matches
      cut sampling density; smoother PVT).
    dense=False: only endpoints (2 points if distinct) — far fewer rows; motion planner still
      gets one straight segment (velocity profile may be harsher).
    """
    a = np.asarray(prev_exit, dtype=float).reshape(2)
    b = np.asarray(entry, dtype=float).reshape(2)
    if np.linalg.norm(b - a) < 1e-12:
        return np.empty((0, 2))
    if dense:
        return interpolate_line_2d(a, b, spacing)
    return np.vstack([a, b])


def dedupe_consecutive_points(points, eps_mm):
    """Drop consecutive rows at the same location (stitch duplicates, float noise)."""
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return pts
    out_p = [pts[0]]
    for i in range(1, len(pts)):
        if float(np.linalg.norm(pts[i] - out_p[-1])) >= eps_mm:
            out_p.append(pts[i])
    return np.array(out_p, dtype=float)


def bridge_junction_epsilon_mm(spacing_mm):
    """Tolerance for travel/cut stitches; keep CSV dedupe aligned with build_cutting_path_with_bridges."""
    return max(1e-6, float(spacing_mm) * 1e-4)


def build_cutting_path_with_bridges(
    contours_ordered,
    spacing,
    overlap_count=0,
    overlap_fraction=0.0,
    dense_travel=True,
):
    """
    Concatenate contours with travel segments between them.
    For each contour after the first: rotate so nearest entry to previous exit is first,
    insert interpolated travel (not cut), then emit cut points (with overlap on closed).
    Returns (points_Nx2, is_cut_bool_1d).

    Point count vs raw DXF interpolation: entity samples are unchanged. Extra rows come from
    (1) travel between contours — many if dense_travel (~distance/spacing per gap), or 2 per
    gap if dense_travel=False; (2) overlap on closed contours if enabled.
    """
    # Same-location stitches (travel end == next cut start) must merge; float noise handled too.
    junction_eps_mm = max(1e-6, float(spacing) * 1e-4)

    chunks_pts = []
    chunks_cut = []
    prev_exit = None

    for idx, c in enumerate(contours_ordered):
        pts = np.asarray(c["points"], dtype=float)
        closed = bool(c["closed"])
        if len(pts) == 0:
            continue

        if idx > 0:
            pts = rotate_contour_to_entry(pts, closed, prev_exit)
            travel = travel_polyline_between(prev_exit, pts[0], spacing, dense=dense_travel)
            if len(travel) > 0 and np.allclose(travel[0], prev_exit, atol=junction_eps_mm):
                travel = travel[1:]
            # Travel ends at pts[0]; cut_pts will start at pts[0] — drop duplicate endpoint on travel.
            if len(travel) > 0 and np.allclose(travel[-1], pts[0], atol=junction_eps_mm):
                travel = travel[:-1]
            if len(travel) > 0:
                chunks_pts.append(travel)
                chunks_cut.append(np.zeros(len(travel), dtype=bool))

        cut_pts = apply_overlap_closed(pts, closed, overlap_count, overlap_fraction)
        chunks_pts.append(cut_pts)
        chunks_cut.append(np.ones(len(cut_pts), dtype=bool))
        prev_exit = cut_pts[-1]

    if not chunks_pts:
        return np.empty((0, 2)), np.empty(0, dtype=bool)
    points = np.vstack(chunks_pts)
    is_cut = np.concatenate(chunks_cut)
    points, is_cut = dedupe_consecutive_points(points, is_cut, junction_eps_mm)
    return points, is_cut

def optimize_path(points):
    """
    Legacy: global reorder (angle sort + NN). Prefer build_cutting_path_with_bridges on contours
    for multi-contour jobs so closed loops and bridging are preserved.
    """
    if len(points) == 0:
        return points
    centroid = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - centroid[1], points[:, 0] - centroid[0])
    sorted_points = points[np.argsort(angles)]
    tree = KDTree(sorted_points)
    num_points = len(sorted_points)
    visited = np.zeros(num_points, dtype=bool)
    optimized_order = []
    current_idx = 0
    for _ in range(num_points):
        optimized_order.append(current_idx)
        visited[current_idx] = True
        remaining_points = np.where(~visited)[0]
        if len(remaining_points) == 0:
            break
        _, indices = tree.query(sorted_points[current_idx], k=len(sorted_points))
        for idx in indices:
            if not visited[idx]:
                current_idx = idx
                break
    return sorted_points[optimized_order]

def remove_close_points(points, fuzz):
    """Removes points that are within 'fuzz' distance of each other."""
    unique_points = []
    for i, point in enumerate(points):
        if not any(np.linalg.norm(point - p) < fuzz for p in unique_points):
            unique_points.append(point)
    return np.array(unique_points)

def calculate_time_to_move(distance, max_velocity, max_acceleration):
    """Calculate the time to move a given distance considering max velocity and max acceleration."""
    if distance == 0:
        return 0
    time_to_max_velocity = max_velocity / max_acceleration
    distance_accel = 0.5 * max_acceleration * (time_to_max_velocity ** 2)
    if distance <= 2 * distance_accel:
        time_to_move = np.sqrt(2 * distance / max_acceleration)
    else:
        distance_at_max_velocity = distance - 2 * distance_accel
        time_at_max_velocity = distance_at_max_velocity / max_velocity
        time_to_move = 2 * time_to_max_velocity + time_at_max_velocity
    return time_to_move

def compute_relative_time_and_velocity(points, max_velocity, max_acceleration):
    """Compute cumulative time and velocity for each point ensuring smooth velocity transitions."""
    relative_times = [0]
    horizontal_velocities = [0]
    vertical_velocities = [0]
    min_segment_time_s = 1e-6  # avoid div by zero when dist ~= 0 after dedup / float noise
    for i in range(1, len(points)):
        p1, p2 = np.array(points[i - 1]), np.array(points[i])
        dist = float(np.linalg.norm(p2 - p1))
        time_needed = calculate_time_to_move(dist, max_velocity, max_acceleration)
        if dist < 1e-15 or time_needed <= 0 or not np.isfinite(time_needed):
            time_needed = min_segment_time_s
            horizontal_velocity = 0.0
            vertical_velocity = 0.0
        else:
            horizontal_velocity = (p2[0] - p1[0]) / time_needed
            vertical_velocity = (p2[1] - p1[1]) / time_needed
        relative_times.append(time_needed)
        horizontal_velocities.append(horizontal_velocity)
        vertical_velocities.append(vertical_velocity)
    return relative_times, horizontal_velocities, vertical_velocities

def generate_csv_from_points(points, output_filename, max_velocity, max_acceleration, is_cut=None):
    """Generates CSV with positions and velocities; optional Cut column (1=cut, 0=travel)."""
    fuzz = 0.001
    points = np.asarray(points, dtype=float)
    if is_cut is None:
        unique_points = remove_close_points(points, fuzz)
        is_cut_u = None
    else:
        is_cut = np.asarray(is_cut, dtype=bool)
        unique_pts = []
        cut_flags = []
        for i, pt in enumerate(points):
            if not unique_pts:
                unique_pts.append(pt.copy())
                cut_flags.append(is_cut[i])
                continue
            if any(np.linalg.norm(pt - p) < fuzz for p in unique_pts):
                continue
            unique_pts.append(pt.copy())
            cut_flags.append(is_cut[i])
        unique_points = np.array(unique_pts)
        is_cut_u = np.array(cut_flags, dtype=bool)

    times, horizontal_velocities, vertical_velocities = compute_relative_time_and_velocity(
        unique_points, max_velocity, max_acceleration
    )

    unique_points = unique_points.copy()
    unique_points[:, 0] = unique_points[:, 0] - unique_points[0, 0]
    unique_points[:, 1] = unique_points[:, 1] - unique_points[0, 1]
    first_point = unique_points[0:1]
    deltas = np.diff(unique_points, axis=0)
    rel_points = np.vstack([first_point, deltas])

    pvt_data = {
        "Time (s)": times,
        "Horizontal position (mm)": rel_points[:, 0],
        "Horizontal velocity (mm/s)": horizontal_velocities,
        "Vertical position (mm)": rel_points[:, 1],
        "Vertical velocity (mm/s)": vertical_velocities,
    }
    if is_cut_u is not None and len(is_cut_u) == len(unique_points):
        pvt_data["Cut (1=cut 0=travel)"] = is_cut_u.astype(int)

    pvt_df = pd.DataFrame(pvt_data)
    pvt_df.to_csv(output_filename, index=False)
    print(f"✅ CSV file saved to {output_filename}")

def plot_points_with_velocity_vectors(
    points, horizontal_velocities, vertical_velocities, offset_distance=0.001, save_path=None
):
    """Plots the path with velocity vectors and point order labels.

    Uses blocking show() so the window stays until you close it (VS / some hosts
    otherwise exit and the figure disappears). If save_path is set, also writes a PNG.
    """
    plt.ioff()
    num_points = len(points)
    centroid = np.mean(points, axis=0)
    plt.figure(figsize=(10, 6))
    plt.plot(points[:, 0], points[:, 1], color='blue', marker='o', markersize=4, label='Path')
    plt.quiver(
        points[:, 0], points[:, 1], horizontal_velocities, vertical_velocities,
        angles='xy', scale_units='xy', scale=0.1, color='red', label='Velocity Vectors'
    )
    for i, point in enumerate(points):
        direction = np.array([point[0] - centroid[0], point[1] - centroid[1]])
        direction_normalized = direction / (np.linalg.norm(direction) + 1e-12)
        offset = direction_normalized * offset_distance
        plt.text(point[0] + offset[0], point[1] + offset[1], str(i), fontsize=8, color='black', ha='center', va='center')
    plt.xlabel("X Coordinate (mm)")
    plt.ylabel("Y Coordinate (mm)")
    plt.title("Path with Velocity Vectors and Point Order")
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Preview image saved to {save_path}")
    plt.show(block=True)

def prompt_overlap_settings():
    """
    Ask in the terminal how to apply overlap on closed contours.
    Returns (overlap_count, overlap_fraction).
    If overlap_count > 0, it wins and overlap_fraction is ignored by apply_overlap_closed.
    """
    print()
    print("--- Laser overlap (closed contours only) ---")
    print(
        "After one full pass on a closed loop, the path can repeat the first part of the loop\n"
        "again so the laser passes over the same region twice (stronger cut).\n"
        "  • By count: repeat the first N interpolated points (integer ≥ 1).\n"
        "  • By fraction: repeat the first (fraction × loop length) points, e.g. 0.05 = 5%.\n"
        "If you enter a count > 0, the fraction is ignored.\n"
    )
    overlap_count = 0
    overlap_fraction = 0.0

    ans = input("Add overlap on closed contours? [y/N]: ").strip().lower()
    if ans not in ("y", "yes"):
        print("Overlap disabled.\n")
        return overlap_count, overlap_fraction

    mode = input("Use (c)ount of points or (f)raction of loop? [c/f, default f]: ").strip().lower()
    if mode in ("c", "count"):
        while True:
            raw = input("Number of points to repeat from the start of each closed loop [e.g. 10]: ").strip()
            try:
                overlap_count = int(raw)
                if overlap_count < 1:
                    print("Enter a positive integer, or 0 to cancel count mode.")
                    continue
                print(f"Using overlap_count = {overlap_count} (fraction ignored).\n")
                return overlap_count, 0.0
            except ValueError:
                print("Invalid integer; try again.")
    else:
        while True:
            raw = input("Fraction of each closed loop to repeat [0–1, default 0.05]: ").strip()
            if not raw:
                overlap_fraction = 0.05
                break
            try:
                overlap_fraction = float(raw)
                if overlap_fraction <= 0 or overlap_fraction > 1:
                    print("Enter a number between 0 and 1 (e.g. 0.05 for 5%).")
                    continue
                break
            except ValueError:
                print("Invalid number; try again.")
        print(f"Using overlap_fraction = {overlap_fraction}\n")
        return 0, overlap_fraction


def prompt_dense_travel():
    """True = sample each travel segment at full spacing (more PVT rows). False = endpoints only."""
    print()
    print(
        "Travel between contours: 'dense' samples the rapid move at the same spacing as the cut\n"
        "(many points on long jumps). 'Sparse' uses only segment endpoints (fewer rows).\n"
    )
    ans = input("Use dense travel sampling? [Y/n]: ").strip().lower()
    return ans not in ("n", "no")


def prompt_gap_split_mode():
    """
    Optional diagnostic path: split a flat merged polyline by gap threshold instead of
    using per-entity contours from the DXF (see split_flat_polyline_by_gap).
    """
    print()
    ans = input(
        "Contour mode: (e)ntity-based from DXF [default], or (g)ap-split on merged point list? [e/g]: "
    ).strip().lower()
    return ans in ("g", "gap", "gap-split")


if __name__ == '__main__':
    ''' dxf_file = input("Path to DXF file [SingleChipUpperLung.dxf]: ").strip() or "SingleChipUpperLung.dxf" '''
    dxf_file = "C:/Users/DaveGleason/Desktop/FILETEST/SingleChipUpperLung.dxf"
    spacing = 0.01
    max_velocity = 100
    max_acceleration = 5000

    dxf_resolved = resolve_dxf_path(dxf_file)

    if prompt_gap_split_mode():
        flat = generate_points_from_dxf(dxf_resolved, spacing)
        contours = split_flat_polyline_by_gap(flat, spacing, k_multiplier=5.0)
        print(f"Gap-split produced {len(contours)} segment(s) from merged polyline.\n")
    else:
        contours = generate_contours_from_dxf(dxf_resolved, spacing)

    if not contours:
        print("No contours extracted; exiting.")
        raise SystemExit(1)

    contours_ordered = order_contours_greedy_nn(contours, start_idx=0)

    overlap_count, overlap_fraction = prompt_overlap_settings()

    dense_travel = prompt_dense_travel()

    optimized_points, is_cut = build_cutting_path_with_bridges(
        contours_ordered,
        spacing,
        overlap_count=overlap_count,
        overlap_fraction=overlap_fraction,
        dense_travel=dense_travel,
    )

    times, horizontal_velocities, vertical_velocities = compute_relative_time_and_velocity(
        optimized_points, max_velocity, max_acceleration
    )

    output_csv = os.path.splitext(dxf_resolved)[0] + "_pvt.csv"
    generate_csv_from_points(
        optimized_points, output_csv, max_velocity, max_acceleration, is_cut=is_cut
    )

    preview_png = os.path.splitext(dxf_resolved)[0] + "_preview.png"
    plot_points_with_velocity_vectors(
        optimized_points, horizontal_velocities, vertical_velocities, save_path=preview_png
    )
