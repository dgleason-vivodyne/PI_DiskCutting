import ezdxf
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import math

# Cut paths: only entities on layers that are **on** in the LAYER table. Layer "Startpoints" is
# excluded from cut geometry; CIRCLEs on Startpoints are used only as seam/start markers.
_EXCLUDED_LAYER_NAME = "Startpoints"


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
    """
    dxf_file = "C:/Users/DaveGleason/Desktop/FILETEST/SingleChipUpperLung.dxf"
    doc, contours = generate_contours_from_dxf(dxf_file, spacing)
    apply_startpoint_seam_rotations(doc, contours)
    if not contours:
        return np.empty((0, 2))
    return np.vstack([c["points"] for c in contours])


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


# Function to generate the CSV
def generate_csv_from_points(points, output_filename, max_velocity, max_acceleration):
    """Generates a CSV file with Axis 1 and Axis 2 positions and velocities."""

    # Collapse consecutive duplicate XY only (overlap retraces preserved)
    fuzz = 0.001  # Specify the fuzz distance (in mm)
    unique_points = dedupe_consecutive_points(points, fuzz)

    # Compute time and velocity for unique points
    times, horizontal_velocities, vertical_velocities = compute_relative_time_and_velocity(unique_points, max_velocity, max_acceleration)

    # Make the moves relative (i.e., subtract the first data point from all data points in X and Y, then subtract n-1th point from nth)
    unique_points[:,0] = unique_points[:,0] - unique_points[0,0]
    unique_points[:,1] = unique_points[:,1] - unique_points[0,1]
    first_point = unique_points[0:1]
    deltas = np.diff(unique_points, axis=0)
    rel_points = np.vstack([first_point, deltas])

    # Create DataFrame with the required data in the specified order
    pvt_data = {
        "Time (s)": times,
        "Horizontal position (mm)": rel_points[:, 0],
        "Horizontal velocity (mm/s)": horizontal_velocities,
        "Vertical position (mm)": rel_points[:, 1],
        "Vertical velocity (mm/s)": vertical_velocities
    }

    pvt_df = pd.DataFrame(pvt_data)

    # Save to CSV
    pvt_df.to_csv(output_filename, index=False)
    print(f"✅ CSV file saved to {output_filename}")


# Function to plot optimized path with velocity vectors
def plot_points_with_velocity_vectors(points, horizontal_velocities, vertical_velocities, offset_distance=0.001):
    """Plots the optimized points with velocity vectors and labels the order of points, offset to the outside."""
    num_points = len(points)

    # Compute centroid (center of mass)
    centroid = np.mean(points, axis=0)

    # Create a plot with both points and velocity vectors
    plt.figure(figsize=(10, 6))

    # Plot the path
    plt.plot(points[:, 0], points[:, 1], color='blue', marker='o', markersize=4, label='Path')

    # Plot velocity vectors
    plt.quiver(points[:, 0], points[:, 1], horizontal_velocities, vertical_velocities, angles='xy', scale_units='xy', scale=0.1, color='red', label='Velocity Vectors')

    # Label each point with its order in the sequence, offset to the outside of the path
    for i, point in enumerate(points):

        # Calculate direction vector from the centroid to the point
        direction = np.array([point[0] - centroid[0], point[1] - centroid[1]])

        # Normalize the direction vector
        direction_normalized = direction / np.linalg.norm(direction)

        # Offset the label slightly outside the path
        offset = direction_normalized * offset_distance

        # Apply offset to the label position
        label_x = point[0] + offset[0]
        label_y = point[1] + offset[1]


        # Plot the label with an offset
        plt.text(label_x, label_y, str(i), fontsize=8, color='black', ha='center', va='center')

    plt.xlabel("X Coordinate (mm)")
    plt.ylabel("Y Coordinate (mm)")
    plt.title("Optimized Path with Velocity Vectors and Point Order")
    plt.axis("equal")
    plt.legend()
    plt.show()


def prompt_overlap_point_count():
    """Ask how many points at the start of the path to repeat at the end. Returns a non-negative int (0 = none)."""
    print()
    print("--- Overlap ---")
    print(
        "After the full path, append the first N points again (same order as the start)\n"
        "to extend the cut over the beginning of the contour.\n"
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
            print(f"Overlap: {n} point(s) from the start appended after the path.\n")
            return n
        except ValueError:
            print("Enter an integer, e.g. 0 or 10.")


if __name__ == '__main__':
    dxf_file = "SingleChipUpperLung.dxf"  # Path to your DXF file
    spacing = 0.01  # Spacing between points (in mm)
    max_velocity = 100  # Max velocity (in mm/sec)
    max_acceleration = 5000  # Max acceleration (in mm/s^2)

    # DXF chain order + Startpoints seam rotation (no optimize_path — it scrambles closed curves)
    optimized_points = generate_points_from_dxf(dxf_file, spacing)

    overlap_count = prompt_overlap_point_count()
    if overlap_count > 0 and len(optimized_points) >= 1:
        n_extra = min(int(overlap_count), len(optimized_points))
        optimized_points = np.vstack([optimized_points, optimized_points[:n_extra]])

    # Compute time and velocity for optimized path
    times, horizontal_velocities, vertical_velocities = compute_relative_time_and_velocity(optimized_points, max_velocity, max_acceleration)

    # Generate the CSV output (with duplicates removed)
    output_csv = dxf_file.replace(".dxf", "_pvt.csv")
    generate_csv_from_points(optimized_points, output_csv, max_velocity, max_acceleration)

    # Plot the optimized path with velocity vectors
    plot_points_with_velocity_vectors(optimized_points, horizontal_velocities, vertical_velocities)
