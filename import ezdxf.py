import csv
import json
import math
import os
from collections import deque
import tkinter as tk
from tkinter import filedialog, messagebox

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


def interpolate_spline(entity, spacing):
    """
    Sample a DXF SPLINE entity to a 2D polyline in WCS (mm).

    Uses ezdxf adaptive ``Spline.flattening(distance)`` to approximate the spline, then
    re-densifies each resulting chord with ``spacing`` so point-to-point motion matches
    the spacing-driven behavior used by other sampled entity types.

    Parameters
    ----------
    entity
        A DXF entity with dxftype ``SPLINE``.
    spacing : float
        Target segment spacing (mm); clamped to a small positive minimum.

    Returns
    -------
    ndarray of shape (N, 2), dtype float64, or None if not a SPLINE or flattening fails.
    """
    if entity.dxftype() != "SPLINE":
        return None
    step = max(float(spacing), 1e-9)
    tol = step
    try:
        verts = list(entity.flattening(distance=tol))
    except (ValueError, AttributeError):
        return None
    if len(verts) < 2:
        return None
    base = np.array([[float(v.x), float(v.y)] for v in verts], dtype=float)

    dense = [base[0]]
    for i in range(1, len(base)):
        dense_seg = _chord_dense_samples(base[i - 1], base[i], step)
        if len(dense_seg) > 0:
            dense.extend(dense_seg)
    if len(dense) < 2:
        return None
    return np.asarray(dense, dtype=float)


def interpolate_polyline(entity, spacing):
    """
    Sample a DXF ``LWPOLYLINE`` or ``POLYLINE`` (2D outline with line and bulge arcs) to XY samples.

    Uses ``ezdxf.path.make_path`` to resolve straight segments and arc bulges, then ``Path.flattening``
    with tolerance ``spacing`` (same role as for SPLINE flattening).

    ``POLYLINE`` polymesh / entities that cannot be converted to a path return ``None``.

    Parameters
    ----------
    entity
        DXF entity with dxftype ``LWPOLYLINE`` or ``POLYLINE``.
    spacing : float
        Flattening tolerance (mm).

    Returns
    -------
    ndarray of shape (N, 2), dtype float64, or None.
    """
    dt = entity.dxftype()
    if dt not in ("LWPOLYLINE", "POLYLINE"):
        return None
    tol = max(float(spacing), 1e-9)
    try:
        pth = make_path(entity)
    except Exception:
        return None
    try:
        verts = list(pth.flattening(distance=tol))
    except (ValueError, AttributeError):
        return None
    if len(verts) < 2:
        return None
    return np.array([[float(v.x), float(v.y)] for v in verts], dtype=float)


def interpolate_entity_xy(entity, spacing):
    """Sample one LINE/ARC/CIRCLE/ELLIPSE/SPLINE/LWPOLYLINE/POLYLINE to a polyline in WCS (mm)."""
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
    if dt == "SPLINE":
        return interpolate_spline(entity, spacing)
    if dt in ("LWPOLYLINE", "POLYLINE"):
        return interpolate_polyline(entity, spacing)
    return None


def _infer_chain_closed(
    points,
    spacing,
    gap_threshold,
    single_circle,
    single_full_ellipse,
    single_closed_spline=False,
    single_closed_polyline=False,
):
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        return False
    if single_circle: 
        return True
    if single_full_ellipse:
        return True
    if single_closed_spline:
        return True
    if single_closed_polyline:
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


def _order_segments_into_chains(
    segments: list,
    gap_threshold: float,
) -> list[list[tuple]]:
    """
    Group interpolated DXF primitives into contours and order each contour head-to-tail.

    Parameters
    ----------
    segments
        ``(entity, raw_xy)`` items with ``raw_xy`` ``(N, 2)`` from ``interpolate_entity_xy``,
        in arbitrary order (typically modelspace iteration order).
    gap_threshold
        Max endpoint distance (mm) for treating two segments as adjacent.

    Returns
    -------
    List of contours; each contour is ``[(entity, oriented_raw_ndarray), ...]`` in traversal order.
    """
    n = len(segments)
    if n == 0:
        return []

    unused = set(range(n))
    chains_out: list[list[tuple]] = []

    def best_attach_to_tail(tail: np.ndarray) -> tuple[int | None, np.ndarray | None, float]:
        best_j: int | None = None
        best_raw: np.ndarray | None = None
        best_d = float("inf")
        for j in unused:
            raw = np.asarray(segments[j][1], dtype=float)
            if len(raw) == 0:
                continue
            if len(raw) == 1:
                d = float(np.linalg.norm(raw[0] - tail))
                if d < best_d:
                    best_d, best_j, best_raw = d, j, raw.copy()
                continue
            d_fwd = float(np.linalg.norm(raw[0] - tail))
            d_rev = float(np.linalg.norm(raw[-1] - tail))
            if d_rev < d_fwd - 1e-9:
                cand_d, cand_raw = d_rev, np.flipud(raw).copy()
            else:
                cand_d, cand_raw = d_fwd, raw.copy()
            if cand_d < best_d:
                best_d, best_j, best_raw = cand_d, j, cand_raw
        if best_j is None:
            return None, None, float("inf")
        return best_j, best_raw, best_d

    def best_attach_to_head(head: np.ndarray) -> tuple[int | None, np.ndarray | None, float]:
        """Pick a segment to prepend so its **last** vertex meets ``head``."""
        best_j: int | None = None
        best_raw: np.ndarray | None = None
        best_d = float("inf")
        for j in unused:
            raw = np.asarray(segments[j][1], dtype=float)
            if len(raw) == 0:
                continue
            if len(raw) == 1:
                d = float(np.linalg.norm(raw[0] - head))
                if d < best_d:
                    best_d, best_j, best_raw = d, j, raw.copy()
                continue
            d_fwd = float(np.linalg.norm(raw[-1] - head))
            d_rev = float(np.linalg.norm(raw[0] - head))
            if d_rev < d_fwd - 1e-9:
                cand_d, cand_raw = d_rev, np.flipud(raw).copy()
            else:
                cand_d, cand_raw = d_fwd, raw.copy()
            if cand_d < best_d:
                best_d, best_j, best_raw = cand_d, j, cand_raw
        if best_j is None:
            return None, None, float("inf")
        return best_j, best_raw, best_d

    while unused:
        seed = min(unused)
        unused.remove(seed)
        ent0, raw0 = segments[seed]
        chain_dq: deque[tuple] = deque()
        chain_dq.append((ent0, np.asarray(raw0, dtype=float).copy()))

        while True:
            tail = chain_dq[-1][1][-1]
            j, oriented, d = best_attach_to_tail(tail)
            if j is None or d > gap_threshold:
                break
            unused.remove(j)
            chain_dq.append((segments[j][0], oriented))

        while True:
            head = chain_dq[0][1][0]
            j, oriented, d = best_attach_to_head(head)
            if j is None or d > gap_threshold:
                break
            unused.remove(j)
            chain_dq.appendleft((segments[j][0], oriented))

        chains_out.append(list(chain_dq))

    return chains_out


def generate_contours_from_dxf(
    dxf_path: str,
    spacing: float,
    gap_multiplier: float = 5.0,
    min_stitch_gap_mm: float = 1.0,
):
    """
    Head-to-tail chains: collect LINE/ARC/CIRCLE/ELLIPSE/SPLINE/LWPOLYLINE/POLYLINE on allowed
    layers into contiguous modelspace **runs** (turned-off layers still split runs, except
    Startpoints markers). Within each run, **sort primitives by endpoint proximity** (within gap
    threshold), orient each segment along the cut, then merge polylines. Returns a list of dicts:
    ``{"points": (N,2) float array}``.

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

    segment_runs: list[list[tuple]] = []
    current_run: list[tuple] = []
    for entity in msp:
        dt = entity.dxftype()
        if dt not in ("LINE", "ARC", "CIRCLE", "ELLIPSE", "SPLINE", "LWPOLYLINE", "POLYLINE"):
            continue
        if not _entity_layer_processed(doc, entity):
            # Same split semantics as legacy entity-order pass: layer-off (except Startpoints)
            # ends the current run so hidden sandwiched geometry cannot stitch separate cuts.
            if _is_startpoints_layer_name(entity):
                continue
            if current_run:
                segment_runs.append(current_run)
                current_run = []
            continue
        raw = interpolate_entity_xy(entity, spacing)
        if raw is None or len(raw) == 0:
            continue
        current_run.append((entity, raw))
    if current_run:
        segment_runs.append(current_run)

    ordered_chains: list[list[tuple]] = []
    for run in segment_runs:
        ordered_chains.extend(_order_segments_into_chains(run, gap_threshold))

    chains = []
    for ordered in ordered_chains:
        current = None
        chain_entities = []
        for entity, raw in ordered:
            raw = np.asarray(raw, dtype=float)
            if current is None:
                current = raw.copy()
                chain_entities = [entity]
                continue

            prev_tail = current[-1]
            seg = raw.copy()
            d_joint = float(np.linalg.norm(seg[0] - prev_tail))

            if d_joint > gap_threshold:
                chains.append((current, chain_entities))
                current = seg.copy()
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
        single_closed_spline = False
        if len(ents) == 1 and ents[0].dxftype() == "SPLINE":
            single_closed_spline = bool(getattr(ents[0], "closed", False))
        single_closed_polyline = False
        if len(ents) == 1:
            e0 = ents[0]
            if e0.dxftype() == "LWPOLYLINE":
                single_closed_polyline = bool(e0.closed)
            elif e0.dxftype() == "POLYLINE":
                single_closed_polyline = bool(getattr(e0, "is_closed", False))
        closed = _infer_chain_closed(
            pts,
            spacing,
            gap_threshold,
            single_circle,
            single_full_ellipse,
            single_closed_spline,
            single_closed_polyline,
        )
        contours.append({"points": pts.copy(), "closed": closed, "dxftype": "+".join(types) if len(types) <= 3 else f"CHAIN[{len(types)}]"})

    return doc, contours


def generate_points_from_dxf(dxf_file, spacing):
    """
    Extract points from DXF: contours on allowed layers with primitives ordered head-to-tail.

    Vertex order along each contour follows stitched LINE/ARC/CIRCLE/ELLIPSE/SPLINE/LWPOLYLINE/POLYLINE samples;
    primitives are grouped and oriented by **endpoint proximity** before stitching (DXF draw order may differ).
    Do **not** run ``optimize_path`` on those points:
    angle-sort + nearest-neighbor reorders dense samples on a closed curve and produces long
    chords that zigzag across the interior (looks like non-sequential hops in a preview).

    Startpoints: **closed** contours get a **cyclic shift** so the vertex nearest the marker is
    first (still one continuous loop). **Open** contours only **reverse** if needed so the nearer
    **endpoint** is first—cyclic shifts are not used on open polylines (they would add an
    artificial wrap jump mid-contour).     Contours are concatenated in contour order (large moves
    between separate contours remain).

    Returns ``(flat_points, contour_chunks, contour_closed)`` where ``contour_chunks`` is a list of
    ``(N,2)`` arrays used for laser on/off insertion in the CSV exporter, and ``contour_closed`` is
    a parallel list of bools from DXF chain inference (used by optional travel-order optimization).

    ``dxf_file`` must be the path to the DXF on disk (e.g. from a file dialog when run as a script).
    """
    # dxf_file = "C:/Users/DaveGleason/Desktop/FILETEST/SingleChipUpperLung.dxf"  # legacy hardcoded override
    doc, contours = generate_contours_from_dxf(dxf_file, spacing)
    apply_startpoint_seam_rotations(doc, contours)
    if not contours:
        return np.empty((0, 2)), [], []
    chunks = [np.asarray(c["points"], dtype=float).copy() for c in contours]
    closed_flags = [bool(c.get("closed", False)) for c in contours]
    flat = np.vstack(chunks)
    return flat, chunks, closed_flags


def _travel_variant_candidates_for_chunk(
    pts: np.ndarray,
    is_closed: bool,
    spacing_mm: float,
    max_seam_samples: int,
):
    """
    Enumerate legal contour-level variants: cyclic shifts ± direction (closed) or forward/reverse (open).
    Point spacing along the contour is unchanged — only start vertex and traversal direction.
    """
    p = np.asarray(pts, dtype=float)
    if len(p) < 2:
        return [p.copy()]
    tol_close = max(float(spacing_mm) * 3.0, 0.05)

    if not is_closed:
        return [p.copy(), np.flipud(p).copy()]

    if len(p) >= 2 and float(np.linalg.norm(p[0] - p[-1])) <= tol_close:
        ring = p[:-1].copy()
    else:
        ring = p.copy()
    m = len(ring)
    if m < 3:
        return [p.copy()]

    n_k = min(m, max(6, int(max_seam_samples)))
    ks = np.unique(np.linspace(0, m - 1, num=n_k, dtype=np.int64))
    out = []
    for k in ks:
        k = int(k) % m
        fwd = np.vstack([ring[k:], ring[:k]])
        out.append(fwd.copy())
        back_idx = np.array([(k - j) % m for j in range(m)], dtype=np.int64)
        out.append(ring[back_idx].copy())
    return out


def optimize_contour_chunks_travel_greedy(
    contour_chunks,
    contour_closed,
    *,
    spacing_mm: float,
    origin_xy=(0.0, 0.0),
    max_seam_samples: int = 48,
):
    """
    Reorder contours and choose seam (closed loops) and direction to shorten Euclidean rapid moves
    from ``origin_xy`` and between successive contour ends. Uses a greedy nearest-contour/next-entry
    heuristic (not globally optimal).

    Does **not** resample geometry: only permutes whole contours and applies whole-chain reversal /
    cyclic shifts for closed polylines.

    Parameters
    ----------
    contour_chunks : list of (N,2) arrays
    contour_closed : list of bool, same length as ``contour_chunks``
    spacing_mm : float
        DXF sample spacing (mm); used for duplicate closure detection tolerance only.
    origin_xy : tuple
        Rapid-move start (CAD WCS), typically ``(0, 0)`` for greedy ordering / seam choices.
    max_seam_samples : int
        Max seam indices sampled around each closed contour (each yields forward + reverse traversal).

    Returns
    -------
    list of (N,2) arrays
        Reordered and transformed chunks to pass to ``flatten_contours_with_per_contour_overlap`` /
        ``generate_csv_from_points``.
    """
    n = len(contour_chunks)
    if n == 0:
        return []
    if len(contour_closed) != n:
        raise ValueError("contour_closed must match contour_chunks length")

    variants_per = [
        _travel_variant_candidates_for_chunk(
            contour_chunks[i],
            bool(contour_closed[i]),
            spacing_mm,
            max_seam_samples,
        )
        for i in range(n)
    ]

    remaining = set(range(n))
    ordered = []
    current = np.asarray(origin_xy, dtype=float).reshape(2)

    while remaining:
        best_cost = float("inf")
        best_var = None
        best_i = None
        for i in remaining:
            for var in variants_per[i]:
                if len(var) < 1:
                    continue
                ent = np.asarray(var[0], dtype=float).reshape(2)
                d = float(np.linalg.norm(ent - current))
                if d < best_cost:
                    best_cost = d
                    best_var = np.asarray(var, dtype=float).copy()
                    best_i = i
        if best_i is None:
            break
        ordered.append(best_var)
        current = np.asarray(best_var[-1], dtype=float).reshape(2)
        remaining.remove(best_i)

    return ordered


def prompt_optimize_contour_travel() -> bool:
    """Tk yes/no: optionally optimize contour order / seam / direction for shorter rapid moves."""
    root = tk.Tk()
    root.withdraw()
    try:
        root.attributes("-topmost", True)
    except tk.TclError:
        pass
    try:
        yes = messagebox.askyesno(
            "Optimize air moves?",
            "Optimize contour cutting order, seam position on closed loops, and cut direction\n"
            "to shorten rapid moves? (Greedy heuristic from CAD origin — not globally optimal.)\n\n"
            "Startpoints seam hints may be overridden when this option is enabled.",
            icon="question",
        )
    finally:
        root.destroy()
    return bool(yes)


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


def _default_rapid_max_velocity_mm_s(max_velocity, max_acceleration, spacing_mm):
    """
    Peak speed scale for a spacing-sized chord under symmetric accel (triangle profile):
    dt = sqrt(2*d/a), |v| = d/dt = sqrt(a*d/2). Matches typical cut-row speeds when d ≈ spacing.
    """
    d = max(float(spacing_mm), 1e-9)
    v_like_cut = float(math.sqrt(max_acceleration * d / 2.0))
    return float(min(max_velocity, v_like_cut))


def _schedule_kind_at_segment_index(schedule, j):
    """Return schedule kind (e.g. ``cut``, ``travel``) for segment index ``j``."""
    for kind, lo, hi in schedule:
        if lo <= j < hi:
            return kind
    raise IndexError(f"segment index {j} not covered by schedule")


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


def _append_straight_segment_single(segs, p0, p1):
    """Append one straight segment from ``p0`` to ``p1`` (no spacing subdivision)."""
    p0 = np.asarray(p0, dtype=float).reshape(2)
    p1 = np.asarray(p1, dtype=float).reshape(2)
    if float(np.linalg.norm(p1 - p0)) <= 1e-12:
        return
    segs.append((p0.copy(), p1.copy()))


def _symmetric_fillet_T1_T2(P, u_chord_to_P, v_out, d):
    """Corner ``P``; unit ``u_chord_to_P`` from anchor toward ``P``; unit ``v_out`` from ``P`` toward cut."""
    u = np.asarray(u_chord_to_P, dtype=float).reshape(2)
    v = np.asarray(v_out, dtype=float).reshape(2)
    T1 = P - float(d) * u
    T2 = P + float(d) * v
    return T1, T2


def _solve_travel_corner_fillet_fixed_radius(p_start, v_out, L, anchor, R_fixed_mm, theta_min_rad):
    """
    Symmetric circular fillet at extended corner P = p_start - (L+d)*v_out between chord
    anchor->P and direction v_out. Uses **fixed** fillet radius ``R_fixed_mm`` (clamped by local
    geometry).

    Gating uses the **path deflection** ``beta = angle(u, v_out)`` where ``u`` is the unit chord
    anchor->P (incoming direction into the corner). ``beta`` is 0 for collinear continuation and
    ``pi`` for reversal; **no** arc when ``beta < theta_min_rad``. (Using ``acos(dot(-u,v_out))``
    would compare ``pi - beta`` and invert shallow vs sharp turns.)

    Tangent offsets along each leg use ``d = R * tan(beta/2)`` with ``R <= lc / tan(beta/2)``
    (``lc`` = chord length anchor→P). **Do not** swap with ``R / tan(beta/2)`` — that breaks the
    circle through ``T1``/``T2`` and shifts the cumulative path before the cut.

    Keeps a straight of length L from T2 = p_start - L*v_out to p_start (cut).

    Returns (P, d, R_use, u_chord, delta_arc_rad, T1, T2) where ``delta_arc_rad`` is the angle
    between rays P->anchor and P->cut (for arc-center bisector), i.e. ``acos(dot(-u, v_out))``.
    """
    R_cap = max(float(R_fixed_mm), 0.0)
    anchor = np.asarray(anchor, dtype=float).reshape(2)
    v_out = _unit2d(v_out)
    p_start = np.asarray(p_start, dtype=float).reshape(2)

    def _no_fillet():
        T2 = p_start - L * v_out
        P0 = p_start - L * v_out
        w = P0 - anchor
        lc = float(np.linalg.norm(w))
        if lc > 1e-9:
            u = w / lc
        else:
            u = v_out.copy()
        return P0, 0.0, 0.0, u, 0.0, T2.copy(), T2.copy()

    if R_cap <= 1e-12:
        return _no_fillet()

    d = 0.0
    for _ in range(8):
        P = p_start - (L + d) * v_out
        w = P - anchor
        lc = float(np.linalg.norm(w))
        if lc <= 1e-9:
            return _no_fillet()
        u = w / lc
        cos_beta = float(np.clip(np.dot(u, v_out), -1.0, 1.0))
        beta = float(math.acos(cos_beta))
        if beta < float(theta_min_rad):
            return _no_fillet()
        th2 = 0.5 * beta
        tan_h = math.tan(th2)
        if tan_h < 1e-12:
            return _no_fillet()
        # Tangent offset along each leg: d = R * tan(beta/2); chord budget d <= lc => R <= lc / tan(beta/2).
        R_geom = lc / tan_h
        R_use = min(R_cap, R_geom)
        d_new = R_use * tan_h
        if abs(d_new - d) < 1e-9:
            d = d_new
            break
        d = d_new

    P = p_start - (L + d) * v_out
    w = P - anchor
    lc = float(np.linalg.norm(w))
    if lc <= 1e-9:
        return _no_fillet()
    u = w / max(lc, 1e-15)
    cos_beta = float(np.clip(np.dot(u, v_out), -1.0, 1.0))
    beta = float(math.acos(cos_beta))
    if beta < float(theta_min_rad):
        return _no_fillet()
    th2 = 0.5 * beta
    tan_h = max(math.tan(th2), 1e-12)
    R_geom = lc / tan_h
    R_use = min(R_cap, R_geom)
    if R_use <= 1e-12:
        return _no_fillet()
    d = R_use * tan_h
    T1, T2 = _symmetric_fillet_T1_T2(P, u, v_out, d)
    cos_delta = float(np.clip(np.dot(-u, v_out), -1.0, 1.0))
    delta_arc = float(math.acos(cos_delta))
    return P, d, R_use, u, delta_arc, T1, T2


def _fillet_arc_center_from_corner(P_corner, u_chord_to_P, v_out, R, theta_rad):
    """
    Circle center for a symmetric fillet at corner ``P_corner`` between incoming direction
    ``-u_chord_to_P`` (from corner toward anchor) and outgoing ``v_out``. Uses internal angle
    bisector (stable vs offset-from-T1 construction).
    """
    u = _unit2d(u_chord_to_P)
    v = _unit2d(v_out)
    u_back = -u
    th = max(float(theta_rad), 1e-12)
    sh = math.sin(0.5 * th)
    if sh < 1e-12:
        return None
    bis = u_back + v
    nb = float(np.linalg.norm(bis))
    if nb < 1e-12:
        return None
    bis = bis / nb
    dist_cp = float(R) / sh
    pc = np.asarray(P_corner, dtype=float).reshape(2)
    return pc + dist_cp * bis


def _travel_fillet_arc_points_on_circle(C, R, T1, T2, tol_mm=0.05):
    """True if both tangent points sit on the nominal circle within ``tol_mm`` (radius sanity)."""
    C = np.asarray(C, dtype=float).reshape(2)
    rr = float(R)
    if rr <= 1e-9:
        return False
    t1 = np.asarray(T1, dtype=float).reshape(2)
    t2 = np.asarray(T2, dtype=float).reshape(2)
    e1 = abs(float(np.linalg.norm(t1 - C)) - rr)
    e2 = abs(float(np.linalg.norm(t2 - C)) - rr)
    return max(e1, e2) <= float(tol_mm)


def _choose_arc_sweep_align_tangent_at_a(ta: float, tb: float, tangent_from_a: np.ndarray) -> float:
    """
    Two sweeps connect angles ``ta`` and ``tb`` on a circle; pick the one whose motion **leaving**
    ``p_a`` matches ``tangent_from_a`` (unit-ish vector in the motion direction along the arc).
    Falls back to the shorter sweep when ambiguous.
    """
    tgt = _unit2d(tangent_from_a)
    phi0 = math.atan2(math.sin(tb - ta), math.cos(tb - ta))
    candidates = [phi0]
    if abs(phi0) > 1e-12:
        phi1 = phi0 - 2.0 * math.pi * (1.0 if phi0 > 0.0 else -1.0)
        candidates.append(phi1)

    def leaving_dir(phi: float) -> np.ndarray:
        # theta runs from ta toward ta+phi; CCW tangent at ta is (-sin ta, cos ta)
        ccw = np.array([-math.sin(ta), math.cos(ta)], dtype=float)
        if phi >= 0.0:
            return ccw
        return -ccw

    best_phi = phi0
    best_dot = -2.0
    for phi in candidates:
        d0 = leaving_dir(phi)
        sc = float(np.dot(d0, tgt))
        if sc > best_dot + 1e-12:
            best_dot = sc
            best_phi = phi
    return float(best_phi)


def _append_arc_segments_densified(segs, C, r, p_a, p_b, spacing, tangent_from_a=None):
    """
    Circular arc from ``p_a`` to ``p_b`` on center ``C``, radius ``r``, densified.

    By default uses the **shorter** angular sweep. If ``tangent_from_a`` is set (preferred direction
    of motion **leaving** ``p_a`` along the arc), chooses between the two sweeps so that starting
    tangent best aligns — avoids a ~180° hook when the minor sweep runs backward (seen on lead-out).

    Snaps the polyline to exact ``p_a`` / ``p_b`` endpoints so cumulative deltas match ``p_b - p_a``
    even when ``atan2`` sampling drifts slightly from the true tangent points.
    """
    C = np.asarray(C, dtype=float).reshape(2)
    a = np.asarray(p_a, dtype=float).reshape(2)
    b = np.asarray(p_b, dtype=float).reshape(2)
    rr = float(r)
    if rr <= 1e-9:
        _append_straight_segment_single(segs, a, b)
        return
    va = a - C
    vb = b - C
    ta = math.atan2(float(va[1]), float(va[0]))
    tb = math.atan2(float(vb[1]), float(vb[0]))
    if tangent_from_a is None:
        phi = math.atan2(math.sin(tb - ta), math.cos(tb - ta))
    else:
        phi = _choose_arc_sweep_align_tangent_at_a(ta, tb, tangent_from_a)
    arc_len = abs(rr * phi)
    step = max(float(spacing), 1e-9)
    n = max(2, int(math.ceil(arc_len / step)))
    prev = a.copy()
    for i in range(1, n + 1):
        t1 = i / n
        if i == n:
            p1_ = b.copy()
        else:
            ang1 = ta + phi * t1
            p1_ = C + rr * np.array([math.cos(ang1), math.sin(ang1)], dtype=float)
        segs.append((prev.copy(), p1_.copy()))
        prev = p1_


def _cross2d(a, b) -> float:
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    return ax * by - ay * bx


def _solve_lead_out_arc_after_decel_straight(
    E,
    dir_out,
    u_travel_from_E,
    lc_travel_chord,
    R_fixed_mm,
    theta_min_rad,
    *,
    chord_extra_mm: float = 0.0,
    chord_fraction: float = 1.0,
    arc_length_max_mm: float | None = None,
):
    """
    After a **full** collinear deceleration ending at ``E`` along ``dir_out``, round the corner into
    inter-contour travel direction ``w`` with a circular arc that **starts at E** tangent to
    ``dir_out``. Called only when another contour follows; the **final** contour stops at ``E`` with
    no arc.

    **Variable radius:** for turn angle ``beta`` between ``dir_out`` and ``w``, a feasible radius
    must satisfy ``t = 2 R (w·n) <= lc`` where ``lc`` is the travel chord budget along ``w`` and
    ``n`` is the inward normal used below — equivalently ``R <= lc / (2 (w·n))``. We use
    ``R_use = min(R_cap, R_geom)`` with ``R_cap = travel_fillet_radius_mm`` and
    ``R_geom = lc / (2 (w·n))``, i.e. **the largest circle that fits** inside the cap and chord
    budget (farthest tangent on the travel ray). Optional ``arc_length_max_mm`` shrinks ``R`` so
    arc length ``R * beta`` does not exceed that cap.

    ``chord_fraction`` / ``chord_extra_mm`` scale or inflate ``lc`` derived from the travel probe
    (lets you shrink or extend how far along ``w`` the tangent may sit).

    Returns ``(C_center, R_use, T2)`` or ``None`` if skipped (shallow turn or degenerate geometry).
    """
    R_cap = max(float(R_fixed_mm), 0.0)
    if R_cap <= 1e-12:
        return None
    E = np.asarray(E, dtype=float).reshape(2)
    lc0 = float(lc_travel_chord)
    fr = max(float(chord_fraction), 0.0)
    ex = max(float(chord_extra_mm), 0.0)
    lc_out = lc0 * fr + ex
    if lc_out <= 1e-9:
        return None
    w = _unit2d(u_travel_from_E)
    u0 = _unit2d(dir_out)
    cos_beta = float(np.clip(np.dot(u0, w), -1.0, 1.0))
    beta = float(math.acos(cos_beta))
    if beta < float(theta_min_rad):
        return None
    cr = _cross2d(u0, w)
    if abs(cr) < 1e-14 and abs(beta) > 1e-6:
        return None
    sgn = 1.0 if cr >= 0.0 else -1.0
    n = sgn * np.array([-u0[1], u0[0]], dtype=float)
    wdn = float(np.dot(w, n))
    if wdn <= 1e-12:
        return None
    R_geom = lc_out / (2.0 * wdn)
    R_use = min(R_cap, R_geom)
    if arc_length_max_mm is not None and beta > 1e-12:
        R_use = min(R_use, max(float(arc_length_max_mm), 0.0) / beta)
    if R_use <= 1e-12:
        return None
    t_ray = 2.0 * R_use * wdn
    if t_ray > lc_out + 1e-5:
        return None
    C = E + R_use * n
    T2 = E + t_ray * w
    if not _travel_fillet_arc_points_on_circle(C, R_use, E, T2):
        return None
    return C, float(R_use), T2


def _try_append_unified_inter_contour_transition(
    segs,
    E,
    dir_out,
    p_next,
    dn,
    L,
    R_cap_mm: float,
    theta_min_rad: float,
    spacing: float,
):
    """
    One tangent-preserving corner from lead-out end ``E`` along ``dir_out`` to the next contour's
    collinear approach point ``B = p_next - L * dn``. Uses the intersection of the two infinite lines
    (exit ray and approach ray) as the fillet vertex — tangent points slide along those lines instead
    of stacking a separate lead-out arc + travel arc.

    Appends: optional ``E→T1``, arc ``T1→T2``, optional ``T2→B``. Returns ``(True, B)`` if the
    motion was emitted (straight-only counts), else ``(False, None)`` so callers can fall back to the
    legacy lead-out arc + travel fillet chain.
    """
    E = np.asarray(E, dtype=float).reshape(2)
    p_next = np.asarray(p_next, dtype=float).reshape(2)
    dir_u = _unit2d(dir_out)
    dn_u = _unit2d(dn)
    B = p_next - float(L) * dn_u

    cross = _cross2d(dir_u, dn_u)
    if abs(cross) < 1e-12:
        _append_straight_segment_single(segs, E, B)
        return True, B.copy()

    diff = B - E
    t_hit = float(_cross2d(diff, dn_u) / cross)
    V = E + t_hit * dir_u

    if t_hit < -1e-5:
        return False, None

    lv = float(np.linalg.norm(V - E))
    lb = float(np.linalg.norm(V - B))
    if lv < 1e-9 or lb < 1e-9:
        _append_straight_segment_single(segs, E, B)
        return True, B.copy()

    # ``u_ve`` points V→E; motion toward the corner runs E→V (−u_ve). ``u_vb`` points V→B (departure).
    # Fillet turn angle is the **path deflection** ψ between −u_ve and u_vb, not the acute angle between u_ve and u_vb.
    u_ve = _unit2d(E - V)
    u_vb = _unit2d(B - V)
    cos_psi = float(np.clip(np.dot(-u_ve, u_vb), -1.0, 1.0))
    psi = float(math.acos(cos_psi))

    def _straight_fallback():
        _append_straight_segment_single(segs, E, B)
        return True, B.copy()

    if psi < float(theta_min_rad):
        return _straight_fallback()

    tan_h = math.tan(0.5 * psi)
    if tan_h < 1e-12:
        return _straight_fallback()

    R_cap = max(float(R_cap_mm), 0.0)
    if R_cap <= 1e-12:
        return _straight_fallback()

    R_geom = min(lv, lb) / tan_h
    R_use = min(R_cap, R_geom)
    if R_use <= 1e-12:
        return _straight_fallback()

    d = R_use * tan_h
    if d > lv + 1e-6 or d > lb + 1e-6:
        return _straight_fallback()

    T1 = V + d * u_ve
    T2 = V + d * u_vb

    chord_et = np.asarray(T1, dtype=float).reshape(2) - E.reshape(2)
    tan_in = _unit2d(chord_et) if float(np.linalg.norm(chord_et)) > 1e-9 else (-u_ve).copy()
    p_left = np.array([-tan_in[1], tan_in[0]], dtype=float)
    n1 = _unit2d(p_left)
    n2 = -n1
    best_C = None
    best_err = float("inf")
    for nrm in (n1, n2):
        Cc = np.asarray(T1, dtype=float).reshape(2) + float(R_use) * nrm
        err = max(
            abs(float(np.linalg.norm(T1 - Cc)) - R_use),
            abs(float(np.linalg.norm(T2 - Cc)) - R_use),
        )
        if err < best_err:
            best_err = err
            best_C = Cc
    if best_C is None or best_err > 0.08:
        return _straight_fallback()
    C = best_C

    if float(np.linalg.norm(chord_et)) > 1e-9:
        _append_straight_segment_single(segs, E, T1)
    tan_a = chord_et if float(np.linalg.norm(chord_et)) > 1e-9 else dir_u
    _append_arc_segments_densified(segs, C, R_use, T1, T2, spacing, tangent_from_a=tan_a)
    if float(np.linalg.norm(B.reshape(2) - np.asarray(T2, dtype=float).reshape(2))) > 1e-9:
        _append_straight_segment_single(segs, T2, B)

    return True, B.copy()


def build_export_segments_with_leads(
    full,
    starts,
    chunks_d,
    overlap_n,
    spacing,
    max_velocity,
    max_acceleration,
    cad_origin_xy=(0.0, 0.0),
    travel_fillet_radius_mm: float = 0.35,
    travel_fillet_min_turn_deg: float = 25.0,
    travel_fillet_chord_extra_mm: float = 0.0,
    travel_fillet_chord_fraction: float = 1.0,
    travel_fillet_lead_out_arc_length_max_mm: float | None = None,
):
    """
    Build ordered motion segments: per contour (lead-in -> cut polyline with overlap ->
    lead-out), then travel to the next contour's lead-in start.

    Lead-in/out keep a straight collinear segment of length ``L = v_max^2/(2 a_tan)`` immediately
    before/after the cut: **lead-in** is (approach fillet if any) then **L** flush to the cut start;
    **lead-out** is **L** flush from the cut end to ``E = p_end + L dir_out``. When another contour
    follows, an optional arc at ``E`` blends into inter-contour travel; the **final** contour omits
    that arc (straight lead-out only — no extra motion after ``E``). Non-cutting corners
    (CAD origin approach, lead-out→travel when applicable, travel at the next lead-in) may use a
    **small fixed-radius**
    circular fillet when the **path deflection** at that vertex is at least
    ``travel_fillet_min_turn_deg``; otherwise motion stays straight (allows modest accel jumps).
    **Variable-radius fillets:** ``travel_fillet_radius_mm`` is only a **maximum**; actual
    ``R_use`` is ``min(R_cap, R_geometry)`` from local chord/tangent budgets (lead-in/travel corners
    use the iterative corner solver; lead-out→travel arc uses the budget along the travel probe when
    a next contour exists). Optional ``travel_fillet_chord_fraction`` /
    ``travel_fillet_chord_extra_mm`` adjust that probe budget for the **lead-out→travel** arc only;
    ``travel_fillet_lead_out_arc_length_max_mm`` caps that arc length
    ``R * beta``. Lead-in corner solver still uses ``d = R * tan(beta/2)`` tangent offsets at the
    virtual approach corner. ``spacing`` subdivides **fillet arc** motion only; other non-cutting
    chords are single segments.

    Returns ``(segs, replay, schedule)`` where ``segs`` is a list of (p0, p1), ``replay`` maps
    destination segment index -> source index for overlap retrace, and ``schedule`` is a list of
    (kind, lo, hi) with hi exclusive segment indices into ``segs``.
    """
    L = _lead_length_mm(max_velocity, max_acceleration)
    theta_min_rad = math.radians(float(travel_fillet_min_turn_deg))
    O = np.asarray(cad_origin_xy, dtype=float).reshape(2)

    segs = []
    replay = {}
    schedule = []
    k_count = len(chunks_d)
    next_lead_anchor = O.copy()

    for k in range(k_count):
        V = full[starts[k] : starts[k + 1]]
        if len(V) < 1:
            continue

        if len(V) >= 2:
            dir_in = _unit2d(V[1] - V[0])
            dir_out = _unit2d(V[-1] - V[-2])
        else:
            uu = np.array([1.0, 0.0], dtype=float)
            dir_in = uu
            dir_out = uu

        p_start = V[0]
        li_lo = len(segs)
        B_exp = p_start - L * dir_in
        anchor_snap_tol = max(0.05, float(spacing) * 10.0)
        if float(np.linalg.norm(next_lead_anchor - B_exp)) <= anchor_snap_tol:
            _append_straight_segment_single(segs, next_lead_anchor, p_start)
        else:
            _P, d_li, R_li, u_li, th_li, T1_li, T2_li = _solve_travel_corner_fillet_fixed_radius(
                p_start, dir_in, L, next_lead_anchor, travel_fillet_radius_mm, theta_min_rad
            )
            if d_li > 1e-6 and R_li > 1e-6:
                C_li = _fillet_arc_center_from_corner(_P, u_li, dir_in, R_li, th_li)
                if C_li is None or not _travel_fillet_arc_points_on_circle(C_li, R_li, T1_li, T2_li):
                    _append_straight_segment_single(segs, T1_li, T2_li)
                else:
                    li_chord = np.asarray(T1_li, dtype=float).reshape(2) - np.asarray(next_lead_anchor, dtype=float).reshape(2)
                    li_tan = li_chord if float(np.linalg.norm(li_chord)) > 1e-9 else None
                    _append_arc_segments_densified(segs, C_li, R_li, T1_li, T2_li, spacing, tangent_from_a=li_tan)
            _append_straight_segment_single(segs, T2_li, p_start)
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
        lo_lo = len(segs)

        W_next = full[starts[k + 1] : starts[k + 2]] if k < k_count - 1 else []
        has_next_contour = len(W_next) >= 1

        _append_straight_segment_single(segs, p_end, E)
        lo_hi_lead = len(segs)

        tr_lo = 0
        tr_hi = 0
        did_travel = False

        if has_next_contour:
            if len(W_next) >= 2:
                dn = _unit2d(W_next[1] - W_next[0])
            else:
                dn = np.array([1.0, 0.0], dtype=float)
            p_next = W_next[0]

            tr_lo = len(segs)
            ok_u, B_anchor = _try_append_unified_inter_contour_transition(
                segs,
                E,
                dir_out,
                p_next,
                dn,
                L,
                travel_fillet_radius_mm,
                theta_min_rad,
                spacing,
            )
            if ok_u and B_anchor is not None:
                tr_hi = len(segs)
                next_lead_anchor = np.asarray(B_anchor, dtype=float).reshape(2).copy()
                did_travel = True
            else:
                travel_from = E
                _P_probe, _, _, u_tr0, _, _, _ = _solve_travel_corner_fillet_fixed_radius(
                    p_next,
                    dn,
                    L,
                    E,
                    travel_fillet_radius_mm,
                    theta_min_rad,
                )
                lc_travel = float(np.linalg.norm(np.asarray(_P_probe - E, dtype=float).reshape(2)))
                lo_arc = _solve_lead_out_arc_after_decel_straight(
                    E,
                    dir_out,
                    u_tr0,
                    lc_travel,
                    travel_fillet_radius_mm,
                    theta_min_rad,
                    chord_extra_mm=travel_fillet_chord_extra_mm,
                    chord_fraction=travel_fillet_chord_fraction,
                    arc_length_max_mm=travel_fillet_lead_out_arc_length_max_mm,
                )
                if lo_arc is not None:
                    C_e, R_e, T2_e = lo_arc
                    _append_arc_segments_densified(segs, C_e, R_e, E, T2_e, spacing, tangent_from_a=dir_out)
                    travel_from = T2_e
                else:
                    travel_from = E

                lo_hi_lead = len(segs)

                tr_lo = len(segs)
                _P_tr, d_tr, R_tr, u_tr, th_tr, T1_tr, T2_tr = _solve_travel_corner_fillet_fixed_radius(
                    p_next, dn, L, travel_from, travel_fillet_radius_mm, theta_min_rad
                )
                _append_straight_segment_single(segs, travel_from, T1_tr)
                if d_tr > 1e-6 and R_tr > 1e-6:
                    C_tr = _fillet_arc_center_from_corner(_P_tr, u_tr, dn, R_tr, th_tr)
                    if C_tr is None or not _travel_fillet_arc_points_on_circle(C_tr, R_tr, T1_tr, T2_tr):
                        _append_straight_segment_single(segs, T1_tr, T2_tr)
                    else:
                        tr_chord = np.asarray(T1_tr, dtype=float).reshape(2) - np.asarray(travel_from, dtype=float).reshape(2)
                        tr_tan = tr_chord if float(np.linalg.norm(tr_chord)) > 1e-9 else None
                        _append_arc_segments_densified(segs, C_tr, R_tr, T1_tr, T2_tr, spacing, tangent_from_a=tr_tan)
                tr_hi = len(segs)
                next_lead_anchor = T2_tr.copy()
                did_travel = True

        schedule.append(("lead_out", lo_lo, lo_hi_lead))
        if did_travel:
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
    travel_fillet_radius_mm: float = 0.35,
    travel_fillet_min_turn_deg: float = 25.0,
    travel_fillet_chord_extra_mm: float = 0.0,
    travel_fillet_chord_fraction: float = 1.0,
    travel_fillet_lead_out_arc_length_max_mm: float | None = None,
    rapid_max_velocity_mm_s=None,
):
    """
    Writes the main PVT CSV with laser I/O only around **cut** motion (not during lead-in/out or
    travel). Lead-in/out keep a straight collinear segment of length ``v_max^2 / (2 a_tan)`` mm
    flush against the cut (approach fillet before lead-in ``L``, exit ``L`` then, if another contour
    follows, optional arc into travel — omitted on the final contour).
    Non-cutting corners use an optional variable-radius circular fillet when the **path deflection**
    is at least ``travel_fillet_min_turn_deg``;
    ``travel_fillet_radius_mm`` is a **maximum**; realized radius is ``min(cap, geometry)``. Optional
    ``travel_fillet_chord_fraction`` / ``travel_fillet_chord_extra_mm`` adjust lead-out→travel arc
    chord budget when applicable; ``travel_fillet_lead_out_arc_length_max_mm`` caps that arc. Shallower corners
    stay straight.
    Cut polylines follow DXF sampling; non-cutting **arc** blends (lead-in, lead-out→travel when there
    is a next contour, travel at the next lead-in) use ``spacing``
    along the arc; remaining non-cutting straights are single
    segments. Inter-contour motion is travel (laser off).
    ``0,0,0,0,0`` sync starts the file; relative file ends with laser-off rows.

    Writes a sibling ``*_absolute.csv`` with the same headers and motion order, **no** laser text
    rows, ``0,0,0,0,0`` at start and end; columns 2 and 4 are absolute endpoints ``Origin + p1``.

    Does **not** emit travel rows from CAD WCS ``(0,0)`` to the lead-in: the relative CSV begins at
    the first motion segment (typically lead-in entry). ``<stem>.pvt.meta.json`` sets
    ``origin_xy_mm`` to CAD WCS at that path start (``segs[0][0]``); DMS applies it as a stage
    prelude after placing CAD ``(0,0)`` for buffers 1/2 (see ``LaserService.RunOnePvtInGlobalAsync``
    and ``RunSinglePvtFromCurrentPositionAsync``).

    Overlap retrace (cut polyline only): repeated edges reuse the same PVT row as the first pass.

    ``spacing``: DXF sample spacing (mm) for cut geometry from entities; also chord spacing along
    travel **arc** segments only (non-arc non-cutting chords stay single segments).

    ``rapid_max_velocity_mm_s``: Upper bound on speed magnitude used for **lead-in**, **lead-out**,
    and **travel** rows (same ``max_acceleration``). **Cut** segments always use ``max_velocity``.
    If ``None``, defaults to ``min(max_velocity, sqrt(max_acceleration * spacing / 2))``, i.e. the
    peak speed of a spacing-sized symmetric-accel move — similar to dense cut sampling so long
    travel chords do not cruise at full ``max_velocity``. Pass a positive float to override.
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

    cad_o = (0.0, 0.0)
    segs, replay, schedule = build_export_segments_with_leads(
        full,
        starts,
        chunks_d,
        overlap_n,
        spacing,
        max_velocity,
        max_acceleration,
        cad_origin_xy=cad_o,
        travel_fillet_radius_mm=travel_fillet_radius_mm,
        travel_fillet_min_turn_deg=travel_fillet_min_turn_deg,
        travel_fillet_chord_extra_mm=travel_fillet_chord_extra_mm,
        travel_fillet_chord_fraction=travel_fillet_chord_fraction,
        travel_fillet_lead_out_arc_length_max_mm=travel_fillet_lead_out_arc_length_max_mm,
    )
    if not segs:
        raise ValueError("No export segments (empty path).")

    if rapid_max_velocity_mm_s is None:
        rapid_v = _default_rapid_max_velocity_mm_s(max_velocity, max_acceleration, spacing)
    else:
        rapid_v = float(rapid_max_velocity_mm_s)
        if rapid_v <= 0:
            raise ValueError("rapid_max_velocity_mm_s must be positive")
        rapid_v = min(rapid_v, float(max_velocity))

    rel_rows = []
    for j in range(len(segs)):
        p0, p1 = segs[j]
        kind = _schedule_kind_at_segment_index(schedule, j)
        v_cap = float(max_velocity) if kind == "cut" else rapid_v
        dt, vx, vy = _segment_time_velocity(p0, p1, v_cap, max_acceleration)
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
            "origin_xy_mm is CAD WCS (mm) at the start of the first motion segment (typically lead-in entry). "
            "No travel rows from CAD (0,0) in the relative CSV; DMS applies this offset as a stage prelude "
            "after MoveTo chip-map XY for buffers 1/2 (RunOnePvtInGlobalAsync) or with PvtOffset in "
            "RunSinglePvtFromCurrentPositionAsync."
        ),
        "max_velocity_mm_s": float(max_velocity),
        "rapid_max_velocity_mm_s": rapid_v,
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


def try_read_origin_xy_mm_from_pvt_meta(csv_path):
    """CAD WCS ``origin_xy_mm`` from sibling ``<stem>.pvt.meta.json`` if ``schema_version`` is 1; else ``None``."""
    if not csv_path:
        return None
    stem = csv_path[:-4] if csv_path.lower().endswith(".csv") else csv_path
    meta_path = stem + ".pvt.meta.json"
    if not os.path.isfile(meta_path):
        return None
    try:
        with open(meta_path, encoding="utf-8") as f:
            root = json.load(f)
        if int(root.get("schema_version", -1)) != 1:
            return None
        arr = root.get("origin_xy_mm")
        if not isinstance(arr, list) or len(arr) < 2:
            return None
        return [float(arr[0]), float(arr[1])]
    except (OSError, ValueError, TypeError):
        return None


def deltas_to_polyline(start_xy, deltas):
    """start_xy (2,), deltas (N, 2) -> (N + 1, 2) polyline including start (cumulative CSV path)."""
    start_xy = np.asarray(start_xy, dtype=float).reshape(2)
    if len(deltas) == 0:
        return start_xy.reshape(1, 2)
    cum = np.cumsum(deltas, axis=0)
    return np.vstack([start_xy, start_xy + cum])


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

    If ``csv_path`` is set, overlay cumulative CSV dx/dy. The polyline starts at ``origin_xy_mm``
    from sibling ``.pvt.meta.json`` when present; otherwise at ``(0,0)`` if dynamics args are set,
    else ``points[0]``.

    Checkboxes toggle DXF path, CSV replay, quiver, and DXF point labels.
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
        meta_o = try_read_origin_xy_mm_from_pvt_meta(csv_path)
        if meta_o is not None:
            csv_start = np.asarray(meta_o, dtype=float).reshape(2)
        elif max_velocity is not None or max_acceleration is not None:
            csv_start = np.array([0.0, 0.0], dtype=float)
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

    rax = plt.axes((0.80, 0.45, 0.18, 0.22))
    check_labels = []
    actives = []
    toggle_map = []

    check_labels.append("DXF path")
    actives.append(True)
    toggle_map.append(("line", ln_dxf))

    if ln_csv is not None:
        check_labels.append("CSV replay")
        actives.append(True)
        toggle_map.append(("line", ln_csv))

    check_labels.append("Velocities")
    actives.append(False)
    toggle_map.append(("quiv", quiv))

    check_labels.append("Point labels")
    actives.append(False)
    toggle_map.append(("texts", label_objs))

    quiv.set_visible(False)
    for _t in label_objs:
        _t.set_visible(False)

    chk = CheckButtons(rax, check_labels, actives)

    def on_check_click(_label):
        status = chk.get_status()
        for i, (typ, ref) in enumerate(toggle_map):
            vis = status[i]
            if typ == "line":
                ref.set_visible(vis)
            elif typ == "quiv":
                ref.set_visible(vis)
            elif typ == "texts":
                for t in ref:
                    t.set_visible(vis)
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
    max_acceleration = 5000  # Max tangential acceleration (in mm/s^2)
    travel_fillet_radius_mm = 0.35  # Non-cutting corner blend radius (mm); clamped by geometry
    travel_fillet_min_turn_deg = 25.0  # Skip fillet below this angle (deg); straighter = straight chords

    # DXF chain order + Startpoints seam rotation (no optimize_path — it scrambles closed curves)
    optimized_points, contour_chunks, contour_closed = generate_points_from_dxf(dxf_file, spacing)
    if prompt_optimize_contour_travel() and len(contour_chunks) > 0:
        contour_chunks = optimize_contour_chunks_travel_greedy(
            contour_chunks,
            contour_closed,
            spacing_mm=spacing,
            origin_xy=(0.0, 0.0),
        )

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
        travel_fillet_radius_mm=travel_fillet_radius_mm,
        travel_fillet_min_turn_deg=travel_fillet_min_turn_deg,
    )

    plot_points_with_velocity_vectors(
        optimized_points,
        horizontal_velocities,
        vertical_velocities,
        csv_path=output_csv,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
    )
