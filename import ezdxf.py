import ezdxf
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import math

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

def generate_points_from_dxf(dxf_file, spacing):
    """Extracts points from DXF ARC, LINE, CIRCLE, and ELLIPSE entities with equal spacing."""
    dxf_file = "C:/Users/DaveGleason/Desktop/FILETEST/SingleChipUpperLung.dxf"
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()
    points = []

    # Process LINE entities
    for entity in msp.query("LINE"):
        start_point = np.array((entity.dxf.start.x, entity.dxf.start.y))
        end_point = np.array((entity.dxf.end.x, entity.dxf.end.y))
        distance = np.linalg.norm(end_point - start_point)
        num_points = max(2, int(distance / spacing))

        segment_points = np.linspace(start_point, end_point, num_points)
        points.extend(segment_points)

    # Process ARC entities
    for entity in msp.query("ARC"):
        center = (entity.dxf.center.x, entity.dxf.center.y)
        radius = entity.dxf.radius
        start_angle = entity.dxf.start_angle
        end_angle = entity.dxf.end_angle

        arc_points = interpolate_arc(center, radius, start_angle, end_angle, spacing)
        points.extend(arc_points)

    # Process CIRCLE entities
    for entity in msp.query("CIRCLE"):
        center = (entity.dxf.center.x, entity.dxf.center.y)
        radius = entity.dxf.radius

        circle_points = interpolate_circle(center, radius, spacing)
        points.extend(circle_points)

    # Process ELLIPSE entities
    for entity in msp.query("ELLIPSE"):
        center = (entity.dxf.center.x, entity.dxf.center.y)
        major_axis = entity.dxf.major_axis.magnitude  # No division by 2
        minor_axis = major_axis * entity.dxf.ratio  # Calculate the semi-minor axis using the ratio
        start_param = entity.dxf.start_param
        end_param = entity.dxf.end_param

        # Interpolate ellipse points
        ellipse_points = interpolate_ellipse(center, major_axis, minor_axis, start_param, end_param, spacing)

        # Rotate ellipse points 90 degrees clockwise about the center point
        rotated_ellipse_points = []
        for (x, y) in ellipse_points:
            # Translate points to origin
            x_shifted = x - center[0]
            y_shifted = y - center[1]

            # Rotate 90 degrees clockwise: (x', y') = (y, -x)
            x_rotated = -y_shifted
            y_rotated = x_shifted

            # Translate back to the original center
            rotated_ellipse_points.append((x_rotated + center[0], y_rotated + center[1]))

        points.extend(rotated_ellipse_points)

    return np.array(points)

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
        dist = np.linalg.norm(p2 - p1)

        time_needed = calculate_time_to_move(dist, max_velocity, max_acceleration)

        # Calculate velocities in both directions
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

if __name__ == '__main__':
    dxf_file = "SingleChipUpperLung.dxf"  # Path to your DXF file
    spacing = 0.01  # Spacing between points (in mm)
    max_velocity = 100  # Max velocity (in mm/sec)
    max_acceleration = 5000  # Max acceleration (in mm/s^2)

    # Extract points from the DXF using the generate_points_from_dxf function
    extracted_points = generate_points_from_dxf(dxf_file, spacing)

    # Optimize path traversal in counter-clockwise direction
    optimized_points = optimize_path(extracted_points)
 
    # Compute time and velocity for optimized path
    times, horizontal_velocities, vertical_velocities = compute_relative_time_and_velocity(optimized_points, max_velocity, max_acceleration)

    # Generate the CSV output (with duplicates removed)
    output_csv = dxf_file.replace(".dxf", "_pvt.csv")
    generate_csv_from_points(optimized_points, output_csv, max_velocity, max_acceleration)

    # Plot the optimized path with velocity vectors
    plot_points_with_velocity_vectors(optimized_points, horizontal_velocities, vertical_velocities)
