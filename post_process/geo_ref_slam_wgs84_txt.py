#!/usr/bin/env python3

import numpy as np
from scipy.linalg import orthogonal_procrustes
import matplotlib.pyplot as plt
from pyproj import Transformer
import argparse
import os
import pandas as pd
import folium


# === Original Functions (from your file) ===

def read_gps_file(filepath):
    """Reads GPS data from the provided txt file."""
    timestamps = []
    lats = []
    lons = []
    alts = []
    with open(filepath, 'r') as f:
        header_skipped = False
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            try:
                ts = float(parts[0])
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                timestamps.append(ts)
                lats.append(lat)
                lons.append(lon)
                alts.append(alt)
                header_skipped = True
            except ValueError:
                if not header_skipped:
                    print(f"Skipping header line: {line.strip()}")
                    continue
                else:
                    print(f"Warning: Could not parse line: {line.strip()}")
    return np.array(timestamps), np.array(lats), np.array(lons), np.array(alts)


def read_slam_file(filepath):
    """Reads SLAM poses (TUM format) from the provided txt file."""
    timestamps = []
    xs = []
    ys = []
    zs = []
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if not parts or line.startswith('#'):
                continue
            try:
                ts = float(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                timestamps.append(ts)
                xs.append(x)
                ys.append(y)
                zs.append(z)
            except (ValueError, IndexError):
                print(f"Warning: Could not parse SLAM line: {line.strip()}")
    return np.array(timestamps), np.array(xs), np.array(ys), np.array(zs)


def convert_wgs84_to_hk1980(lats, lons, alts):
    """Converts WGS84 (lat, lon, alt) to HK1980 (E, N, H)."""
    transformer = Transformer.from_crs("epsg:4326", "epsg:2326", always_xy=True)
    eastings, northings = transformer.transform(lons, lats)
    heights = alts
    return eastings, northings, heights


def match_trajectories(gps_ts, gps_coords_2d, slam_ts, slam_coords_2d, tolerance=0.5):
    """Matches GPS and SLAM points based on timestamps."""
    matched_gps = []
    matched_slam = []
    sorted_slam_indices = np.argsort(slam_ts)
    sorted_slam_ts = slam_ts[sorted_slam_indices]
    sorted_slam_coords = slam_coords_2d[sorted_slam_indices]
    for i, ts in enumerate(gps_ts):
        idx = np.searchsorted(sorted_slam_ts, ts)
        candidates = []
        if idx < len(sorted_slam_ts):
            diff_right = abs(sorted_slam_ts[idx] - ts)
            if diff_right <= tolerance:
                candidates.append((diff_right, idx))
        if idx > 0:
            diff_left = abs(sorted_slam_ts[idx - 1] - ts)
            if diff_left <= tolerance:
                candidates.append((diff_left, idx - 1))
        if candidates:
            candidates.sort()
            best_idx_in_sorted = candidates[0][1]
            best_idx_original = sorted_slam_indices[best_idx_in_sorted]
            matched_gps.append(gps_coords_2d[i])
            matched_slam.append(slam_coords_2d[best_idx_original])
    if len(matched_gps) < 3:
        raise ValueError(f"Insufficient matching pairs found ({len(matched_gps)}) for alignment.")
    print(f"Found {len(matched_gps)} matching pairs for alignment.")
    return np.array(matched_gps), np.array(matched_slam)


def align_trajectories_horn(gps_pts, lidar_pts):
    """Aligns two 2D point sets using Horn's method (SVD-based)."""
    if len(gps_pts) != len(lidar_pts):
        raise ValueError("Point sets must have the same number of points.")
    gps_array = np.array(gps_pts)
    lidar_array = np.array(lidar_pts)
    gps_center = np.mean(gps_array, axis=0)
    lidar_center = np.mean(lidar_array, axis=0)
    gps_centered = gps_array - gps_center
    lidar_centered = lidar_array - lidar_center
    norm_lidar = np.linalg.norm(lidar_centered)
    norm_gps = np.linalg.norm(gps_centered)
    if norm_lidar == 0 or norm_gps == 0:
        raise ValueError("Degenerate case: Points have zero variance.")
    scale = norm_gps / norm_lidar
    H = lidar_centered.T @ gps_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    translation = gps_center - scale * (R @ lidar_center)
    return scale, R, translation


def apply_transformation(trajectory_2d, scale, rotation, translation):
    """Applies the scale, rotation, and translation to a 2D trajectory."""
    transformed = []
    for point in trajectory_2d:
        pt_array = np.array(point)
        transformed_pt = scale * (rotation @ pt_array) + translation
        transformed.append(transformed_pt)
    return np.array(transformed)


# === Plotting Functions ===

def plot_raw_trajectories(gps_lats, gps_lons, slam_xs, slam_ys, output_dir):
    output_path = os.path.join(output_dir, "step1_raw_trajectories.png")
    plt.figure(figsize=(12, 10))
    plt.plot(gps_lons, gps_lats, 'b-', label='GPS (WGS84 Lat/Lon View)', linewidth=1, alpha=0.7)
    plt.scatter(gps_lons, gps_lats, c='blue', s=5, alpha=0.6)
    plt.plot(slam_xs, slam_ys, 'r-', label='SLAM (Local Frame x/y)', linewidth=1, alpha=0.7)
    plt.scatter(slam_xs, slam_ys, c='red', s=5, alpha=0.6)
    plt.xlabel("Longitude / SLAM X")
    plt.ylabel("Latitude / SLAM Y")
    plt.title("Step 1: Raw Trajectories (GPS in Lat/Lon view, SLAM in local x/y)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Raw trajectories plot saved to {output_path}")


def plot_converted_gps(gps_eastings, gps_northings, output_dir):
    output_path = os.path.join(output_dir, "step2_converted_gps_hk1980.png")
    plt.figure(figsize=(12, 10))
    plt.plot(gps_eastings, gps_northings, 'b-', label='GPS (HK1980 Easting/Northing)', linewidth=1, alpha=0.7)
    plt.scatter(gps_eastings, gps_northings, c='blue', s=5, alpha=0.6)
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Step 2: GPS Trajectory in HK1980 Coordinates")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Converted GPS plot saved to {output_path}")


def plot_only_georeferenced_slam(aligned_slam_coords_2d, gps_coords_2d, output_dir):
    output_path = os.path.join(output_dir, "step3_only_georeferenced_slam.png")
    plt.figure(figsize=(12, 10))
    if gps_coords_2d is not None:
        plt.scatter(gps_coords_2d[:, 0], gps_coords_2d[:, 1], c='lightblue', s=2, alpha=0.5,
                    label='GPS (HK1980) Reference', edgecolors='none')
    plt.plot(aligned_slam_coords_2d[:, 0], aligned_slam_coords_2d[:, 1], 'g-', label='SLAM (Geo-referenced HK1980)',
             linewidth=1.5, alpha=0.9)
    plt.scatter(aligned_slam_coords_2d[:, 0], aligned_slam_coords_2d[:, 1], c='green', s=8, alpha=0.7, edgecolors='none')
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Final Geo-referenced SLAM Trajectory (HK1980)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    if len(aligned_slam_coords_2d) > 0:
        slam_x_min, slam_x_max = np.min(aligned_slam_coords_2d[:, 0]), np.max(aligned_slam_coords_2d[:, 0])
        slam_y_min, slam_y_max = np.min(aligned_slam_coords_2d[:, 1]), np.max(aligned_slam_coords_2d[:, 1])
        padding = max(5.0, 0.05 * (slam_x_max - slam_x_min), 0.05 * (slam_y_max - slam_y_min))
        plt.xlim(slam_x_min - padding, slam_x_max + padding)
        plt.ylim(slam_y_min - padding, slam_y_max + padding)
    else:
        print("Warning: SLAM coordinates array is empty.")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Geo-referenced SLAM plot saved to {output_path}")


def save_geo_referenced_slam_wgs84_integer_ts(slam_ts, slam_xyz, aligned_slam_coords_2d, output_file_path):
    """Saves geo-referenced SLAM in WGS84 with integer timestamps."""
    transformer = Transformer.from_crs("epsg:2326", "epsg:4326", always_xy=True)
    lons, lats = transformer.transform(aligned_slam_coords_2d[:, 0], aligned_slam_coords_2d[:, 1])
    with open(output_file_path, 'w') as f:
        f.write("# Geo-referenced SLAM Trajectory in WGS84\n")
        f.write("# Timestamp is integer (no decimals), coordinates in degrees, height in meters\n")
        f.write("# timestamp latitude longitude height\n")
        for i in range(len(slam_ts)):
            ts_int = int(round(slam_ts[i]))
            lat = lats[i]
            lon = lons[i]
            height = slam_xyz[i, 2]
            f.write(f"{ts_int} {lat:.8f} {lon:.8f} {height:.6f}\n")
    print(f"Geo-referenced SLAM trajectory saved to {output_file_path}")


# === Save 2D Alignment Parameters (No Timestamp) ===

def save_alignment_parameters(scale, R, T, output_dir):
    """Saves 2D scale, rotation matrix, and translation vector. No generation timestamp."""
    output_path = os.path.join(output_dir, "geo_ref_matrix.txt")
    with open(output_path, 'w') as f:
        f.write("# Alignment Parameters (2D)\n")
        f.write(f"\nscale = {scale:.8f}\n")
        f.write("\nrotation_matrix_2d:\n")
        f.write(f"  [{R[0,0]: .6f}, {R[0,1]: .6f}]\n")
        f.write(f"  [{R[1,0]: .6f}, {R[1,1]: .6f}]\n")
        f.write(f"\ntranslation_vector_2d = [{T[0]:.6f}, {T[1]:.6f}]\n")
        angle_deg = np.degrees(np.arctan2(R[1,0], R[0,0]))
        f.write(f"rotation_angle_xy_deg = {angle_deg:.6f}\n")
    print(f"Alignment parameters saved to {output_path}")


# === Map Generation ===

def create_gps_accuracy_map(gps_file, output_path):
    """Creates GPS accuracy map with color-coded segments."""
    def parse_gnss_stream(file_path):
        records = []
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if not parts or len(parts) % 4 != 0:
                    continue
                for i in range(0, len(parts), 4):
                    if i + 3 >= len(parts):
                        break
                    try:
                        unix = int(parts[i])
                        lat = float(parts[i+1])
                        lon = float(parts[i+2])
                        alt = float(parts[i+3])
                        records.append({'unix': unix, 'lat': lat, 'lon': lon})
                    except ValueError:
                        continue
        return pd.DataFrame(records)

    df = parse_gnss_stream(gps_file)
    if df.empty:
        print("No GPS data to map.")
        return

    df = df.sort_values('unix').reset_index(drop=True)
    window = 5
    lat_std = df['lat'].rolling(window, center=True, min_periods=1).std()
    lon_std = df['lon'].rolling(window, center=True, min_periods=1).std()
    df['jitter_m'] = np.sqrt(lat_std**2 + lon_std**2) * 111320
    df['jitter_m'] = df['jitter_m'].fillna(0).clip(upper=5.0)
    good = df['jitter_m'] < 0.5

    m = folium.Map(location=[df['lat'].mean(), df['lon'].mean()], zoom_start=18)
    for i in range(len(df)-1):
        start = (df.iloc[i]['lat'], df.iloc[i]['lon'])
        end = (df.iloc[i+1]['lat'], df.iloc[i+1]['lon'])
        color = '#0000FF' if good.iloc[i] else '#FF0000'
        folium.PolyLine([start, end], color=color, weight=5, opacity=0.8).add_to(m)

    legend = """
    <div style="position: fixed; bottom: 50px; left: 50px; width: 180px; height: 70px;
                border:2px solid grey; z-index:9999; font-size:14px; background-color:white; opacity:0.9;">
        &nbsp;<b>Accuracy</b><br>
        &nbsp;<i class="fa fa-circle" style="color:#0000FF"></i>&nbsp; Good (<50cm)<br>
        &nbsp;<i class="fa fa-circle" style="color:#FF0000"></i>&nbsp; Warning (&ge;50cm)
    </div>
    """
    m.get_root().html.add_child(folium.Element(legend))
    m.save(output_path)
    print(f"GPS map saved to {output_path}")


def create_slam_map(slam_file, output_path):
    """Creates map for geo-referenced SLAM."""
    try:
        df = pd.read_csv(slam_file, sep=r'\s+', comment='#')
        df.columns = ['timestamp', 'lat', 'lon', 'height']
        if df.empty:
            print("No SLAM data to map.")
            return
        m = folium.Map(location=[df['lat'].mean(), df['lon'].mean()], zoom_start=18)
        coords = df[['lat', 'lon']].values.tolist()
        folium.PolyLine(coords, color='green', weight=5, opacity=0.9).add_to(m)
        folium.Marker(coords[0], popup="Start", icon=folium.Icon(color="blue")).add_to(m)
        folium.Marker(coords[-1], popup="End", icon=folium.Icon(color="red")).add_to(m)
        legend = """
        <div style="position: fixed; bottom: 50px; left: 50px; width: 180px; height: 50px;
                    border:2px solid grey; z-index:9999; font-size:14px; background-color:white; opacity:0.9;">
            &nbsp;<b>SLAM Path</b><br>
            &nbsp;<span style="color:green;">●</span>&nbsp; Geo-referenced Path
        </div>
        """
        m.get_root().html.add_child(folium.Element(legend))
        m.save(output_path)
        print(f"SLAM map saved to {output_path}")
    except Exception as e:
        print(f"Error creating SLAM map: {e}")


# === Main Pipeline ===

def main_pipeline(gps_file, slam_file, plot_steps=True):
    print(f"Processing GPS file: {gps_file}")
    print(f"Processing SLAM file: {slam_file}")

    # Ensure files exist
    for f in [gps_file, slam_file]:
        if not os.path.isfile(f):
            print(f"Error: File not found: {f}")
            exit(1)

    # Determine output directory (parent dir of GPS file)
    output_dir = os.path.dirname(gps_file)
    os.makedirs(output_dir, exist_ok=True)

    output_file = os.path.join(output_dir, "slam_geo_referenced.txt")

    # Read data
    gps_ts, gps_lats, gps_lons, gps_alts = read_gps_file(gps_file)
    if len(gps_ts) == 0:
        print("No GPS data.")
        return

    slam_ts, slam_xs, slam_ys, slam_zs = read_slam_file(slam_file)
    if len(slam_ts) == 0:
        print("No SLAM data.")
        return
    slam_xy = np.column_stack((slam_xs, slam_ys))

    # Plot 1: Raw
    if plot_steps:
        plot_raw_trajectories(gps_lats, gps_lons, slam_xs, slam_ys, output_dir)

    # Convert GPS
    gps_eastings, gps_northings, _ = convert_wgs84_to_hk1980(gps_lats, gps_lons, gps_alts)
    gps_hk1980 = np.column_stack((gps_eastings, gps_northings))

    # Plot 2: Converted GPS
    if plot_steps:
        plot_converted_gps(gps_eastings, gps_northings, output_dir)

    # Match and align
    try:
        matched_gps, matched_slam = match_trajectories(gps_ts, gps_hk1980, slam_ts, slam_xy)
        scale, R, T = align_trajectories_horn(matched_gps, matched_slam)
        print(f"Alignment: Scale={scale:.6f}, Rot={np.degrees(np.arctan2(R[1,0], R[0,0])):.4f}°, Trans={T}")
    except Exception as e:
        print(f"Alignment failed: {e}")
        return

    # Save alignment params (2D only, no timestamp)
    save_alignment_parameters(scale, R, T, output_dir)

    # Apply and save
    aligned_slam = apply_transformation(slam_xy, scale, R, T)
    slam_xyz = np.column_stack((slam_xs, slam_ys, slam_zs))
    save_geo_referenced_slam_wgs84_integer_ts(slam_ts, slam_xyz, aligned_slam, output_file)

    # Plot 3: Final
    if plot_steps:
        plot_only_georeferenced_slam(aligned_slam, gps_hk1980, output_dir)

    # Maps
    create_gps_accuracy_map(gps_file, os.path.join(output_dir, "gps.html"))
    create_slam_map(output_file, os.path.join(output_dir, "slam_geo_referenced.html"))

    # Final report
    aligned_matched = apply_transformation(matched_slam, scale, R, T)
    errors = np.linalg.norm(matched_gps - aligned_matched, axis=1)
    print(f"\n--- Alignment Quality ---")
    print(f"Mean Error: {np.mean(errors):.4f} m, Std: {np.std(errors):.4f} m")
    print(f"\n✅ All outputs saved to: {output_dir}/")


# === CLI Entry ===

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process GPS and SLAM files directly → full pipeline")
    parser.add_argument("gps_file", help="Path to GPS file (e.g., gps_data_*.txt)")
    parser.add_argument("slam_file", help="Path to SLAM file (e.g., poses_tum_converted.txt or slam_poses_*.txt)")
    parser.add_argument("--no-plots", action="store_true", help="Disable matplotlib plots")
    args = parser.parse_args()

    main_pipeline(args.gps_file, args.slam_file, not args.no_plots)