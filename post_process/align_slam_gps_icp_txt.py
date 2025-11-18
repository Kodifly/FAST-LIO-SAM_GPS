#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import orthogonal_procrustes
from pyproj import Transformer
import argparse
import os
import pandas as pd
import folium


# === Original Functions (adapted) ===

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


# === ICP-Based Alignment (Replaces Timestamp Matching) ===

def icp_alignment(gps_coords_2d, slam_coords_2d, max_iterations=50, tolerance=1e-4, verbose=True):
    """
    Aligns SLAM to GPS using point-to-point ICP (no timestamps needed).
    Returns: scale, rotation (2x2), translation (2,)
    """
    A = slam_coords_2d.astype(np.float64).copy()  # SLAM (source)
    B = gps_coords_2d.astype(np.float64).copy()  # GPS (target)

    if len(A) == 0 or len(B) == 0:
        raise ValueError("Empty point set in ICP alignment.")

    # Normalize scale for better convergence
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    A_centered = A - centroid_A
    B_centered = B - centroid_B
    scale_A = np.linalg.norm(A_centered)
    scale_B = np.linalg.norm(B_centered)
    if scale_A == 0 or scale_B == 0:
        raise ValueError("Degenerate point set (zero variance).")

    # Initial rough alignment: scale and center
    s_initial = scale_B / scale_A if scale_A > 1e-8 else 1.0
    A_aligned = s_initial * A_centered

    src = np.ones((3, A.shape[0]))
    src[:2, :] = A_aligned.T
    dst = np.ones((3, B.shape[0]))
    dst[:2, :] = B_centered.T

    prev_error = float('inf')

    for i in range(max_iterations):
        # Find closest points (nearest neighbor)
        distances = np.linalg.norm(src[:2, :].T[:, np.newaxis] - dst[:2, :].T, axis=2)
        indices = distances.argmin(axis=1)

        matched_dst = dst[:2, indices].T  # Closest GPS points

        # Compute optimal transformation via SVD
        AA = src[:2, :].T
        BB = matched_dst
        centroid_src = np.mean(AA, axis=0)
        centroid_dst = np.mean(BB, axis=0)
        AA_centered = AA - centroid_src
        BB_centered = BB - centroid_dst

        H = AA_centered.T @ BB_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        scale_opt = np.linalg.norm(BB_centered) / np.linalg.norm(AA_centered)
        t_opt = centroid_dst - scale_opt * (R @ centroid_src)

        # Apply transformation
        src[:2, :] = scale_opt * (R @ src[:2, :]) + t_opt.reshape(-1, 1)

        mean_error = np.mean(distances[np.arange(len(indices)), indices])
        if verbose and i % 10 == 0:
            print(f"ICP Iter {i}: Mean distance = {mean_error:.6f} m")

        if abs(prev_error - mean_error) < tolerance:
            if verbose:
                print(f"ICP converged after {i+1} iterations with error {mean_error:.6f} m")
            break
        prev_error = mean_error

    # Final transformation (chain with initial)
    final_scale = s_initial * scale_opt
    final_R = R
    final_T = centroid_B + scale_opt * (R @ (centroid_dst - s_initial * R.T @ (centroid_B - t_opt)))

    return final_scale, final_R, final_T


# === Other Functions (unchanged or slightly improved) ===

def align_trajectories_horn(gps_pts, lidar_pts):
    """Horn's method for 2D rigid alignment (used only for evaluation now)."""
    if len(gps_pts) != len(lidar_pts):
        raise ValueError("Point sets must have same length.")
    gps = np.array(gps_pts)
    lidar = np.array(lidar_pts)
    gps_mean = np.mean(gps, axis=0)
    lidar_mean = np.mean(lidar, axis=0)
    gps_c = gps - gps_mean
    lidar_c = lidar - lidar_mean
    H = lidar_c.T @ gps_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    scale = np.linalg.norm(gps_c) / np.linalg.norm(lidar_c)
    translation = gps_mean - scale * (R @ lidar_mean)
    return scale, R, translation


def apply_transformation(trajectory_2d, scale, rotation, translation):
    """Applies transformation to 2D trajectory."""
    transformed = scale * (rotation @ trajectory_2d.T).T + translation
    return transformed


# === Plotting Functions ===

def plot_raw_trajectories(gps_lats, gps_lons, slam_xs, slam_ys, output_dir):
    output_path = os.path.join(output_dir, "step1_raw_trajectories.png")
    plt.figure(figsize=(12, 10))
    plt.plot(gps_lons, gps_lats, 'b-', label='GPS (Lat/Lon)', linewidth=1, alpha=0.7)
    plt.scatter(gps_lons, gps_lats, c='blue', s=5, alpha=0.6)
    plt.plot(slam_xs, slam_ys, 'r-', label='SLAM (Local)', linewidth=1, alpha=0.7)
    plt.scatter(slam_xs, slam_ys, c='red', s=5, alpha=0.6)
    plt.xlabel("Longitude / X")
    plt.ylabel("Latitude / Y")
    plt.title("Step 1: Raw Trajectories")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Raw trajectories plot saved to {output_path}")


def plot_converted_gps(gps_eastings, gps_northings, output_dir):
    output_path = os.path.join(output_dir, "step2_converted_gps_hk1980.png")
    plt.figure(figsize=(12, 10))
    plt.plot(gps_eastings, gps_northings, 'b-', label='GPS (HK1980)', linewidth=1, alpha=0.7)
    plt.scatter(gps_eastings, gps_northings, c='blue', s=5, alpha=0.6)
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Step 2: GPS in HK1980 Coordinates")
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
                    label='GPS (HK1980)', edgecolors='none')
    plt.plot(aligned_slam_coords_2d[:, 0], aligned_slam_coords_2d[:, 1], 'g-', label='SLAM (Geo-referenced)',
             linewidth=1.5, alpha=0.9)
    plt.scatter(aligned_slam_coords_2d[:, 0], aligned_slam_coords_2d[:, 1], c='green', s=8, alpha=0.7, edgecolors='none')
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Final Geo-referenced SLAM Trajectory (HK1980)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    if len(aligned_slam_coords_2d) > 0:
        x_min, x_max = np.min(aligned_slam_coords_2d[:, 0]), np.max(aligned_slam_coords_2d[:, 0])
        y_min, y_max = np.min(aligned_slam_coords_2d[:, 1]), np.max(aligned_slam_coords_2d[:, 1])
        padding = max(5.0, 0.05 * (x_max - x_min), 0.05 * (y_max - y_min))
        plt.xlim(x_min - padding, x_max + padding)
        plt.ylim(y_min - padding, y_max + padding)
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
    print(f"Geo-referenced SLAM saved to {output_file_path}")


def save_alignment_parameters(scale, R, T, output_dir):
    """Saves 2D alignment parameters."""
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


# === Auto-detect Files (Legacy fallback) ===

def find_gps_and_slam_files(folder_path):
    gps_file = None
    slam_file = None
    for fname in os.listdir(folder_path):
        if fname.startswith("gps_data_") and fname.endswith(".txt"):
            gps_file = os.path.join(folder_path, fname)
        if fname == "poses_tum_converted.txt" or (fname.startswith("slam_poses_") and fname.endswith(".txt")):
            slam_file = os.path.join(folder_path, fname)
    if not gps_file:
        raise FileNotFoundError("No GPS file starting with 'gps_data_' found.")
    if not slam_file:
        raise FileNotFoundError("No SLAM file found: 'poses_tum_converted.txt' or 'slam_poses_*.txt'.")
    return gps_file, slam_file


# === Main Pipeline ===

def main_pipeline(gps_file, slam_file, output_dir, plot_steps=True):
    print(f"Processing GPS: {gps_file}")
    print(f"Processing SLAM: {slam_file}")
    os.makedirs(output_dir, exist_ok=True)

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

    # Convert GPS to HK1980
    gps_eastings, gps_northings, _ = convert_wgs84_to_hk1980(gps_lats, gps_lons, gps_alts)
    gps_hk1980 = np.column_stack((gps_eastings, gps_northings))

    # Pre-align: center and scale SLAM to GPS
    slam_xy_centered = slam_xy - np.mean(slam_xy, axis=0)
    gps_hk1980_centered = gps_hk1980 - np.mean(gps_hk1980, axis=0)
    slam_scale = np.linalg.norm(slam_xy_centered)
    gps_scale = np.linalg.norm(gps_hk1980_centered)
    scale_init = gps_scale / slam_scale if slam_scale > 1e-8 else 1.0
    slam_xy_pre = scale_init * slam_xy_centered + np.mean(gps_hk1980, axis=0)

    # === GEOMETRIC ALIGNMENT USING ICP (NO TIMESTAMPS) ===
    print("Aligning trajectories using ICP (geometry-only, no timestamps)...")
    try:
        scale, R, T = icp_alignment(gps_hk1980, slam_xy_pre, verbose=True)
        print(f"ICP Alignment Result:")
        print(f"  Scale: {scale:.6f}")
        print(f"  Rotation: {np.degrees(np.arctan2(R[1,0], R[0,0])):.4f}°")
        print(f"  Translation: {T}")
    except Exception as e:
        print(f"ICP alignment failed: {e}")
        return

    # Save alignment parameters
    save_alignment_parameters(scale, R, T, output_dir)

    # Apply transformation to full SLAM trajectory
    aligned_slam = apply_transformation(slam_xy_pre, scale, R, T)

    # Save geo-referenced SLAM in WGS84
    slam_xyz = np.column_stack((slam_xs, slam_ys, slam_zs))
    output_file = os.path.join(output_dir, "slam_geo_referenced.txt")
    save_geo_referenced_slam_wgs84_integer_ts(slam_ts, slam_xyz, aligned_slam, output_file)

    # Plot 3: Final aligned result
    if plot_steps:
        plot_only_georeferenced_slam(aligned_slam, gps_hk1980, output_dir)

    # Evaluate alignment quality
    from scipy.spatial.distance import cdist
    distances = cdist(gps_hk1980, aligned_slam)
    closest_idx = distances.argmin(axis=1)
    aligned_slam_matched = aligned_slam[closest_idx]
    errors = np.linalg.norm(gps_hk1980 - aligned_slam_matched, axis=1)
    print(f"\n--- Alignment Quality (ICP) ---")
    print(f"Mean Error: {np.mean(errors):.4f} m, Std: {np.std(errors):.4f} m, Max: {np.max(errors):.4f} m")

    # Generate maps
    create_gps_accuracy_map(gps_file, os.path.join(output_dir, "gps.html"))
    create_slam_map(output_file, os.path.join(output_dir, "slam_geo_referenced.html"))

    print(f"\n✅ All outputs saved to: {output_dir}/")


# === CLI Entry ===

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Align SLAM to GPS using ICP (geometry-only, no timestamps)")
    parser.add_argument("folder", nargs='?', help="Path to folder with gps_data_*.txt and slam/pose file (optional if --gps-file and --slam-file are provided)")
    parser.add_argument("--gps-file", type=str, help="Path to GPS data file (WGS84 format: timestamp lat lon alt)")
    parser.add_argument("--slam-file", type=str, help="Path to SLAM poses file (TUM format: timestamp x y z)")
    parser.add_argument("--output-dir", type=str, help="Directory to save outputs (default: directory of GPS file)")
    parser.add_argument("--no-plots", action="store_true", help="Disable matplotlib plots")
    args = parser.parse_args()

    # Validate inputs
    if not args.gps_file or not args.slam_file:
        # Fallback to auto-detect from folder
        if not args.folder:
            print("Error: Either provide --gps-file and --slam-file, OR a folder path.")
            exit(1)
        if not os.path.isdir(args.folder):
            print(f"Error: Folder not found: {args.folder}")
            exit(1)
        try:
            gps_file, slam_file = find_gps_and_slam_files(args.folder)
        except FileNotFoundError as e:
            print(f"Error: {e}")
            exit(1)
        output_dir = args.folder  # Use folder as output dir
    else:
        # Direct file mode
        if not os.path.isfile(args.gps_file):
            print(f"Error: GPS file not found: {args.gps_file}")
            exit(1)
        if not os.path.isfile(args.slam_file):
            print(f"Error: SLAM file not found: {args.slam_file}")
            exit(1)
        gps_file = args.gps_file
        slam_file = args.slam_file
        output_dir = args.output_dir if args.output_dir else os.path.dirname(os.path.abspath(gps_file))

    # Run pipeline
    main_pipeline(gps_file, slam_file, output_dir, not args.no_plots)