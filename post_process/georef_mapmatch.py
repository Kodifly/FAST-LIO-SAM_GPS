#!/usr/bin/env python3

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyproj import Transformer, CRS
from scipy.linalg import orthogonal_procrustes
from shapely.geometry import Point
from geopy.distance import great_circle
import folium

# Optional: Only import mappymatch if needed (to avoid hard dependency if not used)
try:
    from mappymatch.constructs.trace import Trace
    from mappymatch.constructs.geofence import Geofence
    from mappymatch.maps.nx.nx_map import NxMap
    from mappymatch.maps.nx.readers.osm_readers import NetworkType
    from mappymatch.matchers.lcss.lcss import LCSSMatcher
    from mappymatch.utils.plot import plot_matches
    HAS_MAPPYMATCH = True
except ImportError:
    HAS_MAPPYMATCH = False
    print("Warning: mappymatch not installed. Map-matching stage will be skipped.")

# ======================
# === Stage 1: Alignment Functions (from first script)
# ======================

def read_gps_file(filepath):
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
                    continue
                else:
                    print(f"Warning: Could not parse line: {line.strip()}")
    return np.array(timestamps), np.array(lats), np.array(lons), np.array(alts)

def read_slam_file(filepath):
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
    transformer = Transformer.from_crs("epsg:4326", "epsg:2326", always_xy=True)
    eastings, northings = transformer.transform(lons, lats)
    return eastings, northings, alts

def match_trajectories(gps_ts, gps_coords_2d, slam_ts, slam_coords_2d, tolerance=0.5):
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
        raise ValueError(f"Insufficient matching pairs ({len(matched_gps)}) for alignment.")
    print(f"Found {len(matched_gps)} matching pairs for alignment.")
    return np.array(matched_gps), np.array(matched_slam)

def align_trajectories_horn(gps_pts, lidar_pts):
    if len(gps_pts) != len(lidar_pts):
        raise ValueError("Point sets must have same length.")
    gps_array = np.array(gps_pts)
    lidar_array = np.array(lidar_pts)
    gps_center = np.mean(gps_array, axis=0)
    lidar_center = np.mean(lidar_array, axis=0)
    gps_centered = gps_array - gps_center
    lidar_centered = lidar_array - lidar_center
    norm_lidar = np.linalg.norm(lidar_centered)
    norm_gps = np.linalg.norm(gps_centered)
    if norm_lidar == 0 or norm_gps == 0:
        raise ValueError("Degenerate point sets.")
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
    return np.array([scale * (rotation @ np.array(p)) + translation for p in trajectory_2d])

# ======================
# === Plotting & Saving
# ======================

def plot_raw_trajectories(gps_lats, gps_lons, slam_xs, slam_ys, output_dir):
    path = os.path.join(output_dir, "step1_raw_trajectories.png")
    plt.figure(figsize=(12, 10))
    plt.plot(gps_lons, gps_lats, 'b-', label='GPS (WGS84)', linewidth=1, alpha=0.7)
    plt.scatter(gps_lons, gps_lats, c='blue', s=5, alpha=0.6)
    plt.plot(slam_xs, slam_ys, 'r-', label='SLAM (local)', linewidth=1, alpha=0.7)
    plt.scatter(slam_xs, slam_ys, c='red', s=5, alpha=0.6)
    plt.xlabel("Longitude / SLAM X")
    plt.ylabel("Latitude / SLAM Y")
    plt.title("Step 1: Raw Trajectories")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")

def plot_converted_gps(gps_eastings, gps_northings, output_dir):
    path = os.path.join(output_dir, "step2_converted_gps_hk1980.png")
    plt.figure(figsize=(12, 10))
    plt.plot(gps_eastings, gps_northings, 'b-', linewidth=1, alpha=0.7)
    plt.scatter(gps_eastings, gps_northings, c='blue', s=5, alpha=0.6)
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Step 2: GPS in HK1980")
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")

def plot_only_georeferenced_slam(aligned_slam, gps_hk1980, output_dir):
    path = os.path.join(output_dir, "step3_only_georeferenced_slam.png")
    plt.figure(figsize=(12, 10))
    if gps_hk1980 is not None:
        plt.scatter(gps_hk1980[:, 0], gps_hk1980[:, 1], c='lightblue', s=2, alpha=0.5, label='GPS')
    plt.plot(aligned_slam[:, 0], aligned_slam[:, 1], 'g-', linewidth=1.5, alpha=0.9, label='SLAM (geo-ref)')
    plt.scatter(aligned_slam[:, 0], aligned_slam[:, 1], c='green', s=8, alpha=0.7)
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Geo-referenced SLAM Trajectory")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    if len(aligned_slam) > 0:
        x_min, x_max = aligned_slam[:, 0].min(), aligned_slam[:, 0].max()
        y_min, y_max = aligned_slam[:, 1].min(), aligned_slam[:, 1].max()
        pad = max(5.0, 0.05 * (x_max - x_min), 0.05 * (y_max - y_min))
        plt.xlim(x_min - pad, x_max + pad)
        plt.ylim(y_min - pad, y_max + pad)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")

def save_geo_referenced_slam_wgs84_integer_ts(slam_ts, slam_xyz, aligned_slam_2d, output_file):
    transformer = Transformer.from_crs("epsg:2326", "epsg:4326", always_xy=True)
    lons, lats = transformer.transform(aligned_slam_2d[:, 0], aligned_slam_2d[:, 1])
    with open(output_file, 'w') as f:
        f.write("# Geo-referenced SLAM in WGS84\n# ts lat lon height\n")
        for i in range(len(slam_ts)):
            ts_int = int(round(slam_ts[i]))
            f.write(f"{ts_int} {lats[i]:.8f} {lons[i]:.8f} {slam_xyz[i,2]:.6f}\n")
    print(f"Saved geo-referenced SLAM: {output_file}")

def save_alignment_parameters(scale, R, T, output_dir):
    path = os.path.join(output_dir, "geo_ref_matrix.txt")
    with open(path, 'w') as f:
        f.write("# Alignment: P_gps = s * R * P_slam + T\n")
        f.write(f"scale: {scale:.8f}\n")
        f.write("rotation_matrix:\n")
        f.write(f"  {R[0,0]: .8f} {R[0,1]: .8f}\n")
        f.write(f"  {R[1,0]: .8f} {R[1,1]: .8f}\n")
        f.write(f"translation_vector: [{T[0]:.8f}, {T[1]:.8f}]\n")
    print(f"Saved alignment params: {path}")

# ======================
# === Map Visualization (GPS Accuracy + SLAM)
# ======================

def create_gps_accuracy_map(gps_file, output_path):
    records = []
    with open(gps_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 4:
                try:
                    unix = int(float(parts[0]))
                    lat = float(parts[1])
                    lon = float(parts[2])
                    records.append({'unix': unix, 'lat': lat, 'lon': lon})
                except:
                    continue
    if not records:
        return
    df = pd.DataFrame(records).sort_values('unix').reset_index(drop=True)
    window = 5
    lat_std = df['lat'].rolling(window, center=True, min_periods=1).std()
    lon_std = df['lon'].rolling(window, center=True, min_periods=1).std()
    df['jitter_m'] = np.sqrt(lat_std**2 + lon_std**2) * 111320
    df['jitter_m'] = df['jitter_m'].fillna(0).clip(upper=5.0)
    good = df['jitter_m'] < 0.5

    m = folium.Map(location=[df['lat'].mean(), df['lon'].mean()], zoom_start=18)
    for i in range(len(df) - 1):
        color = '#0000FF' if good.iloc[i] else '#FF0000'
        folium.PolyLine([(df.iloc[i]['lat'], df.iloc[i]['lon']),
                         (df.iloc[i+1]['lat'], df.iloc[i+1]['lon'])],
                        color=color, weight=5, opacity=0.8).add_to(m)
    legend = '''<div style="position: fixed; bottom: 50px; left: 50px; width: 200px; height: 70px;
                border:2px solid grey; z-index:9999; font-size:14px; background:white; opacity:0.9;">
        &nbsp;<b>GPS Accuracy</b><br>
        &nbsp;<span style="color:#0000FF;">●</span>&nbsp; Good (<50cm)<br>
        &nbsp;<span style="color:#FF0000;">●</span>&nbsp; Warning (≥50cm)
    </div>'''
    m.get_root().html.add_child(folium.Element(legend))
    m.save(output_path)
    print(f"Saved GPS map: {output_path}")

def create_slam_map(slam_file, output_path):
    try:
        df = pd.read_csv(slam_file, sep=r'\s+', comment='#', names=['ts', 'lat', 'lon', 'h'])
        if df.empty:
            return
        m = folium.Map(location=[df['lat'].mean(), df['lon'].mean()], zoom_start=18)
        coords = df[['lat', 'lon']].values.tolist()
        folium.PolyLine(coords, color='green', weight=5, opacity=0.9).add_to(m)
        folium.Marker(coords[0], popup="Start", icon=folium.Icon(color="blue")).add_to(m)
        folium.Marker(coords[-1], popup="End", icon=folium.Icon(color="red")).add_to(m)
        legend = '''<div style="position: fixed; bottom: 50px; left: 50px; width: 200px; height: 50px;
                    border:2px solid grey; z-index:9999; font-size:14px; background:white; opacity:0.9;">
            &nbsp;<b>SLAM Path</b><br>
            &nbsp;<span style="color:green;">●</span>&nbsp; Geo-referenced Path
        </div>'''
        m.get_root().html.add_child(folium.Element(legend))
        m.save(output_path)
        print(f"Saved SLAM map: {output_path}")
    except Exception as e:
        print(f"SLAM map error: {e}")

# ======================
# === Stage 2: Map Matching (from second script)
# ======================

def run_map_matching(input_file, output_dir):
    if not HAS_MAPPYMATCH:
        print("❌ mappymatch not available. Skipping map matching.")
        return

    base = os.path.splitext(os.path.basename(input_file))[0]
    output_txt = os.path.join(output_dir, f"{base}_snapped.txt")
    output_html = os.path.join(output_dir, f"{base}_snapped_map.html")
    manual_html = os.path.join(output_dir, f"{base}_manual_snapped_map.html")

    # Load trace
    df = pd.read_csv(input_file, sep=r'\s+', comment='#', names=["ts", "lat", "lon", "h"])
    trace = Trace.from_dataframe(df, lat_column="lat", lon_column="lon", xy=True)

    # Geofence & OSM map
    geofence = Geofence.from_trace(trace, padding=2000)
    nx_map = NxMap.from_geofence(geofence, network_type=NetworkType.DRIVE)

    # Match
    matcher = LCSSMatcher(nx_map)
    match_result = matcher.match_trace(trace)

    # Snap points to road
    transformer = Transformer.from_crs(nx_map.crs, CRS.from_epsg(4326), always_xy=True)
    snapped = []
    for match in match_result.matches:
        if match.road:
            point = match.coordinate.geom
            road = match.road.geom
            snapped_point = road.interpolate(road.project(point))
        else:
            snapped_point = match.coordinate.geom
        lon, lat = transformer.transform(snapped_point.x, snapped_point.y)
        snapped.append((lat, lon))

    # Update and save
    df['lat'] = [p[0] for p in snapped]
    df['lon'] = [p[1] for p in snapped]
    with open(output_txt, 'w') as f:
        f.write("# timestamp latitude longitude height\n")
        df.to_csv(f, sep=" ", index=False, header=False, float_format="%.6f")
    print(f"✅ Snapped trace saved: {output_txt}")

    # Auto plot
    plot_matches(match_result.matches).save(output_html)
    print(f"✅ Auto map saved: {output_html}")

    # Manual Folium map with distance
    if not snapped:
        return
    df_snapped = pd.DataFrame(snapped, columns=['lat', 'lon'])
    df_snapped['jitter_m'] = np.sqrt(
        df_snapped['lat'].rolling(5, center=True, min_periods=1).std()**2 +
        df_snapped['lon'].rolling(5, center=True, min_periods=1).std()**2
    ) * 111320

    total_km = sum(
        great_circle(snapped[i], snapped[i+1]).kilometers
        for i in range(len(snapped) - 1)
    )
    total_mi = total_km * 0.621371

    m = folium.Map(location=[df_snapped['lat'].mean(), df_snapped['lon'].mean()], zoom_start=16)
    for i in range(len(df_snapped) - 1):
        color = 'blue' if df_snapped['jitter_m'].iloc[i] < 0.5 else 'red'
        folium.PolyLine([(df_snapped.iloc[i]['lat'], df_snapped.iloc[i]['lon']),
                         (df_snapped.iloc[i+1]['lat'], df_snapped.iloc[i+1]['lon'])],
                        color=color, weight=5, opacity=0.8).add_to(m)
    folium.Marker([df_snapped.iloc[0]['lat'], df_snapped.iloc[0]['lon']],
                  popup="Start", icon=folium.Icon(color='green')).add_to(m)
    folium.Marker([df_snapped.iloc[-1]['lat'], df_snapped.iloc[-1]['lon']],
                  popup=f"End<br>Total: {total_km:.2f} km", icon=folium.Icon(color='red')).add_to(m)
    folium.LayerControl().add_to(m)
    m.save(manual_html)
    print(f"✅ Manual snapped map saved: {manual_html}")

# ======================
# === Main Pipeline
# ======================

def main():
    parser = argparse.ArgumentParser(description="GPS–SLAM alignment + optional map matching")
    parser.add_argument("gps_file", help="Input GPS file (timestamp lat lon alt)")
    parser.add_argument("slam_file", help="Input SLAM file (TUM format: ts x y z ...)")
    parser.add_argument("--no-plots", action="store_true", help="Skip matplotlib plots")
    parser.add_argument("--no-mapmatch", action="store_true", help="Skip map matching")
    args = parser.parse_args()

    gps_file = args.gps_file
    slam_file = args.slam_file
    plot_steps = not args.no_plots
    do_mapmatch = not args.no_mapmatch

    output_dir = os.path.dirname(os.path.abspath(gps_file))
    os.makedirs(output_dir, exist_ok=True)

    # === Stage 1: Align ===
    gps_ts, gps_lats, gps_lons, gps_alts = read_gps_file(gps_file)
    slam_ts, slam_xs, slam_ys, slam_zs = read_slam_file(slam_file)
    if len(gps_ts) == 0 or len(slam_ts) == 0:
        raise RuntimeError("Empty input data")

    if plot_steps:
        plot_raw_trajectories(gps_lats, gps_lons, slam_xs, slam_ys, output_dir)

    gps_e, gps_n, _ = convert_wgs84_to_hk1980(gps_lats, gps_lons, gps_alts)
    gps_hk = np.column_stack((gps_e, gps_n))

    if plot_steps:
        plot_converted_gps(gps_e, gps_n, output_dir)

    slam_xy = np.column_stack((slam_xs, slam_ys))
    matched_gps, matched_slam = match_trajectories(gps_ts, gps_hk, slam_ts, slam_xy)
    scale, R, T = align_trajectories_horn(matched_gps, matched_slam)

    save_alignment_parameters(scale, R, T, output_dir)
    aligned_slam = apply_transformation(slam_xy, scale, R, T)
    slam_xyz = np.column_stack((slam_xs, slam_ys, slam_zs))
    geo_slam_file = os.path.join(output_dir, "slam_geo_referenced.txt")
    save_geo_referenced_slam_wgs84_integer_ts(slam_ts, slam_xyz, aligned_slam, geo_slam_file)

    if plot_steps:
        plot_only_georeferenced_slam(aligned_slam, gps_hk, output_dir)

    create_gps_accuracy_map(gps_file, os.path.join(output_dir, "gps.html"))
    create_slam_map(geo_slam_file, os.path.join(output_dir, "slam_geo_referenced.html"))

    errors = np.linalg.norm(matched_gps - apply_transformation(matched_slam, scale, R, T), axis=1)
    print(f"\n📊 Alignment Quality: Mean error = {np.mean(errors):.3f} m")

    # === Stage 2: Map Matching ===
    if do_mapmatch and HAS_MAPPYMATCH:
        print("\n🛣️  Starting map matching...")
        run_map_matching(geo_slam_file, output_dir)
    elif do_mapmatch and not HAS_MAPPYMATCH:
        print("\n⚠️  Map matching skipped: mappymatch not installed.")

    print(f"\n✅ All outputs saved to: {output_dir}")

if __name__ == "__main__":
    main()
