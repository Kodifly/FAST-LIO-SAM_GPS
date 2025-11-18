import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # For 3D plotting

def parse_gps_file(filename):
    """Parses the GPS data file."""
    timestamps = []
    latitudes = []
    longitudes = []
    altitudes = []

    with open(filename, 'r') as f:
        for line in f:
            if line.strip(): # Skip empty lines
                parts = line.strip().split()
                if len(parts) == 4: # Ensure line has timestamp, lat, lon, alt
                    try:
                        ts = int(parts[0]) # Timestamp might be large, keep as int or convert if needed
                        lat = float(parts[1])
                        lon = float(parts[2])
                        alt = float(parts[3])
                        # Basic check for potentially valid coordinates (Hong Kong-ish based on your data)
                        # Adjust ranges if your data is elsewhere
                        # if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                        #     print(f"Warning: Skipping line with suspicious coordinates: {line}")
                        #     continue
                        timestamps.append(ts)
                        latitudes.append(lat)
                        longitudes.append(lon)
                        altitudes.append(alt)
                    except ValueError:
                        print(f"Warning: Skipping line due to conversion error: {line}")
                        continue
                else:
                    print(f"Warning: Skipping line due to unexpected format: {line}")
                    continue
    # Sort data by timestamp if needed (though your file seems mostly ordered)
    # data = sorted(zip(timestamps, latitudes, longitudes, altitudes))
    # timestamps, latitudes, longitudes, altitudes = zip(*data) if data else ([], [], [], [])
    return np.array(timestamps), np.array(latitudes), np.array(longitudes), np.array(altitudes)


# --- Main Script ---
filename = '/home/kodifly/ssd1/indo_0828/lane1/demo/gps_000.txt' # Replace with your file path
timestamps, latitudes, longitudes, altitudes = parse_gps_file(filename)

if len(latitudes) > 0:
    print(f"Loaded {len(latitudes)} GPS points.")

    # --- 2D Ground Track Plot (Longitude vs Latitude) ---
    plt.figure(figsize=(10, 8))
    plt.plot(longitudes, latitudes, marker='o', linestyle='-', markersize=2, linewidth=0.8)
    plt.xlabel('Longitude (degrees)')
    plt.ylabel('Latitude (degrees)')
    plt.title('GPS Trajectory - Ground Track')
    plt.grid(True, alpha=0.5)
    # Ensure equal aspect ratio for geographic accuracy
    plt.axis('equal')
    # Optional: Zoom in to the data range
    margin = 0.0001 # Adjust margin as needed
    plt.xlim(longitudes.min() - margin, longitudes.max() + margin)
    plt.ylim(latitudes.min() - margin, latitudes.max() + margin)
    plt.tight_layout()
    plt.show()


    # --- Altitude vs Time Plot ---
    if len(timestamps) == len(altitudes):
        plt.figure(figsize=(12, 5))
        # Convert timestamps to relative seconds for easier viewing (optional)
        rel_time_sec = (timestamps - timestamps[0]) / 100.0 # Assuming timestamps are in 100s of ns or similar
        plt.plot(rel_time_sec, altitudes, marker='.', linestyle='-')
        plt.xlabel('Relative Time (seconds)')
        plt.ylabel('Altitude (m)')
        plt.title('GPS Altitude Profile')
        plt.grid(True, alpha=0.5)
        plt.tight_layout()
        plt.show()
    else:
         print("Warning: Timestamp and altitude arrays have different lengths. Skipping altitude plot.")


    # --- 3D Trajectory Plot ---
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory path
    # Note: Z-axis usually represents height, so altitude is used directly.
    # Depending on scale, you might want to exaggerate the Z-axis.
    ax.plot(longitudes, latitudes, altitudes, label='GPS Trajectory')
    ax.set_xlabel('Longitude (degrees)')
    ax.set_ylabel('Latitude (degrees)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D GPS Trajectory')
    ax.legend()
    plt.tight_layout()
    plt.show()

    # --- Optional: 2D Projection with Altitude as Color ---
    plt.figure(figsize=(10, 8))
    scatter = plt.scatter(longitudes, latitudes, c=altitudes, cmap='viridis', s=5)
    plt.colorbar(scatter, label='Altitude (m)')
    plt.xlabel('Longitude (degrees)')
    plt.ylabel('Latitude (degrees)')
    plt.title('GPS Trajectory - Ground Track (Colored by Altitude)')
    plt.grid(True, alpha=0.5)
    plt.axis('equal')
    margin = 0.0001
    plt.xlim(longitudes.min() - margin, longitudes.max() + margin)
    plt.ylim(latitudes.min() - margin, latitudes.max() + margin)
    plt.tight_layout()
    plt.show()



else:
    print("No valid GPS data found in the file.")
