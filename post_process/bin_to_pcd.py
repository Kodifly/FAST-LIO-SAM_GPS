#!/usr/bin/env python3

import os
import numpy as np

def bin_to_pcd(bin_file, pcd_file, field_names):
    """
    Convert a .bin file (binary format) to a .pcd file (ASCII format).
    The field names and their order must be provided.
    """
    try:
        # Define the dtype for the structured NumPy array
        dtype = [(field, 'f4') for field in field_names]  # Assume all fields are float32 (4 bytes)

        # Read the binary file into a structured NumPy array
        data = np.fromfile(bin_file, dtype=dtype)

        # Save as .pcd file in ASCII format
        with open(pcd_file, "w") as f:
            # Write the PCD header
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write(f"FIELDS {' '.join(field_names)}\n")
            f.write(f"SIZE {' '.join(['4'] * len(field_names))}\n")  # All fields are float32 (4 bytes)
            f.write(f"TYPE {' '.join(['F'] * len(field_names))}\n")  # All fields are float
            f.write(f"COUNT {' '.join(['1'] * len(field_names))}\n")  # One value per field
            f.write(f"WIDTH {len(data)}\n")  # Number of points
            f.write("HEIGHT 1\n")  # Unorganized point cloud
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")  # Default viewpoint
            f.write(f"POINTS {len(data)}\n")  # Number of points
            f.write("DATA ascii\n")

            # Write the point cloud data
            for point in data:
                line = " ".join([f"{point[field]:.6f}" for field in field_names])
                f.write(line + "\n")

        print(f"Converted {bin_file} to {pcd_file}")

    except Exception as e:
        print(f"Error converting .bin to .pcd: {e}")

if __name__ == "__main__":
    # Input .bin file
    bin_file = "fast_lio_sam/data/lidar/bin/2323.390806.bin"
    
    # Output .pcd file
    pcd_file = "fast_lio_sam/data/lidar/bin/2323.390806.pcd"

    # Field names (must match the fields in the .bin file)
    # field_names = ["x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"]  # Update as needed x y z intensity t reflectivity ring ambient range
    field_names = ["x", "y", "z", "intensity"]  # Update as needed x y z intensity t reflectivity ring ambient range

    bin_to_pcd(bin_file, pcd_file, field_names)