#!/usr/bin/env python3

import os
import numpy as np
import argparse

# Fixed field names
# FIELD_NAMES = ["x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"]
FIELD_NAMES = ["x", "y", "z", "intensity"]

def bin_to_pcd(bin_file, pcd_file, field_names):
    """
    Convert a .bin file (binary format) to a .pcd file (ASCII format).
    """
    try:
        # Define the dtype for the structured NumPy array
        dtype = [(field, 'f4') for field in field_names]  # All fields are float32 (4 bytes)

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
            f.write(f"COUNT {' '.join(['1'] * len(field_names))}\n")
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

def batch_convert_bin_to_pcd(input_dir, output_dir):
    """
    Batch convert all .bin files in the input directory to .pcd files in the output directory,
    processing them in sequence based on file names.
    """
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Get all .bin files in the input directory and sort them by name
    bin_files = [f for f in os.listdir(input_dir) if f.endswith(".bin")]
    bin_files.sort()  # Sort files alphabetically/numerically

    # Process each .bin file in sequence
    for bin_file_name in bin_files:
        bin_file_path = os.path.join(input_dir, bin_file_name)
        pcd_file_name = os.path.splitext(bin_file_name)[0] + ".pcd"
        pcd_file_path = os.path.join(output_dir, pcd_file_name)

        # Convert the .bin file to .pcd
        bin_to_pcd(bin_file_path, pcd_file_path, FIELD_NAMES)

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Batch convert .bin files to .pcd files.")
    parser.add_argument("--input_dir", required=True, help="Directory containing .bin files.")
    parser.add_argument("--output_dir", required=True, help="Directory to save .pcd files.")

    # Parse arguments
    args = parser.parse_args()

    # Extract arguments
    input_dir = args.input_dir
    output_dir = args.output_dir

    # Perform batch conversion
    batch_convert_bin_to_pcd(input_dir, output_dir)

if __name__ == "__main__":
    main()