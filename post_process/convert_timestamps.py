import os

def find_gps_file(folder_path):
    """Finds the GPS data file in the specified folder."""
    for filename in os.listdir(folder_path):
        if filename.startswith("gps_data_") and filename.endswith(".txt"):
            return os.path.join(folder_path, filename)
    return None

def find_slam_file(folder_path):
    """Finds the SLAM poses file in the specified folder."""
    for filename in os.listdir(folder_path):
        if filename.startswith("slam_poses_") and filename.endswith(".txt"):
            return os.path.join(folder_path, filename)
    return None

def get_first_timestamp(filepath):
    """Reads the first valid timestamp from a data file."""
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                return int(line.split()[0])
            except (ValueError, IndexError):
                continue
    raise ValueError(f"No valid timestamp found in {filepath}")

def convert_timestamps(slam_file, gps_file, output_file):
    """
    Shifts the timestamps in a SLAM file to align with a GPS file.
    """
    try:
        slam_start_ts = get_first_timestamp(slam_file)
        gps_start_ts = get_first_timestamp(gps_file)
    except ValueError as e:
        print(f"Error: {e}")
        return

    offset = gps_start_ts - slam_start_ts
    print(f"SLAM start: {slam_start_ts}")
    print(f"GPS start:  {gps_start_ts}")
    print(f"Calculated timestamp offset: {offset}")

    with open(slam_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            line = line.strip()
            if not line:
                outfile.write('\n')
                continue
            
            if line.startswith('#'):
                outfile.write(line + '\n')
                continue

            parts = line.split()
            try:
                original_ts = int(parts[0])
                new_ts = original_ts + offset
                parts[0] = str(new_ts)
                outfile.write(' '.join(parts) + '\n')
            except (ValueError, IndexError):
                outfile.write(line + '\n') # Write invalid lines as-is
    
    print(f"Successfully created new SLAM file with aligned timestamps: {output_file}")

def main():
    """
    Main function to find files and run the conversion.
    """
    folder_path = "/media/kodifly/Extreme Pro/2025-11-11-rosbags/isds_1111_135548"
    
    slam_file = find_slam_file(folder_path)
    gps_file = find_gps_file(folder_path)
    output_file = "poses_tum_converted.txt"

    if not slam_file:
        print("Error: No SLAM file (slam_poses_*.txt) found in the current directory.")
        return
    if not gps_file:
        print("Error: No GPS file (gps_data_*.txt) found in the current directory.")
        return

    print(f"Found SLAM file: {slam_file}")
    print(f"Found GPS file:  {gps_file}")
    
    convert_timestamps(slam_file, gps_file, output_file)

if __name__ == "__main__":
    main()