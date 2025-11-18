import sys

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
    Shifts the timestamps in the SLAM file to align with the GPS file.
    """
    try:
        slam_start_ts = get_first_timestamp(slam_file)
        gps_start_ts = get_first_timestamp(gps_file)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    offset = gps_start_ts - slam_start_ts
    print(f"SLAM start: {slam_start_ts}")
    print(f"GPS start:  {gps_start_ts}")
    print(f"Applying timestamp offset: {offset}")

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
                # Keep invalid lines unchanged
                outfile.write(line + '\n')
    
    print(f"✅ Successfully created aligned SLAM file: {output_file}")

def main():
    if len(sys.argv) != 4:
        print("Usage: python align_timestamps.py <slam_file.txt> <gps_file.txt> <output_file.txt>")
        print("Example: python align_timestamps.py slam_poses.txt gps_data.txt poses_tum_converted.txt")
        sys.exit(1)

    slam_file = sys.argv[1]
    gps_file = sys.argv[2]
    output_file = sys.argv[3]

    for filepath in [slam_file, gps_file]:
        if not os.path.isfile(filepath):
            print(f"Error: File not found: {filepath}")
            sys.exit(1)

    convert_timestamps(slam_file, gps_file, output_file)

if __name__ == "__main__":
    import os
    main()