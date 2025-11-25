# add_timestamp_offset.py

input_file = "poses_tum_converted.txt"
output_file = "poses_tum_converted_offset.txt"
offset = 80000400

with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            # Write comment or empty lines as-is
            outfile.write(line)
            continue

        parts = line.split()
        if len(parts) < 8:
            # Not a valid data line; write as-is
            outfile.write(line)
            continue

        try:
            timestamp = int(parts[0])
            new_timestamp = timestamp + offset
            parts[0] = str(new_timestamp)
            outfile.write(' '.join(parts) + '\n')
        except ValueError:
            # If timestamp is not an integer, write line as-is
            outfile.write(line)
