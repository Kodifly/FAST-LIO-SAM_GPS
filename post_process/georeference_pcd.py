# georeference_pcd.py
import numpy as np
import os
import sys

def read_georeferencing_params(params_file_path):
    """Reads the geo-referencing parameters from the text file."""
    with open(params_file_path, 'r') as f:
        lines = [line.strip() for line in f.readlines() if not line.startswith('#')]

    if len(lines) < 4:
        raise ValueError("Params file seems to be malformed or missing data.")

    try:
        scale = float(lines[0])
        r11, r12 = map(float, lines[1].split())
        r21, r22 = map(float, lines[2].split())
        t1, t2 = map(float, lines[3].split())

        rotation_matrix = np.array([[r11, r12],
                                   [r21, r22]])
        translation_vector = np.array([t1, t2])

        return scale, rotation_matrix, translation_vector
    except (ValueError, IndexError) as e:
        raise ValueError(f"Error parsing parameters from {params_file_path}: {e}")

def parse_pcd_header(file_path):
    """Parses the PCD header to get metadata."""
    header = {}
    with open(file_path, 'rb') as f:
        line_num = 0
        while True:
            line = f.readline().decode('utf-8').strip()
            if not line:
                continue
            if line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0].rstrip(':')
            value = parts[1:] if len(parts) > 1 else (parts[1] if len(parts) == 2 else "")
            header[key.lower()] = value
            line_num += 1
            if key.lower() == "data":
                header['data'] = value[0]
                header['header_end_pos'] = f.tell()
                break
            if line_num > 100:
                raise ValueError("Could not find 'DATA' line in PCD header within first 100 lines.")
    return header

def georeference_pcd_numpy(input_pcd_path, params_file_path, output_pcd_path):
    """
    Applies geo-referencing transformation to a PCD file using numpy,
    aiming to preserve all fields including intensity.
    Supports ASCII and simple binary PCD formats.
    """
    if not os.path.exists(input_pcd_path):
        print(f"Error: Input PCD file '{input_pcd_path}' not found.")
        return False

    print(f"Parsing PCD header: {input_pcd_path}")
    try:
        header = parse_pcd_header(input_pcd_path)
    except Exception as e:
        print(f"Error parsing PCD header: {e}")
        return False

    print("Header Info:")
    for k, v in header.items():
        print(f"  {k}: {v}")

    try:
        version = header.get('version', ['0.7'])[0]
        fields = [f.lower() for f in header['fields']]
        types_list = header['type']
        sizes_list = [int(s) for s in header['size']]
        counts_list = [int(c) for c in header.get('count', [1] * len(fields))]

        width = int(header['width'][0])
        height = int(header['height'][0])
        points = int(header['points'][0])
        data_format = header['data'].lower()

        print(f"\nPCD Details:")
        print(f"  Version: {version}")
        print(f"  Fields: {fields}")
        print(f"  Types: {types_list}")
        print(f"  Sizes: {sizes_list}")
        print(f"  Counts: {counts_list}")
        print(f"  Width: {width}, Height: {height}")
        print(f"  Total Points: {points}")
        print(f"  Data Format: {data_format}")

        try:
            x_idx = fields.index('x')
            y_idx = fields.index('y')
        except ValueError:
            print("Error: 'x' and/or 'y' fields not found in PCD header.")
            return False

        pcd_to_numpy_dtype = {
            'I': 'int32', 'U': 'uint32', 'F': 'float32', 'D': 'float64'
        }
        dtype_desc = []
        for i, field_name in enumerate(fields):
             pcd_type_char = types_list[i][0]
             size = sizes_list[i]
             count = counts_list[i]

             if pcd_type_char in pcd_to_numpy_dtype:
                 numpy_base_type = pcd_to_numpy_dtype[pcd_type_char]
             elif pcd_type_char == 'F' and size == 4:
                 numpy_base_type = 'float32'
             elif pcd_type_char == 'F' and size == 8:
                 numpy_base_type = 'float64'
             elif pcd_type_char == 'U' and size == 4:
                 numpy_base_type = 'uint32'
             elif pcd_type_char == 'I' and size == 4:
                 numpy_base_type = 'int32'
             else:
                 print(f"Warning: Unhandled PCD type '{pcd_type_char}' with size {size} for field '{field_name}'. Defaulting to float32 view for transformation.")
                 numpy_base_type = 'float32'

             if count == 1:
                 dtype_desc.append((field_name, numpy_base_type))
             else:
                 dtype_desc.append((field_name, numpy_base_type, (count,)))

        print(f"  Inferred NumPy dtype: {dtype_desc}")

    except (KeyError, IndexError, ValueError) as e:
        print(f"Error extracting information from PCD header: {e}")
        return False

    print(f"\nReading geo-referencing parameters from: {params_file_path}")
    try:
        scale, rotation_2d, translation_2d = read_georeferencing_params(params_file_path)
    except Exception as e:
        print(f"Error reading geo-referencing parameters: {e}")
        return False

    print(f"  Scale: {scale}")
    print(f"  Rotation Matrix:\n{rotation_2d}")
    print(f"  Translation Vector (2D): {translation_2d}")

    print(f"\nReading point cloud data ({data_format})...")
    try:
        if data_format == 'ascii':
            with open(input_pcd_path, 'r') as f_handle:
                actual_header_lines = 0
                f_handle.seek(0)
                while True:
                    h_line = f_handle.readline()
                    if not h_line or h_line.startswith("DATA"):
                        actual_header_lines += 1
                        break
                    if h_line.startswith("#"): continue
                    actual_header_lines += 1

                print(f"Skipping {actual_header_lines} header lines.")

                try:
                    data_array_2d = np.genfromtxt(
                        f_handle,
                        delimiter=' ',
                        skip_header=0,
                        invalid_raise=False,
                        filling_values=np.nan,
                        loose=True
                    )
                except Exception as gen_err:
                    print(f"Error using np.genfromtxt: {gen_err}")
                    raise

                if data_array_2d.ndim == 1:
                    data_array_2d = data_array_2d.reshape(1, -1)
                if data_array_2d.size == 0:
                    raise ValueError("No data points could be read from the ASCII section.")
                if data_array_2d.shape[1] != len(fields):
                     f_handle.seek(0)
                     for _ in range(actual_header_lines): f_handle.readline()
                     line_num = actual_header_lines + 1
                     problematic_line = ""
                     for line in f_handle:
                         parts = line.strip().split()
                         if len(parts) != len(fields):
                             problematic_line = line.strip()
                             break
                         line_num += 1

                     raise ValueError(f"Mismatch between header fields ({len(fields)}) and data columns. "
                                      f"Expected {len(fields)} columns, got {data_array_2d.shape[1]} columns. "
                                      f"First problematic line (around {line_num}): '{problematic_line}'")

                num_points_read = data_array_2d.shape[0]
                print(f"Read raw data array with shape {data_array_2d.shape}.")

                data_array = np.empty(num_points_read, dtype=dtype_desc)
                for i, field_name in enumerate(fields):
                    data_array[field_name] = data_array_2d[:, i]

        elif data_format in ('binary', 'binary_compressed'):
            if data_format == 'binary_compressed':
                print("Warning: 'binary_compressed' format detected. This script handles simple 'binary'. It might not work correctly.")

            with open(input_pcd_path, 'rb') as f:
                f.seek(header['header_end_pos'])
                bytes_per_point = sum(s * c for s, c in zip(sizes_list, counts_list))
                raw_data = f.read()
                data_array = np.frombuffer(raw_data, dtype=dtype_desc, count=points)

        else:
            print(f"Error: Unsupported PCD data format '{data_format}'.")
            return False

        if len(data_array) != points:
            print(f"Warning: Number of points read ({len(data_array)}) does not match header points count ({points}). Using number of points read.")

        print(f"Successfully read {len(data_array)} points.")

    except Exception as e:
        print(f"Error reading PCD data: {e}")
        try:
            with open(input_pcd_path, 'r') as f_debug:
                 lines = f_debug.readlines()
                 start_line = max(0, len(header) - 2)
                 end_line = min(len(lines), start_line + 10)
                 print(f"Context around data section (lines {start_line+1}-{end_line}):")
                 for i in range(start_line, end_line):
                     print(f"  Line {i+1}: {repr(lines[i])}")
        except Exception as debug_err:
            print(f"Could not read file for debugging context: {debug_err}")
        return False

    print("\nApplying geo-referencing transformation to X and Y...")
    try:
        x_local = data_array['x']
        y_local = data_array['y']
        xy_local = np.vstack((x_local, y_local)).T
        xy_geo = (xy_local @ rotation_2d.T) * scale + translation_2d
        data_array['x'] = xy_geo[:, 0]
        data_array['y'] = xy_geo[:, 1]
        print("Transformation applied successfully.")
    except Exception as e:
        print(f"Error applying transformation: {e}")
        return False

    print(f"\nWriting geo-referenced PCD to: {output_pcd_path}")
    try:
        with open(output_pcd_path, 'wb') as f:
            with open(input_pcd_path, 'r', encoding='utf-8') as f_in:
                 header_lines = []
                 while True:
                     line = f_in.readline()
                     if line.startswith("DATA"):
                         header_lines.append(line)
                         break
                     header_lines.append(line)
            for line in header_lines:
                f.write(line.encode('utf-8'))

            if data_format == 'ascii':
                field_names = [f for f in fields]
                # np.savetxt handles structured arrays better this way
                format_strings = []
                for name, dtype_tuple in data_array.dtype.fields.items():
                    dtype_str = dtype_tuple[0].name
                    if 'float' in dtype_str:
                        format_strings.append('%.6f') # Adjust precision as needed
                    elif 'int' in dtype_str:
                        format_strings.append('%d')
                    else:
                        format_strings.append('%s')
                np.savetxt(f, data_array, fmt=' '.join(format_strings))

            elif data_format in ('binary', 'binary_compressed'):
                data_array.tofile(f)

        print("Geo-referenced PCD written successfully.")
        print(f"Output file: {output_pcd_path}")
        return True

    except Exception as e:
        print(f"Error writing geo-referenced PCD: {e}")
        return False

# --- Example Usage ---
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python georeference_pcd.py <input_pcd_file.pcd> <params_file.txt> <output_pcd_file.pcd>")
        sys.exit(1)

    input_pcd_file = sys.argv[1]
    georeferencing_params_file = sys.argv[2]
    output_pcd_file = sys.argv[3]

    success = georeference_pcd_numpy(input_pcd_file, georeferencing_params_file, output_pcd_file)
    if success:
        print("\nPCD geo-referencing completed successfully.")
    else:
        print("\nPCD geo-referencing failed.")
        sys.exit(1)
