import numpy as np
import os
import struct
import argparse

def parse_pcd_header(file_path):
    """Parse PCD header and return metadata."""
    header = {}
    header_lines = []
    
    with open(file_path, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if not line:  # Empty line
                continue
                
            header_lines.append(line)
            
            # Parse key-value pairs
            if ' ' in line and not line.startswith('#'):
                parts = line.split(None, 1)  # Split on whitespace, max 2 parts
                if len(parts) == 2:
                    key, value = parts
                    header[key] = value
                elif len(parts) == 1:
                    header[parts[0]] = ''
            
            # Check if we've reached the DATA line
            if line.startswith('DATA'):
                if ' ' in line:
                    header['DATA'] = line.split()[1]
                else:
                    header['DATA'] = 'binary'  # default
                header['data_offset'] = f.tell()
                break
    
    return header, header_lines

def read_pcd_data(file_path):
    """Read PCD file preserving all fields."""
    header, header_lines = parse_pcd_header(file_path)
    
    required = ['FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'DATA']
    missing = [k for k in required if k not in header]
    if missing:
        raise ValueError(f"Missing required header fields: {missing}")
    
    fields = header['FIELDS'].split()
    sizes = list(map(int, header['SIZE'].split()))
    types = header['TYPE'].split()
    counts = list(map(int, header['COUNT'].split()))
    width = int(header['WIDTH'])
    height = int(header['HEIGHT'])
    points_count = width * height
    
    dtype_list = []
    for field, size, type_char, count in zip(fields, sizes, types, counts):
        if type_char == 'F':
            np_type = f'f{size}'
        elif type_char == 'U':
            np_type = f'u{size}'
        elif type_char == 'I':
            np_type = f'i{size}'
        else:
            np_type = f'f{size}'
        
        if count == 1:
            dtype_list.append((field, np_type))
        else:
            dtype_list.append((field, np_type, count))
    
    dtype = np.dtype(dtype_list)
    data_type = header['DATA'].lower()
    
    with open(file_path, 'rb') as f:
        f.seek(header['data_offset'])
        
        if data_type == 'ascii':
            data = np.loadtxt(f, dtype=dtype, max_rows=points_count)
        elif data_type == 'binary':
            data = np.fromfile(f, dtype=dtype, count=points_count)
        elif data_type == 'binary_compressed':
            raise NotImplementedError("Compressed PCD format not supported")
        else:
            raise ValueError(f"Unknown DATA type: {data_type}")
    
    return data, fields, header

def write_pcd_with_fields(file_path, combined_data, fields):
    points_count = len(combined_data)
    field_info = []
    for field in fields:
        dtype = combined_data[field].dtype
        if np.issubdtype(dtype, np.floating):
            type_char = 'F'
        elif np.issubdtype(dtype, np.unsignedinteger):
            type_char = 'U'
        elif np.issubdtype(dtype, np.signedinteger):
            type_char = 'I'
        else:
            type_char = 'F'
        
        count = 1 if combined_data[field].ndim == 1 else combined_data[field].shape[1]
        size = dtype.itemsize
        field_info.append({'name': field, 'size': size, 'type': type_char, 'count': count})
    
    with open(file_path, 'wb') as f:
        f.write(b'# .PCD v0.7 - Point Cloud Data file format\n')
        f.write(b'VERSION 0.7\n')
        f.write(f'FIELDS {" ".join([fi["name"] for fi in field_info])}\n'.encode())
        f.write(f'SIZE {" ".join([str(fi["size"]) for fi in field_info])}\n'.encode())
        f.write(f'TYPE {" ".join([fi["type"] for fi in field_info])}\n'.encode())
        f.write(f'COUNT {" ".join([str(fi["count"]) for fi in field_info])}\n'.encode())
        f.write(f'WIDTH {points_count}\n'.encode())
        f.write(b'HEIGHT 1\n')
        f.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {points_count}\n'.encode())
        f.write(b'DATA binary\n')
        combined_data.tofile(f)

def combine_pcds_preserve_all(input_dir, output_file):
    pcd_files = [os.path.join(input_dir, f) for f in os.listdir(input_dir) 
                 if f.endswith('.pcd') and os.path.isfile(os.path.join(input_dir, f))]
    
    if not pcd_files:
        print("No PCD files found in the directory.")
        return
    
    print(f"Reading reference file: {pcd_files[0]}")
    first_data, reference_fields, _ = read_pcd_data(pcd_files[0])
    print(f"Detected fields: {reference_fields}")
    
    all_data = [first_data]
    
    for i, pcd_file in enumerate(pcd_files[1:], 1):
        print(f"Processing ({i+1}/{len(pcd_files)}): {pcd_file}")
        try:
            data, fields, _ = read_pcd_data(pcd_file)
            if fields != reference_fields:
                print(f"  Warning: Field mismatch in {pcd_file}")
                continue
            all_data.append(data)
        except Exception as e:
            print(f"  Error reading {pcd_file}: {e}")
            continue
    
    print(f"\nCombining {len(all_data)} point clouds...")
    combined_data = np.concatenate(all_data)
    print(f"Total points: {len(combined_data)}")
    print(f"Saving to: {output_file}")
    write_pcd_with_fields(output_file, combined_data, reference_fields)
    print("Done!")
    print(f"\nPreserved fields:")
    for field in reference_fields:
        print(f"  - {field}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Combine PCD files while preserving all fields.")
    parser.add_argument("input_dir", type=str, help="Directory containing PCD files to combine.")
    parser.add_argument("output_file", type=str, help="Output PCD file path.")
    
    args = parser.parse_args()
    
    combine_pcds_preserve_all(args.input_dir, args.output_file)
