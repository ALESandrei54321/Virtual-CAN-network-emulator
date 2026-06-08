#!/usr/bin/env python3
"""
Create a LittleFS image from a directory of files.
Usage: build_littlefs.py <source_dir> <output_image>
"""
import os
import sys
from littlefs import LittleFS

def create_image(src_dir, out_file):
    # Standard MicroPython LittleFS geometry
    lfs = LittleFS(block_size=4096, block_count=352, read_size=32, prog_size=32, lookahead_size=32, disk_version=0x00020000)
    
    for root, _, files in os.walk(src_dir):
        for f in files:
            # Skip hidden files or pycache
            if f.startswith('.') or '__pycache__' in root:
                continue
                
            src_path = os.path.join(root, f)
            # Make path relative to src_dir
            rel_path = os.path.relpath(src_path, src_dir)
            
            # Create subdirectories in LittleFS if needed
            dirname = os.path.dirname(rel_path)
            if dirname:
                # Walk the path and create dirs
                parts = dirname.split(os.sep)
                current_dir = ''
                for part in parts:
                    current_dir = f"{current_dir}/{part}" if current_dir else part
                    try:
                        lfs.mkdir(current_dir)
                    except FileExistsError:
                        pass

            print(f"Adding {rel_path}...")
            with open(src_path, 'rb') as f_in, lfs.open(rel_path, 'wb') as f_out:
                f_out.write(f_in.read())

    with open(out_file, 'wb') as f_out:
        f_out.write(lfs.context.buffer)
        
    print(f"Created {out_file} ({len(lfs.context.buffer)} bytes)")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: build_littlefs.py <source_dir> <output_image>")
        sys.exit(1)
    
    create_image(sys.argv[1], sys.argv[2])
