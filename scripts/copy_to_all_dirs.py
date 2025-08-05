#!/usr/bin/env python3

import sys
import os
import shutil
import re

def main():
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <source_file> <destination_directory> <C> <D>")
        sys.exit(1)

    source_file = sys.argv[1]
    dest_dir = sys.argv[2]
    try:
        c = int(sys.argv[3])
        d = int(sys.argv[4])
    except ValueError:
        print("C and D must be integers.")
        sys.exit(1)

    if c > d:
        print("C should be less than or equal to D.")
        sys.exit(1)

    if not os.path.isfile(source_file):
        print(f"Source file '{source_file}' does not exist.")
        sys.exit(1)
    if not os.path.isdir(dest_dir):
        print(f"Destination '{dest_dir}' is not a directory.")
        sys.exit(1)

    pattern = re.compile(r'^(\d+)_\w+$')
    for root, dirs, files in os.walk(dest_dir):
        basename = os.path.basename(root)
        match = pattern.match(basename)
        if match:
            xxxx = int(match.group(1).lstrip('0') or '0')
            if c <= xxxx <= d:
                dest_path = os.path.join(root, os.path.basename(source_file))
                try:
                    shutil.copy2(source_file, dest_path)
                    print(f"Copied to {dest_path}")
                except Exception as e:
                    print(f"Failed to copy to {dest_path}: {e}")

if __name__ == "__main__":
    main()