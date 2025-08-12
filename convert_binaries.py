#!/usr/bin/env python3
"""
Convert .bin files to C header files for inclusion in Arduino projects
"""

import sys
import os

def bin_to_header(bin_file, header_file, var_name):
    """Convert a binary file to a C header file with byte array"""
    
    with open(bin_file, 'rb') as f:
        data = f.read()
    
    with open(header_file, 'w') as f:
        f.write(f"/*\n")
        f.write(f" * Generated from {bin_file}\n")
        f.write(f" * Size: {len(data)} bytes\n")
        f.write(f" */\n\n")
        f.write(f"#pragma once\n\n")
        f.write(f"const unsigned char {var_name}[] = {{\n")
        
        # Write data in rows of 16 bytes
        for i in range(0, len(data), 16):
            f.write("    ")
            for j in range(16):
                if i + j < len(data):
                    f.write(f"0x{data[i + j]:02x}")
                    if i + j < len(data) - 1:
                        f.write(", ")
                else:
                    break
            f.write("\n")
        
        f.write(f"}};\n\n")
        f.write(f"const unsigned int {var_name}_size = {len(data)};\n")

def main():
    # Convert traffic_light_controller.bin
    if os.path.exists("traffic_light_controller.bin"):
        print("Converting traffic_light_controller.bin...")
        bin_to_header("traffic_light_controller.bin", 
                     "include/traffic_light_controller.h",
                     "traffic_light_controller_bin")
        print("Created include/traffic_light_controller.h")
    else:
        print("Warning: traffic_light_controller.bin not found")
    
    # Convert blank_design.bin  
    if os.path.exists("blank_design.bin"):
        print("Converting blank_design.bin...")
        bin_to_header("blank_design.bin",
                     "include/blank_design.h", 
                     "blank_design_bin")
        print("Created include/blank_design.h")
    else:
        print("Warning: blank_design.bin not found")

    # Convert blink.bin
    if os.path.exists("blink.bin"):
        print("Converting blink.bin...")
        bin_to_header("blink.bin",
                      "include/blink.h",
                      "blink_bin")
        print("Created include/blink.h")
    else:
        print("Warning: blink.bin not found")

if __name__ == "__main__":
    main()
