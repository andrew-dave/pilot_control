#!/usr/bin/env python3
"""
Batch convert JPEG XL images to PNG
Usage: python3 jxl_to_png.py <folder_path>
"""

import sys
import os
import cv2
import glob
from pathlib import Path

def decode_jxl(jxl_path):
    """Decode a JPEG XL image to numpy array"""
    try:
        import imagecodecs
        # Read the JXL file
        with open(jxl_path, 'rb') as f:
            jxl_data = f.read()
        
        # Decode JPEG XL to numpy array
        # Note: Our JPEG XL images are stored in BGR format (OpenCV native)
        img = imagecodecs.jpegxl_decode(jxl_data)
        
        # The image is already in BGR format
        return img
    except ImportError:
        print("Error: 'imagecodecs' library not found.")
        print("Install it with: pip3 install imagecodecs")
        return None
    except Exception as e:
        print(f"Error decoding JPEG XL: {e}")
        return None

def convert_jxl_to_png(jxl_path, output_path):
    """Convert a single JPEG XL image to PNG"""
    img = decode_jxl(jxl_path)
    
    if img is None:
        return False
    
    # Save as PNG in the same folder
    success = cv2.imwrite(output_path, img)
    
    if success:
        # Get file sizes for comparison
        jxl_size = os.path.getsize(jxl_path)
        png_size = os.path.getsize(output_path)
        ratio = (png_size / jxl_size) * 100 if jxl_size > 0 else 0
        
        print(f"  ✓ {Path(jxl_path).name} -> {Path(output_path).name}")
        print(f"    JXL: {jxl_size:,} bytes | PNG: {png_size:,} bytes | Ratio: {ratio:.1f}%")
        return True
    else:
        print(f"  ✗ Failed to save: {output_path}")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 jxl_to_png.py <folder_path>")
        print("Example: python3 jxl_to_png.py ~/simple_captures")
        print("         python3 jxl_to_png.py /path/to/data/session_20231022_123456")
        sys.exit(1)
    
    folder_path = sys.argv[1]
    
    if not os.path.exists(folder_path):
        print(f"Error: Folder not found: {folder_path}")
        sys.exit(1)
    
    if not os.path.isdir(folder_path):
        print(f"Error: Not a directory: {folder_path}")
        sys.exit(1)
    
    # Find all .jxl files in the folder
    jxl_pattern = os.path.join(folder_path, "*.jxl")
    jxl_files = sorted(glob.glob(jxl_pattern))
    
    if not jxl_files:
        print(f"No .jxl files found in: {folder_path}")
        sys.exit(0)
    
    print(f"Found {len(jxl_files)} JPEG XL images in: {folder_path}")
    print(f"Converting to PNG...\n")
    
    success_count = 0
    fail_count = 0
    
    for jxl_path in jxl_files:
        # Generate output PNG path (same name, different extension)
        base_name = Path(jxl_path).stem
        png_path = os.path.join(folder_path, f"{base_name}.png")
        
        # Convert the image
        if convert_jxl_to_png(jxl_path, png_path):
            success_count += 1
        else:
            fail_count += 1
    
    print(f"\n=== Conversion Complete ===")
    print(f"✓ Successful: {success_count}")
    if fail_count > 0:
        print(f"✗ Failed: {fail_count}")
    print(f"Output folder: {folder_path}")

if __name__ == '__main__':
    main()

