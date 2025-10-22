#!/usr/bin/env python3
"""
Simple script to open and display PNG or JPEG XL images
Usage: python3 view_image.py <path_to_image>
"""

import sys
import os
import cv2
import numpy as np

def decode_jxl(jxl_path):
    """Decode a JPEG XL image to numpy array"""
    try:
        import imagecodecs
        # Read the JXL file
        with open(jxl_path, 'rb') as f:
            jxl_data = f.read()
        
        # Decode JPEG XL to numpy array
        # Note: Our JPEG XL images are stored in BGR format (OpenCV native)
        # The decoder returns RGB by default, but our data was encoded as BGR
        # So we need to interpret the decoded RGB data as BGR (no conversion needed)
        img = imagecodecs.jpegxl_decode(jxl_data)
        
        # The image is already in the correct format (BGR data was stored as-is)
        return img
    except ImportError:
        print("Error: 'imagecodecs' library not found.")
        print("Install it with: pip3 install imagecodecs")
        return None
    except Exception as e:
        print(f"Error decoding JPEG XL: {e}")
        return None

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 view_image.py <path_to_image>")
        print("Example: python3 view_image.py ~/simple_captures/image.png")
        print("         python3 view_image.py ~/simple_captures/image.jxl")
        sys.exit(1)
    
    image_path = sys.argv[1]
    
    if not os.path.exists(image_path):
        print(f"Error: File not found: {image_path}")
        sys.exit(1)
    
    # Check file extension
    _, ext = os.path.splitext(image_path)
    ext = ext.lower()
    
    # Read the image based on format
    if ext == '.jxl':
        print(f"Decoding JPEG XL image: {image_path}")
        img = decode_jxl(image_path)
    else:
        print(f"Reading image: {image_path}")
        img = cv2.imread(image_path)
    
    if img is None:
        print(f"Error: Could not read image from {image_path}")
        print("Make sure the file exists and is a valid image.")
        sys.exit(1)
    
    # Display image info
    print(f"Image: {image_path}")
    print(f"Size: {img.shape[1]} x {img.shape[0]}")
    print(f"Channels: {img.shape[2] if len(img.shape) > 2 else 1}")
    print(f"Data type: {img.dtype}")
    
    # Get file size
    file_size = os.path.getsize(image_path)
    if file_size < 1024:
        print(f"File size: {file_size} bytes")
    elif file_size < 1024 * 1024:
        print(f"File size: {file_size / 1024:.2f} KB")
    else:
        print(f"File size: {file_size / (1024 * 1024):.2f} MB")
    
    print("\nPress any key to close the window...")
    
    # Display the image
    cv2.imshow('Image Viewer', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

