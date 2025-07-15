#!/usr/bin/env python3
"""
Test script to demonstrate COLMAP export functionality for VGGT-SLAM.
This script shows how to use the new COLMAP format export feature.
"""

import os
import argparse
import subprocess
import sys

def main():
    parser = argparse.ArgumentParser(description="Test COLMAP export functionality")
    parser.add_argument("--image_folder", type=str, required=True, 
                       help="Path to folder containing images")
    parser.add_argument("--colmap_output", type=str, default="colmap_output",
                       help="Directory to save COLMAP format output")
    parser.add_argument("--max_points", type=int, default=50000,
                       help="Maximum number of points to export per frame")
    
    args = parser.parse_args()
    
    # Check if image folder exists
    if not os.path.exists(args.image_folder):
        print(f"Error: Image folder {args.image_folder} does not exist")
        sys.exit(1)
    
    # Create output directory
    os.makedirs(args.colmap_output, exist_ok=True)
    
    # Build the command
    cmd = [
        "python", "main.py",
        "--image_folder", args.image_folder,
        "--colmap_output", args.colmap_output,
        "--colmap_max_points", str(args.max_points),
        "--submap_size", "8",  # Smaller for testing
        "--max_loops", "1",
        "--min_disparity", "50",
        "--conf_threshold", "25"
    ]
    
    print("Running VGGT-SLAM with COLMAP export...")
    print(f"Command: {' '.join(cmd)}")
    print()
    
    try:
        # Run the command
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("VGGT-SLAM completed successfully!")
        print(result.stdout)
        
        # Check if COLMAP files were created
        colmap_files = ["cameras.txt", "images.txt", "points3D.txt"]
        for file in colmap_files:
            file_path = os.path.join(args.colmap_output, file)
            if os.path.exists(file_path):
                size = os.path.getsize(file_path)
                print(f"✓ {file} created ({size} bytes)")
            else:
                print(f"✗ {file} not found")
        
        print(f"\nCOLMAP format files saved to: {args.colmap_output}")
        print("You can now open these files in COLMAP or other SfM tools.")
        
    except subprocess.CalledProcessError as e:
        print(f"Error running VGGT-SLAM: {e}")
        print(f"STDOUT: {e.stdout}")
        print(f"STDERR: {e.stderr}")
        sys.exit(1)
    except FileNotFoundError:
        print("Error: Could not find main.py. Make sure you're running this from the VGGT-SLAM directory.")
        sys.exit(1)

if __name__ == "__main__":
    main() 