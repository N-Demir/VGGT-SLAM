# COLMAP Export Feature for VGGT-SLAM

This document describes the COLMAP format export functionality added to VGGT-SLAM.

## Overview

VGGT-SLAM now supports exporting reconstructions in the standard COLMAP format using the official `read_write_model.py` functions, making it compatible with a wide range of Structure-from-Motion (SfM) tools and evaluation frameworks.

## COLMAP Format Files

The export creates three standard COLMAP files in text format:

- **`cameras.txt`** - Camera intrinsic parameters
- **`images.txt`** - Camera poses and image information  
- **`points3D.txt`** - 3D point cloud with colors

## Usage

### Command Line Interface

```bash
# Basic COLMAP export
python main.py --image_folder /path/to/images --colmap_output colmap_reconstruction

# With custom parameters
python main.py \
    --image_folder /path/to/images \
    --colmap_output colmap_reconstruction \
    --colmap_voxel_size 0.005 \
    --colmap_max_points 100000 \
    --submap_size 16 \
    --max_loops 1
```

### Parameters

- `--colmap_output`: Directory to save COLMAP files (required for export)
- `--colmap_max_points`: Maximum number of points to export per frame (default: 100000)

### Gradio Web Interface

The Gradio demo now includes a "Export COLMAP format" checkbox that will create COLMAP files in a `colmap_output/` directory.

### Test Script

Use the provided test script for quick testing:

```bash
python test_colmap_export.py --image_folder /path/to/images --colmap_output test_output
```

## File Format Details

### cameras.txt
```
# Camera list with one line of data per camera:
#   camera_id, model, width, height, params[]
1 SIMPLE_PINHOLE 640 480 585.0 320.0 240.0
```

### images.txt
```
# Image list with two lines of data per image:
#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
#   POINTS2D[] as (X, Y, POINT3D_ID)
1 0.7071068 0.0 0.0 0.7071068 1.0 2.0 3.0 1 frame_001.jpg

2 0.7071068 0.0 0.0 0.7071068 2.0 3.0 4.0 1 frame_002.jpg

```

### points3D.txt
```
# 3D point list with one line of data per point:
#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
1 1.234 2.345 3.456 255 128 64 0.0
2 2.345 3.456 4.567 128 255 64 0.0
```

## Integration with Other Tools

### COLMAP GUI
1. Open COLMAP
2. File → Import model → Import COLMAP format
3. Select the exported directory
4. View the reconstruction

### Evaluation Tools
Many SLAM evaluation tools support COLMAP format:
- EVO (for trajectory evaluation)
- COLMAP's built-in evaluation tools
- Custom evaluation scripts

### Other SfM Tools
- OpenMVS
- MeshLab
- CloudCompare
- Custom reconstruction pipelines

## Technical Details

### Implementation
- Uses official COLMAP `read_write_model.py` functions for format compliance
- Exports in text format for maximum compatibility
- Proper quaternion conversion using `rotmat2qvec()` function
- Standard COLMAP data structures (Camera, Image, Point3D)

### Point Cloud Processing
- Uses the same strategy as `save_framewise_pointclouds`
- Processes point clouds frame-by-frame with confidence filtering
- Preserves original point cloud structure without voxelization
- Applies confidence threshold filtering per frame
- Colors are preserved from original RGB images
- Maximum point limit per frame prevents memory issues

### Camera Parameters
- Uses SIMPLE_PINHOLE camera model
- Intrinsics extracted from VGGT predictions
- Poses converted from SL(4)/Sim(3) to SE(3) format using official COLMAP functions

### Limitations
- No 2D-3D point correspondences (points2D arrays are empty)
- No track information for points
- No reprojection error metrics
- Single camera model assumed
- Text format only (for simplicity and compatibility)

## Example Workflow

1. **Run VGGT-SLAM with COLMAP export:**
   ```bash
   python main.py --image_folder office_images --colmap_output office_colmap
   ```

2. **Open in COLMAP:**
   - Launch COLMAP GUI
   - Import the exported model
   - Visualize and analyze the reconstruction

3. **Further processing:**
   - Dense reconstruction in COLMAP
   - Mesh generation
   - Texture mapping

## Troubleshooting

### Common Issues

**No points exported:**
- Check confidence threshold settings
- Verify point cloud processing completed successfully
- Ensure frames have sufficient valid points

**Large file sizes:**
- Reduce `--colmap_max_points` per frame
- Use confidence filtering
- Check for excessive point density

**Import errors in COLMAP:**
- Verify file format compliance (uses official COLMAP functions)
- Check for missing or corrupted files
- Ensure proper file permissions

### Performance Tips

- Use smaller submap sizes for faster processing
- Adjust max_points per frame based on scene complexity
- Use confidence filtering to remove noise
- Monitor memory usage for large scenes

## Future Improvements

Potential enhancements for the COLMAP export:

- Support for multiple camera models
- 2D-3D point correspondences
- Track information for points
- Reprojection error metrics
- Binary format export option
- Support for different coordinate systems 