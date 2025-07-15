import viser
import numpy as np
from typing import Dict, List, Optional, Tuple
import viser.transforms as viser_tf
import os
import argparse
from pathlib import Path
import cv2

# Import COLMAP reading functions
from read_write_model import read_model, Camera, Image, Point3D, qvec2rotmat


class COLMAPViewer:
    def __init__(self, port: int = 8080):
        print(f"Starting COLMAP viser server on port {port}")

        self.server = viser.ViserServer(host="0.0.0.0", port=port)
        self.server.gui.configure_theme(titlebar_content=None, control_layout="collapsible", show_logo=False)

        # GUI controls
        self.gui_show_cameras = self.server.gui.add_checkbox(
            "Show Cameras",
            initial_value=True,
        )
        self.gui_show_points = self.server.gui.add_checkbox(
            "Show Point Cloud",
            initial_value=True,
        )
        self.gui_point_size = self.server.gui.add_slider(
            "Point Size",
            min=0.001,
            max=0.1,
            step=0.001,
            initial_value=0.001,
        )
        self.gui_camera_scale = self.server.gui.add_slider(
            "Camera Scale",
            min=0.01,
            max=1.0,
            step=0.01,
            initial_value=0.1,
        )

        # Store handles
        self.camera_frames: List[viser.FrameHandle] = []
        self.camera_frustums: List[viser.CameraFrustumHandle] = []
        self.point_cloud_handle: Optional[viser.PointCloudHandle] = None

    def load_colmap_data(self, colmap_dir: str, images_dir: Optional[str] = None) -> Tuple[Dict, Dict, Dict]:
        """
        Load COLMAP data from directory.
        
        Args:
            colmap_dir: Directory containing COLMAP files (cameras.txt, images.txt, points3D.txt)
            images_dir: Directory containing the actual image files (optional)
            
        Returns:
            Tuple of (cameras, images, points3D) dictionaries
        """
        colmap_path = Path(colmap_dir)
        print(f"Loading COLMAP data from {colmap_path}")
        
        # Check if binary or text format
        if (colmap_path / "cameras.bin").exists():
            result = read_model(str(colmap_path), ext=".bin")
        else:
            result = read_model(str(colmap_path), ext=".txt")

        if result is None:
            raise ValueError(f"Could not read COLMAP model from {colmap_path}")
        
        cameras, images, points3D = result
        
        print(f"Loaded {len(cameras)} cameras, {len(images)} images, {len(points3D)} 3D points")
        
        return cameras, images, points3D

    def visualize_cameras(self, cameras: Dict, images: Dict, images_dir: Optional[str] = None):
        """
        Visualize camera poses and frustums.
        
        Args:
            cameras: Dictionary of camera parameters
            images: Dictionary of image poses
            images_dir: Directory containing image files for frustum visualization
        """
        print("Visualizing cameras...")
        
        # Clear existing camera visualizations by setting visibility to False
        for frame in self.camera_frames:
            frame.visible = False
        for frustum in self.camera_frustums:
            frustum.visible = False
        self.camera_frames.clear()
        self.camera_frustums.clear()
        
        images_path = Path(images_dir) if images_dir else None
        
        for image_id, image in images.items():
            # Get camera parameters
            camera = cameras[image.camera_id]
            
            # Convert quaternion to rotation matrix
            R_cam2world = qvec2rotmat(image.qvec)
            t_cam2world = image.tvec
            
            # Create 4x4 transformation matrix
            T_cam2world = np.eye(4)
            T_cam2world[:3, :3] = R_cam2world
            T_cam2world[:3, 3] = t_cam2world
            
            # Create SE3 transform for viser
            T_world_camera = viser_tf.SE3.from_matrix(T_cam2world)
            
            # Add coordinate frame
            frame_name = f"camera_{image_id}"
            frame_axis = self.server.scene.add_frame(
                frame_name,
                wxyz=T_world_camera.rotation().wxyz,
                position=T_world_camera.translation(),
                axes_length=0.05 * self.gui_camera_scale.value,
                axes_radius=0.002,
                origin_radius=0.002,
            )
            frame_axis.visible = self.gui_show_cameras.value
            self.camera_frames.append(frame_axis)
            
            # Add camera frustum
            frustum_name = f"{frame_name}/frustum"
            
            # Calculate FOV from camera parameters
            if camera.model == "SIMPLE_PINHOLE":
                fx, cx, cy = camera.params
                fy = fx  # Assume square pixels
            elif camera.model == "PINHOLE":
                fx, fy, cx, cy = camera.params
            else:
                # For other models, use default values
                fx, fy = camera.params[0], camera.params[1]
                cx, cy = camera.params[2], camera.params[3]
            
            # Calculate FOV
            fov = 2 * np.arctan2(camera.height / 2, fy)
            aspect = camera.width / camera.height
            
            # Load image for frustum if available
            frustum_image = None
            if images_path:
                image_path = images_path / image.name
                if image_path.exists():
                    try:
                        img = cv2.imread(str(image_path))
                        if img is not None:
                            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            frustum_image = img
                    except Exception as e:
                        print(f"Warning: Could not load image {image_path}: {e}")
            
            # Create frustum
            frustum = self.server.scene.add_camera_frustum(
                frustum_name,
                fov=fov,
                aspect=aspect,
                scale=0.05 * self.gui_camera_scale.value,
                image=frustum_image,
                line_width=2.0,
            )
            frustum.visible = self.gui_show_cameras.value
            self.camera_frustums.append(frustum)

    def visualize_point_cloud(self, points3D: Dict):
        """
        Visualize 3D point cloud.
        
        Args:
            points3D: Dictionary of 3D points
        """
        print("Visualizing point cloud...")
        
        # Hide existing point cloud by setting visibility to False
        if self.point_cloud_handle is not None:
            self.point_cloud_handle.visible = False
        
        if not points3D:
            print("No 3D points to visualize")
            return
        
        # Extract points and colors
        points = []
        colors = []
        
        for point_id, point3D in points3D.items():
            points.append(point3D.xyz)
            # COLMAP stores RGB as integers 0-255
            colors.append(point3D.rgb / 255.0)  # Normalize to 0-1
        
        points = np.array(points)
        colors = np.array(colors)
        
        print(f"Visualizing {len(points)} points")
        
        # Add point cloud to scene
        self.point_cloud_handle = self.server.scene.add_point_cloud(
            name="colmap_points",
            points=points,
            colors=colors,
            point_size=self.gui_point_size.value,
            point_shape="circle",
        )
        self.point_cloud_handle.visible = self.gui_show_points.value

    def load_and_visualize(self, colmap_dir: str, images_dir: Optional[str] = None):
        """
        Load COLMAP data and visualize everything.
        
        Args:
            colmap_dir: Directory containing COLMAP files
            images_dir: Directory containing image files (optional)
        """
        try:
            # Load COLMAP data
            cameras, images, points3D = self.load_colmap_data(colmap_dir, images_dir)
            
            # Visualize cameras
            self.visualize_cameras(cameras, images, images_dir)
            
            # Visualize point cloud
            self.visualize_point_cloud(points3D)
            
            print("COLMAP visualization complete!")
            
        except Exception as e:
            print(f"Error loading COLMAP data: {e}")
            import traceback
            traceback.print_exc()

    def update_visibility(self):
        """Update visibility of all elements based on GUI controls."""
        # Update camera visibility
        for frame in self.camera_frames:
            frame.visible = self.gui_show_cameras.value
        for frustum in self.camera_frustums:
            frustum.visible = self.gui_show_cameras.value
        
        # Update point cloud visibility
        if self.point_cloud_handle is not None:
            self.point_cloud_handle.visible = self.gui_show_points.value

    def update_point_size(self):
        """Update point cloud size."""
        if self.point_cloud_handle is not None:
            self.point_cloud_handle.point_size = self.gui_point_size.value

    def update_camera_scale(self):
        """Update camera scale."""
        scale = self.gui_camera_scale.value
        for i, frame in enumerate(self.camera_frames):
            frame.axes_length = 0.05 * scale
        for i, frustum in enumerate(self.camera_frustums):
            frustum.scale = 0.05 * scale


def main():
    parser = argparse.ArgumentParser(description="COLMAP Viewer for Viser")
    parser.add_argument("--colmap_dir", type=str, required=True, 
                       help="Directory containing COLMAP files (cameras.txt/images.txt/points3D.txt or .bin files)")
    parser.add_argument("--images_dir", type=str, default=None,
                       help="Directory containing image files for frustum visualization")
    parser.add_argument("--port", type=int, default=8080,
                       help="Port for viser server")
    
    args = parser.parse_args()
    
    # Create viewer
    viewer = COLMAPViewer(port=args.port)
    
    # Load and visualize data
    viewer.load_and_visualize(args.colmap_dir, args.images_dir)
    
    # Set up GUI callbacks
    viewer.gui_show_cameras.on_update(lambda _: viewer.update_visibility())
    viewer.gui_show_points.on_update(lambda _: viewer.update_visibility())
    viewer.gui_point_size.on_update(lambda _: viewer.update_point_size())
    viewer.gui_camera_scale.on_update(lambda _: viewer.update_camera_scale())
    
    print(f"COLMAP Viewer running at http://localhost:{args.port}")
    print("Press Ctrl+C to exit")
    
    try:
        # Keep the server running
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down COLMAP Viewer...")


if __name__ == "__main__":
    main() 